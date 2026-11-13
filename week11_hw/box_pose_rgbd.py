#!/usr/bin/env python3
"""Estimate box pose from yellow corners, a white corner mark, and RGB-D.

Fit the visible face plane, intersect corner rays, and infer a right-handed
box frame. Publish a camera-frame pose and a short-lived RViz cube marker.
"""
import time
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker

# Portrait orientation: marked corner is the top-left of the visible face.
BOX_WIDTH = 0.325
BOX_HEIGHT = 0.420
BOX_DEPTH = 0.050


class BoxPoseRGBD(Node):
    def __init__(self):
        super().__init__('eriche_box_pose')
        self.bridge = CvBridge()
        self.info = None
        self.last_log = 0.0
        self.pose_pub = self.create_publisher(PoseStamped, '/eriche/target/pose_camera', 10)
        self.marker_pub = self.create_publisher(Marker, '/eriche/target/box_marker', 10)
        self.debug_pub = self.create_publisher(Image, '/eriche/target/pose_debug', 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.on_info,
            qos_profile_sensor_data)
        self.rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw',
                                  qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw',
            qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.process)

    def on_info(self, message):
        self.info = message

    def estimate(self, frame, depth, encoding):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, (18, 70, 60), (40, 255, 255))
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 1000:
                continue
            quad = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)
            if len(quad) == 4 and cv2.isContourConvex(quad):
                candidates.append((area, quad))
        if not candidates:
            raise ValueError('No yellow rectangle')
        _, quad = max(candidates, key=lambda item: item[0])
        corners = quad.reshape(4, 2).astype(np.float64)
        center = corners.mean(axis=0)
        angles = np.arctan2(corners[:, 1]-center[1], corners[:, 0]-center[0])
        corners = corners[np.argsort(angles)]

        # A unique white patch near one corner resolves rectangle orientation.
        inside = np.zeros(frame.shape[:2], np.uint8)
        cv2.fillConvexPoly(inside, corners.astype(np.int32), 255)
        white = cv2.inRange(hsv, (0, 0, 150), (179, 70, 255))
        white = cv2.bitwise_and(white, inside)
        blobs, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        diagonal = np.linalg.norm(np.ptp(corners, axis=0))
        marks = []
        for blob in blobs:
            area = cv2.contourArea(blob)
            moments = cv2.moments(blob)
            if area < 8 or moments['m00'] <= 0:
                continue
            point = np.array([moments['m10']/moments['m00'],
                              moments['m01']/moments['m00']])
            distances = np.linalg.norm(corners-point, axis=1)
            index = int(np.argmin(distances))
            if distances[index] < 0.23 * diagonal:
                marks.append((area, index, point))
        if not marks:
            raise ValueError('No white corner marker inside yellow border')
        _, index, white_point = max(marks, key=lambda item: item[0])
        corners = np.roll(corners, -index, axis=0)

        # Avoid yellow tape and background by sampling the inner face.
        inner = center + 0.65 * (corners-center)
        face_mask = np.zeros(depth.shape, np.uint8)
        cv2.fillConvexPoly(face_mask, inner.astype(np.int32), 255)
        if encoding == '16UC1':
            scale = 0.001
        elif encoding == '32FC1':
            scale = 1.0
        else:
            raise ValueError(f'Unsupported depth: {encoding}')
        valid = (face_mask > 0) & np.isfinite(depth) & (depth > 0)
        vv, uu = np.nonzero(valid)
        if len(uu) < 100:
            raise ValueError('Not enough valid depth on box face')
        step = max(1, len(uu)//1500)
        uu, vv = uu[::step], vv[::step]
        intrinsic = np.asarray(self.info.k, dtype=float).reshape(3, 3)
        if intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0:
            raise ValueError('Invalid camera intrinsics')
        inverse = np.linalg.inv(intrinsic)
        rays = np.column_stack((uu, vv, np.ones(len(uu)))) @ inverse.T
        points = rays * (depth[vv, uu].astype(float)*scale)[:, None]

        # Iterative residual trimming, followed by a final SVD plane fit.
        for _ in range(3):
            mean = points.mean(axis=0)
            _, _, vt = np.linalg.svd(points-mean, full_matrices=False)
            normal = vt[-1]
            error = np.abs((points-mean) @ normal)
            keep = error < max(0.008, 3.0 * float(np.median(error)))
            if np.count_nonzero(keep) < 80:
                raise ValueError('Unstable face plane')
            points = points[keep]
        mean = points.mean(axis=0)
        _, _, vt = np.linalg.svd(points-mean, full_matrices=False)
        normal = vt[-1]
        if normal @ mean < 0:
            normal = -normal  # Box +Z points inward, away from the camera.
        corner_rays = np.column_stack((corners, np.ones(4))) @ inverse.T
        denominator = corner_rays @ normal
        if np.any(np.abs(denominator) < 0.1):
            raise ValueError('Box face is too edge-on')
        distances = (normal @ mean) / denominator
        if np.any(distances <= 0):
            raise ValueError('Invalid corner-plane intersection')
        xyz = corner_rays * distances[:, None]
        width = float(np.linalg.norm(xyz[1]-xyz[0]))
        height = float(np.linalg.norm(xyz[3]-xyz[0]))
        if abs(width-BOX_WIDTH) > 0.10 or abs(height-BOX_HEIGHT) > 0.12:
            raise ValueError(f'Face size mismatch: {width:.3f} x {height:.3f} m')
        x_axis = xyz[1]-xyz[0]
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(normal, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, normal)
        rotation = np.column_stack((x_axis, y_axis, normal))
        box_center = xyz.mean(axis=0) + (BOX_DEPTH/2.0)*normal
        rvec, _ = cv2.Rodrigues(rotation)
        angle = float(np.linalg.norm(rvec))
        quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        if angle > 1e-10:
            quaternion[:3] = rvec.ravel()/angle * np.sin(angle/2.0)
            quaternion[3] = np.cos(angle/2.0)

        cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 0), 3)
        for number, corner in enumerate(corners):
            cv2.putText(frame, str(number), tuple(corner.astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        cv2.circle(frame, tuple(white_point.astype(int)), 8, (255, 0, 255), 2)
        cv2.drawFrameAxes(frame, intrinsic, np.zeros(5), rvec,
                          box_center.reshape(3, 1), 0.15, 3)
        return box_center, quaternion, width, height

    def process(self, rgb, depth_message):
        frame = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
        try:
            if self.info is None:
                raise ValueError('Waiting for CameraInfo')
            depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
            if (depth.shape != (rgb.height, rgb.width)
                    or depth.shape != (self.info.height, self.info.width)):
                raise ValueError('RGB / aligned depth / CameraInfo sizes differ')
            if not (self.info.header.frame_id == rgb.header.frame_id
                    == depth_message.header.frame_id):
                raise ValueError('RGB / aligned depth / CameraInfo frames differ')
            if any(abs(value) > 1e-8 for value in self.info.d):
                raise ValueError('Nonzero distortion: rectification required')
            position, quaternion, width, height = self.estimate(
                frame, depth, depth_message.encoding)
            pose = PoseStamped()
            pose.header = rgb.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = map(float, quaternion)
            self.pose_pub.publish(pose)
            marker = Marker()
            marker.header = pose.header
            marker.ns = 'detected_box'
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x, marker.scale.y, marker.scale.z = BOX_WIDTH, BOX_HEIGHT, BOX_DEPTH
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.2, 1.0, 0.2, 0.6
            marker.lifetime.nanosec = 500_000_000
            self.marker_pub.publish(marker)
            label = f'POSE OK | measured face {width:.3f} x {height:.3f} m'
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            label = str(error)
        cv2.putText(frame, label, (15, 45), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 255, 0), 3)
        output = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        output.header = rgb.header
        self.debug_pub.publish(output)
        now = time.monotonic()
        if now-self.last_log >= 1.0:
            print(label, flush=True)
            self.last_log = now


def main():
    rclpy.init()
    node = BoxPoseRGBD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
