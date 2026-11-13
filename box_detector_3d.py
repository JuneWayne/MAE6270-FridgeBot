#!/usr/bin/env python3
"""Detect the yellow border and publish a face point in the color optical frame.

Requires RGB-aligned depth and matching, zero-distortion CameraInfo. The point
is a surface sample near the contour's image centroid, not the volume center.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class BoxDetector3D(Node):
    def __init__(self):
        super().__init__('eriche_box_3d')
        self.bridge = CvBridge()
        self.info = None
        self.pairs = 0
        self.debug_pub = self.create_publisher(Image, '/eriche/target/debug_image', 10)
        self.mask_pub = self.create_publisher(Image, '/eriche/target/mask', 10)
        self.point_pub = self.create_publisher(PointStamped, '/eriche/target/point', 10)
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
        self.timer = self.create_timer(5.0, self.report)

    def on_info(self, message):
        self.info = message

    def report(self):
        self.get_logger().info(
            f'Synchronized pairs: {self.pairs}; CameraInfo received: {self.info is not None}')

    def localize(self, rgb, depth_message, depth, u, v):
        if self.info is None:
            return 'Waiting for CameraInfo'
        if (depth.shape != (rgb.height, rgb.width)
                or depth.shape != (self.info.height, self.info.width)):
            return 'RGB / aligned depth / CameraInfo sizes differ'
        if not (self.info.header.frame_id == rgb.header.frame_id
                == depth_message.header.frame_id):
            return 'RGB / aligned depth / CameraInfo frames differ'
        if any(abs(value) > 1e-8 for value in self.info.d):
            return 'Nonzero distortion: rectification required'
        if depth_message.encoding == '16UC1':
            scale = 0.001
        elif depth_message.encoding == '32FC1':
            scale = 1.0
        else:
            return f'Unsupported depth: {depth_message.encoding}'
        patch = depth[max(0, v-3):v+4, max(0, u-3):u+4].astype(float) * scale
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size < 10:
            return 'Not enough valid depth at centroid'
        z = float(np.median(valid))
        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        if fx <= 0 or fy <= 0:
            return 'Invalid camera intrinsics'
        x, y = (u-cx)*z/fx, (v-cy)*z/fy
        point = PointStamped()
        point.header = rgb.header
        point.point.x, point.point.y, point.point.z = float(x), float(y), z
        self.point_pub.publish(point)
        return f'Face point: X={x:.3f} Y={y:.3f} Z={z:.3f} m'

    def process(self, rgb, depth_message):
        self.pairs += 1
        frame = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (18, 70, 60), (40, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 500:
                continue
            quad = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)
            if len(quad) == 4 and cv2.isContourConvex(quad):
                candidates.append((area, quad))
        label = 'No yellow rectangle'
        if candidates:
            _, quad = max(candidates, key=lambda item: item[0])
            moments = cv2.moments(quad)
            if moments['m00'] > 0:
                u = int(moments['m10'] / moments['m00'])
                v = int(moments['m01'] / moments['m00'])
                cv2.polylines(frame, [quad], True, (0, 255, 0), 3)
                cv2.circle(frame, (u, v), 6, (0, 0, 255), -1)
                label = self.localize(rgb, depth_message, depth, u, v)
        cv2.putText(frame, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 255, 0), 2)
        for array, encoding, publisher in (
            (frame, 'bgr8', self.debug_pub), (mask, 'mono8', self.mask_pub)
        ):
            output = self.bridge.cv2_to_imgmsg(array, encoding=encoding)
            output.header = rgb.header
            publisher.publish(output)


def main():
    rclpy.init()
    node = BoxDetector3D()
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
