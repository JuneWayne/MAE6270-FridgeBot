#!/usr/bin/env python3
"""Learning stage: yellow quadrilateral and its image centroid (pixels).

Do not run alongside box_detector_3d.py: they publish the same image topics.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class YellowDetector2D(Node):
    def __init__(self):
        super().__init__('eriche_yellow_detector')
        self.bridge = CvBridge()
        self.debug_pub = self.create_publisher(Image, '/eriche/target/debug_image', 10)
        self.mask_pub = self.create_publisher(Image, '/eriche/target/mask', 10)
        self.subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.process,
            qos_profile_sensor_data)

    def process(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
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
                label = f'Pixel centroid: ({u}, {v})'
        cv2.putText(frame, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2)
        for array, encoding, publisher in (
            (frame, 'bgr8', self.debug_pub), (mask, 'mono8', self.mask_pub)
        ):
            output = self.bridge.cv2_to_imgmsg(array, encoding=encoding)
            output.header = message.header
            publisher.publish(output)


def main():
    rclpy.init()
    node = YellowDetector2D()
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
