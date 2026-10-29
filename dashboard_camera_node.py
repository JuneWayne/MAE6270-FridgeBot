import time

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


# Receive camera images without sending motor commands
class DashboardCameraNode(Node):
    def __init__(self):
        super().__init__('fridgebot_dashboard')
        self.bridge = CvBridge()
        self.pending_frame = None
        self.last_frame_time = None
        self.camera_error = None
        self.publisher_count = 0
        self.last_discovery_check = None
        self.received_images = 0
        self.declare_parameter('camera_topic', '/image_raw')
        camera_topic = self.get_parameter('camera_topic').value

        # Keep the newest image and accept the existing best effort camera publisher
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Image, camera_topic, self.receive_image, qos
        )
        self.camera_topic = self.subscription.topic_name
        self.get_logger().info(f'Camera display subscribes to {self.camera_topic}')

    def receive_image(self, message):
        self.received_images += 1
        # Convert the ROS encoding to RGB bytes for the Qt display
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
            if (frame.size == 0 or frame.ndim != 3 or frame.shape[2] != 3
                    or frame.dtype != np.uint8):
                raise ValueError('The converted camera image must contain RGB8 pixels')
            self.pending_frame = np.ascontiguousarray(frame)
        except (CvBridgeError, cv2.error, ValueError, TypeError) as error:
            self.pending_frame = None
            detail = f'Cannot read camera image: {error}'
            if detail != self.camera_error:
                self.get_logger().warning(detail)
            self.camera_error = detail
            return

        # Use local arrival time to detect a stopped image stream
        self.last_frame_time = time.monotonic()
        self.camera_error = None

    def camera_status(self):
        # Check ROS discovery once per second without slowing each display update
        now = time.monotonic()
        if self.last_discovery_check is None or now - self.last_discovery_check >= 1.0:
            self.publisher_count = self.count_publishers(self.camera_topic)
            self.last_discovery_check = now
        if self.camera_error:
            return self.camera_error
        if self.last_frame_time is not None and now - self.last_frame_time <= 2.0:
            return 'Camera images received'
        if not self.publisher_count:
            return f'No camera publisher on {self.camera_topic}'
        if self.last_frame_time is None:
            return f'Camera publisher found on {self.camera_topic}, but no image received'
        return f'No camera image for 2 seconds on {self.camera_topic}'
