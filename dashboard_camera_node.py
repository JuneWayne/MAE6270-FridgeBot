import time

import numpy as np
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

        # Keep the newest image and accept the existing best effort camera publisher
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.receive_image, qos
        )

    def receive_image(self, message):
        # Convert the ROS encoding to RGB bytes for the Qt display
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
            if frame.size == 0:
                raise ValueError('The camera image is empty')
            self.pending_frame = np.ascontiguousarray(frame)
        except (CvBridgeError, ValueError, TypeError) as error:
            self.pending_frame = None
            self.camera_error = f'Cannot read camera image: {error}'
            return

        # Use local arrival time to detect a stopped image stream
        self.last_frame_time = time.monotonic()
        self.camera_error = None
