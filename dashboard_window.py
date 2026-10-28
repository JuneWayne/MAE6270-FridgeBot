import time
from pathlib import Path

import rclpy
from PyQt5 import uic
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QMainWindow, QSizePolicy
from rclpy.executors import ExternalShutdownException

from dashboard_lidar_panel import DashboardLidarPanel


# Load the Designer layout and update both sensor panels
class DashboardWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.camera_pixmap = None
        ui_path = Path(__file__).resolve().with_name('fridgebot_dashboard.ui')
        uic.loadUi(str(ui_path), self)
        self.setWindowTitle('FridgeBot camera and LiDAR')

        # Give both displays the same share of the window width
        self.horizontalLayout.setStretch(0, 1)
        self.horizontalLayout.setStretch(1, 1)
        self.cameraImage.setMinimumSize(1, 1)
        self.cameraImage.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.cameraImage.setAlignment(Qt.AlignCenter)
        self.cameraImage.setStyleSheet('background: #18212b; color: white;')

        # These buttons cannot control the motors until the robot connection is added
        for button in (self.enableMotorsButton, self.stopMotorsButton):
            button.setEnabled(False)
            button.setToolTip('Motor control is not connected')
        self.lidar_panel = DashboardLidarPanel(node, self.lidarView, self.lidarGroup)
        self.statusbar.showMessage('Waiting for camera | Motor control is not connected')

        # Run ROS callbacks in the Qt thread so display updates stay in the same thread
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.update_camera)
        self.ros_timer.start(33)

    def update_camera(self):
        if not rclpy.ok():
            self.close()
            return
        try:
            # Process both sensor streams without holding the GUI for a long time
            deadline = time.monotonic() + 0.005
            for _ in range(8):
                rclpy.spin_once(self.node, timeout_sec=0)
                if time.monotonic() >= deadline:
                    break
        except ExternalShutdownException:
            self.close()
            return

        # Copy the image bytes before the ROS frame is replaced
        frame = self.node.pending_frame
        if frame is not None:
            self.node.pending_frame = None
            height, width, _ = frame.shape
            image = QImage(
                frame.data, width, height, frame.strides[0], QImage.Format_RGB888
            ).copy()
            self.camera_pixmap = QPixmap.fromImage(image)
            self.fit_camera_image()

        # Remove old images when the stream stops so they do not look live
        if self.node.camera_error:
            camera_status = self.node.camera_error
        elif self.node.last_frame_time is None:
            camera_status = 'Waiting for camera'
        elif time.monotonic() - self.node.last_frame_time > 2.0:
            camera_status = 'No camera image for 2 seconds'
        else:
            camera_status = 'Camera images received'

        if camera_status != 'Camera images received':
            self.camera_pixmap = None
            self.cameraImage.setText(camera_status)
        lidar_status = self.lidar_panel.update()
        self.statusbar.showMessage(
            f'{camera_status} | {lidar_status} | Motor control is not connected | F11: full screen'
        )

    def fit_camera_image(self):
        # Keep the image proportions when the window size changes
        if self.camera_pixmap is not None:
            self.cameraImage.setPixmap(self.camera_pixmap.scaled(
                self.cameraImage.contentsRect().size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            ))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'cameraImage'):
            self.fit_camera_image()
        if hasattr(self, 'lidar_panel'):
            self.lidar_panel.fit_view()

    def keyPressEvent(self, event):
        # Use F11 to change display mode and Escape to leave full screen
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showMaximized()
            else:
                self.showFullScreen()
        elif event.key() == Qt.Key_Escape and self.isFullScreen():
            self.showMaximized()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        self.ros_timer.stop()
        super().closeEvent(event)
