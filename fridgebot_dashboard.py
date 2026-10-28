import sys

import rclpy
from PyQt5.QtWidgets import QApplication

from dashboard_camera_node import DashboardCameraNode
from dashboard_window import DashboardWindow


# Start the camera dashboard on the base computer
def main():
    rclpy.init(args=sys.argv)
    node = None
    window = None
    try:
        # Keep ROS command arguments separate from Qt command arguments
        app = QApplication([sys.argv[0]])
        node = DashboardCameraNode()
        window = DashboardWindow(node)
        window.showMaximized()
        return app.exec_()
    finally:
        if window is not None:
            window.close()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
