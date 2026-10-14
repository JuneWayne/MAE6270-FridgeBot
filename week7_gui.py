import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from PyQt5.QtWidgets import QApplication

from wheel_control.gui_node import GuiNode
from wheel_control.wheel_control_window import WheelControlWindow


# This launcher starts Qt, creates the GUI node, and connects the window to that node
# The window layout and control logic live in separate files so this entry point stays small
def main(args=None):
    rclpy.init(args=args)
    # Keep ROS command-line arguments away from the Qt argument parser
    app = QApplication([sys.argv[0]])
    node = GuiNode()
    window = None
    try:
        window = WheelControlWindow(node)
        window.show()
        return app.exec_()
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        # Close the window before destroying its publisher so normal shutdown can send zero
        if window is not None:
            window.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
