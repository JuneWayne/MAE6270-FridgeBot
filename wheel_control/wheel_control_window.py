from pathlib import Path

import rclpy
from PyQt5 import uic
from PyQt5.QtCore import QSignalBlocker, QTimer, pyqtSlot
from PyQt5.QtWidgets import QMainWindow

from .motor_settings import MotorSettings


# Load the visual layout from Qt Designer and connect it to the ROS publisher node
# The Designer file still handles slider and number display connections
class WheelControlWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        ui_path = Path(__file__).resolve().parent.parent / 'week7_gui.ui'
        uic.loadUi(str(ui_path), self)

        self.controls = [self.LeftSlider, self.RightSlider,
                         self.LeftValue, self.RightValue]
        for control in self.controls:
            control.setRange(-MotorSettings.MAX_COMMAND, MotorSettings.MAX_COMMAND)
        self.reset_controls()
        self.enable_controls(False)
        self.StopButton.clicked.connect(self.stop_controls)
        self.statusbar.showMessage('Motor interface: unavailable')

        # Give ROS a short turn inside the Qt event loop without using a second thread
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.service_ros)
        self.ros_timer.start(10)

        # A held slider needs repeated messages or the robot timeout will remove drive
        self.command_timer = QTimer(self)
        self.command_timer.timeout.connect(self.update_commands)
        self.command_timer.start(round(MotorSettings.COMMAND_PERIOD_SECONDS * 1000))

    def enable_controls(self, enabled):
        for control in self.controls:
            control.setEnabled(enabled)

    def reset_controls(self):
        # Block all four signals so Stop cannot send a pair with only one side reset
        blockers = [QSignalBlocker(control) for control in self.controls]
        for control in self.controls:
            control.setValue(0)
        del blockers

    @pyqtSlot()
    def stop_controls(self):
        self.reset_controls()
        self.node.targets = (0, 0)
        # Send zero immediately instead of waiting for the next 100 ms timer call
        if rclpy.ok():
            self.node.publish_targets()
        self.statusbar.showMessage('Both motor commands are zero')

    def service_ros(self):
        if not rclpy.ok():
            self.close()
            return
        # A zero timeout lets Qt continue to process clicks and window updates
        rclpy.spin_once(self.node, timeout_sec=0)

    def update_commands(self):
        if not rclpy.ok():
            return
        connected = self.node.interface_available()
        self.enable_controls(connected)
        if connected:
            self.node.targets = (self.LeftSlider.value(), self.RightSlider.value())
            self.statusbar.showMessage('Motor interface: connected')
        else:
            # Clear old targets so reconnection does not resume a previous command
            self.reset_controls()
            self.node.targets = (0, 0)
            self.statusbar.showMessage('Motor interface: unavailable')
        self.node.publish_targets()

    def closeEvent(self, event):
        # Send zero on a normal window close and let robot timeouts cover a process crash
        self.command_timer.stop()
        self.ros_timer.stop()
        self.stop_controls()
        event.accept()
