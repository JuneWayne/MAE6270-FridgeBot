import time
import math

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial

from .format_motor_command import format_motor_command
from .motor_settings import MotorSettings
from .parse_motor_command import parse_motor_command


# This class is the robot ROS node that turns topic messages into USB serial commands
# The Arduino applies the output through the motor shield, so this class does not drive motor pins
class MotorInterface(Node):
    def __init__(self):
        super().__init__('motor_interface')
        self.wheel_radius = 0.075
        self.wheel_separation = 0.443
        self.gear_ratio = 504.0
        self.max_wheel_rad_s = 10.0 * 2.0 * math.pi /60.0
        self.declare_parameter('port', MotorSettings.DEFAULT_PORT)
        self.connection = None
        self.targets = (0, 0)
        self.last_command = None
        self.replies = bytearray()

    def connect(self):
        port = self.get_parameter('port').value
        self.connection = serial.Serial(
            port, MotorSettings.SERIAL_BAUD,
            timeout=0, write_timeout=0.2, exclusive=True
        )
        # Opening an Uno serial port usually resets the board, so let setup finish first
        time.sleep(2)
        self.write_targets()

        # Advertise the subscriber only after the serial port is ready and drive is zero
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.receive_command, 1
        )
        self.timer = self.create_timer(MotorSettings.COMMAND_PERIOD_SECONDS, self.update)
        self.get_logger().info(f'Ready on {port}; waiting for {MotorSettings.TOPIC}')

    def receive_command(self, message):
        v = message.linear.x
        omega = message.angular.z

        # Set both wheel speeds based on linear and angular velocity
        right = (v + (self.wheel_separation / 2.0) * omega) / self.wheel_radius
        left = (v - (self.wheel_separation / 2.0) * omega) / self.wheel_radius
       
        # Limit wheel speeds to 10 RPM
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_rad_s:
            scale = self.max_wheel_rad_s / peak
            left *= scale
            right *= scale
            
        # Convert wheel angular velocity (rad/s) to RPM
        left_rpm = left * 60.0 / (2.0 * math.pi)
        right_rpm = right * 60.0 / (2.0 * math.pi)

        # Scale RPM relative to the rated maximum wheel speed
        max_rpm = 13.5
        max_command = MotorSettings.MAX_COMMAND

        self.targets = (
            round(left_rpm / max_rpm * max_command),
            round(right_rpm / max_rpm * max_command),
        )

       self.last_command = time.monotonic()
       self.write_targets()
           

    def update(self):
        # Do not repeat an old moving command after the GUI or network disappears
        if (self.last_command is not None
                and time.monotonic() - self.last_command >= MotorSettings.COMMAND_TIMEOUT_SECONDS):
            if self.targets != (0, 0):
                self.get_logger().warning('ROS command timeout; both commands are zero')
            self.targets = (0, 0)
            self.last_command = None
        self.write_targets()
        self.read_replies()

    def read_replies(self):
        # Save partial response lines because one serial read does not always contain a full line
        self.replies.extend(self.connection.read(min(self.connection.in_waiting, 1024)))
        while b'\n' in self.replies:
            line, _, remaining = self.replies.partition(b'\n')
            self.replies = bytearray(remaining)
            reply = line.decode('ascii', errors='replace').strip()
            # Keep startup, faults, and errors visible without printing every repeated OK
            if reply and reply != 'OK':
                self.get_logger().info(f'Arduino: {reply}')
        if len(self.replies) > 4096:
            self.replies.clear()
            self.get_logger().warning('Discarded an oversized Arduino response')

    def close(self):
        # Try to remove drive before releasing the port, even after a communication error
        if self.connection is not None:
            try:
                self.targets = (0, 0)
                self.write_targets()
            finally:
                self.connection.close()
