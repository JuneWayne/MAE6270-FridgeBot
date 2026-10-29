import math
import time

from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

from .format_motor_command import format_motor_command
from .motor_settings import MotorSettings


# This class is the robot ROS node that turns topic messages into USB serial commands
# The Arduino applies the output through the motor shield, so this class does not drive motor pins
class MotorInterface(Node):
    def __init__(self):
        super().__init__('motor_interface')
        # The wheel diameter is 150 mm, so the radius is 75.0 mm
        self.wheel_radius = 0.1524 / 2.0
        # Measure separation between the left and right wheel centres in metres
        self.wheel_separation = 0.443
        # Keep the 10 RPM request limit already selected in this interface
        self.wheel_rpm_limit = 13.5 * 8.9645 * 1.15
        self.max_wheel_rad_s = self.wheel_rpm_limit * 2.0 * math.pi / 60.0
        # Estimate gearbox output RPM at full drive until encoder calibration is available
        # The gearbox reduction must not be applied again to this output shaft speed
        self.estimated_full_drive_rpm = 13.5 * 8.9645 * 1.15 # multiplied based on calibration plot
        self.declare_parameter('port', MotorSettings.DEFAULT_PORT)
        self.connection = None
        self.targets = (0, 0)
        self.last_command = None
        self.replies = bytearray()
        # Keep the command timeout active even when ROS simulation time is paused
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

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
        self.timer = self.create_timer(
            MotorSettings.COMMAND_PERIOD_SECONDS, self.update, clock=self.steady_clock
        )
        # Report the resolved subscription topic, including any ROS topic remapping
        self.get_logger().info(f'Ready on {port}; waiting for {self.subscription.topic_name}')

    def receive_command(self, message):
        v = message.linear.x
        omega = message.angular.z

        # Calculate wheel speed requests in rad/s from robot speed and turn rate
        right = 0.927 * (v + (self.wheel_separation / 2.0) * omega) / self.wheel_radius
        left = (v - (self.wheel_separation / 2.0) * omega) / self.wheel_radius

        # Invalid or overflowing input must remove drive instead of crashing the callback
        if not math.isfinite(left) or not math.isfinite(right):
            self.targets = (0, 0)
            self.last_command = None
            self.write_targets()
            self.get_logger().warning('Invalid velocity command; both commands are zero')
            return

        # Reduce both requests together to preserve their ratio at the RPM limit
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_rad_s:
            scale = self.max_wheel_rad_s / peak
            left *= scale
            right *= scale

        # Convert wheel angular velocity (rad/s) to RPM
        left_rpm = left * 60.0 / (2.0 * math.pi)
        right_rpm = right * 60.0 / (2.0 * math.pi)

        # Convert requested RPM into an estimated drive value without encoder feedback
        max_command = MotorSettings.MAX_COMMAND

        self.targets = (
            round(left_rpm / self.estimated_full_drive_rpm * max_command),
            round(right_rpm / self.estimated_full_drive_rpm * max_command),
        )

        # Record fresh input only after the complete command has been calculated
        self.last_command = time.monotonic()
        # Forward fresh targets immediately, including a zero velocity request
        self.write_targets()

    def write_targets(self):
        # The Arduino expects both integer drive values followed by a newline
        data = (format_motor_command(*self.targets) + '\n').encode('ascii')
        # Treat an incomplete write as a serial fault so the launcher stops this interface
        if self.connection.write(data) != len(data):
            raise serial.SerialException('Incomplete serial write')

    def update(self):
        # Do not repeat an old moving command after the joystick or network disappears
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
                self.connection = None
