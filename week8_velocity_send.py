# Check that the timing settings are finite numbers
import math

# Measure joystick input age with a clock that cannot move backwards
import time

# Use a timer clock that continues when simulation time is paused
from rclpy.clock import Clock, ClockType

# Use the ROS node base class
from rclpy.node import Node
# Accept the standard sensor message delivery settings
from rclpy.qos import qos_profile_sensor_data

# Read gamepad axes from Joy messages
from sensor_msgs.msg import Joy

# Send linear and angular velocity in Twist messages
from geometry_msgs.msg import Twist


# Convert gamepad input into robot velocity commands
class JoystickTeleopNode(Node):

    # Create the node settings and ROS connections
    def __init__(self):

        # Match the node name used by the YAML parameter file
        super().__init__('joystick_teleop')

        # Select the axes array position for forward and backward input
        self.declare_parameter('linear_axis', 1)

        # Select the axes array position for left and right input
        self.declare_parameter('angular_axis', 0)

        # Set the full-stick forward speed request in m/s
        self.declare_parameter('linear_scale', 2.5)

        # Set the full-stick turn rate request in rad/s
        self.declare_parameter('angular_scale', 1.1)

        # Set how many velocity messages to publish each second
        self.declare_parameter('update_rate', 5.0)

        # Set the permitted gap between valid joystick messages in seconds
        self.declare_parameter('joy_timeout', 0.25)

        # Store the forward input position
        self.linear_axis = self.get_parameter('linear_axis').value

        # Store the turn input position
        self.angular_axis = self.get_parameter('angular_axis').value

        # Store the forward speed scale
        self.linear_scale = self.get_parameter('linear_scale').value

        # Store the turn rate scale
        self.angular_scale = self.get_parameter('angular_scale').value

        # Store the velocity message rate in Hz
        self.update_rate = self.get_parameter('update_rate').value

        # Store the joystick input timeout in seconds
        self.joy_timeout = self.get_parameter('joy_timeout').value

        # Require a valid positive rate before calculating the timer period
        if not math.isfinite(self.update_rate) or self.update_rate <= 0.0:
            # Explain an invalid rate instead of dividing by zero
            raise ValueError('update_rate must be finite and greater than zero')

        # Require a valid positive timeout before accepting movement commands
        if not math.isfinite(self.joy_timeout) or self.joy_timeout <= 0.0:
            # Explain which timing setting needs correction
            raise ValueError('joy_timeout must be finite and greater than zero')

        # Start with zero in every linear and angular velocity field
        self.target = Twist()

        # Record that no joystick input has arrived yet
        self.last_joy = None

        # Create a Twist publisher on /cmd_vel with a queue depth of one
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        # Call receive_joy whenever a Joy message arrives on /joy
        self.subscription = self.create_subscription(
            Joy, '/joy', self.receive_joy, qos_profile_sensor_data
        )

        # Keep command publication and timeout checks independent of simulation time
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        # Publish from this timer instead of publishing every joystick input message
        self.timer = self.create_timer(
            1.0 / self.update_rate, self.update, clock=self.steady_clock
        )

        # Show the loaded speed scales and timing settings in the terminal
        self.get_logger().info(
            f'Week 8: /joy -> /cmd_vel; linear_scale={self.linear_scale}, '
            f'angular_scale={self.angular_scale}; update_rate={self.update_rate} Hz, '
            f'joy_timeout={self.joy_timeout} s'
        )

    # Convert each received gamepad message into a velocity message
    def receive_joy(self, message):

        # Create a Twist message with all six velocity fields at zero
        command = Twist()

        # Calculate forward speed from the selected stick input
        command.linear.x = message.axes[self.linear_axis] * self.linear_scale

        # Calculate turn rate from the selected stick input
        command.angular.z = message.axes[self.angular_axis] * self.angular_scale

        # Save the latest request for the next publication timer call
        self.target = command

        # Record when this joystick input arrived
        self.last_joy = time.monotonic()

    # Publish at the selected rate and cancel commands when joystick input stops
    def update(self):

        # Detect input that has never arrived or has exceeded the permitted age
        if self.last_joy is None or time.monotonic() - self.last_joy >= self.joy_timeout:

            # Report a timeout once when it cancels a movement command
            if self.target.linear.x != 0.0 or self.target.angular.z != 0.0:

                # Explain why the velocity request has returned to zero
                self.get_logger().warning('Joystick input timeout; velocity commands are zero')

            # Replace any previous movement command with zero
            self.target = Twist()

            # Wait for a new joystick message before accepting movement again
            self.last_joy = None

        # Send both velocity values together at the selected update rate
        self.publisher.publish(self.target)

    # Request zero velocity before the program releases its ROS resources
    def stop(self):

        # Remove the previous movement command
        self.target = Twist()

        # Remove the input timestamp so a later timer call also sends zero
        self.last_joy = None

        # Send zero immediately instead of waiting for the next publication timer call
        self.publisher.publish(self.target)
