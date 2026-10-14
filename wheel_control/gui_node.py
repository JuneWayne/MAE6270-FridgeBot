from rclpy.node import Node
from std_msgs.msg import String

from .format_motor_command import format_motor_command
from .motor_settings import MotorSettings


# This class is the base station ROS node, while the window class handles Qt widgets
# It publishes complete motor target pairs and checks whether ROS can see a subscriber
class GuiNode(Node):
    def __init__(self):
        super().__init__('week7_gui')
        # Keep only the latest pending command because old targets are not useful
        self.publisher = self.create_publisher(String, MotorSettings.TOPIC, 1)
        self.targets = (0, 0)

    def publish_targets(self):
        # These are requested drive values, so publishing them does not confirm wheel motion
        self.publisher.publish(String(data=format_motor_command(*self.targets)))

    def interface_available(self):
        # This confirms ROS discovery, not encoder feedback or the motor shield state
        return self.publisher.get_subscription_count() > 0
