import rclpy
from rclpy.executors import ExternalShutdownException
import serial

from wheel_control.motor_interface import MotorInterface


# This launcher runs only the robot node, so the robot computer does not need PyQt
# ROS arguments can set the port parameter or remap the command topic
def main(args=None):
    rclpy.init(args=args)
    node = MotorInterface()
    exit_code = 0
    try:
        node.connect()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (serial.SerialException, OSError) as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        # A failed serial write must not prevent ROS resources from being released
        try:
            node.close()
        except (serial.SerialException, OSError):
            pass
        node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
