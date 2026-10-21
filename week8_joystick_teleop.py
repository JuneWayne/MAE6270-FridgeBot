import rclpy
from rclpy.executors import ExternalShutdownException

# Load the Week 8 class that sends velocity directly from joystick input
from week8_velocity_send import JoystickTeleopNode

def main(args=None):
    # initialize ROS and read command arguments
    rclpy.init(args=args)
    #create the joystick node
    node = JoystickTeleopNode()
    try: 
        # run the subscription when messages arrive
        rclpy.spin(node)
    # shuts down node if ctrl+C or other stopping methods
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Send zero before shutdown when ROS communication is still available
        if node.context.ok():
            node.stop()
        # Release the publisher and subscription when the program stops
        node.destroy_node()
        rclpy.try_shutdown()
    
if __name__ == '__main__':
    main()
