
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotionCommand(Node):
    def __init__(self):
        super().__init__('motion_command')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robot dimensions
        self.wheel_radius = 0.0762       # meters
        self.wheel_separation = 0.443   # meters

        # Match the existing motor interface limit: 30 RPM
        self.wheel_rpm = 10 * 8.9645 * 1.1

        # Linear speed of each wheel in m/s
        self.speed = (
            self.wheel_rpm * 2.0 * math.pi / 60.0
            * self.wheel_radius
        )

        # Angular speed for an in-place turn, in rad/s
        self.turn_speed = 2.0 * self.speed / self.wheel_separation

    def publish_velocity(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)

    def stop(self):
        # Send several stop messages to the motor interface
        for _ in range(5):
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def execute_command(self, command, value):
        if not math.isfinite(value) or value <= 0:
            self.get_logger().error(
                'Distance or angle must be greater than zero.'
            )
            return

        if command == 'forward':
            linear = self.speed
            angular = 0.0
            duration = 1.035 * value / self.speed

        elif command == 'backward':
            linear = -self.speed
            angular = 0.0
            duration = value / self.speed

        elif command == 'left':
            linear = 0.0
            angular = self.turn_speed
            duration = 2.461 * math.radians(value) / self.turn_speed

        elif command == 'right':
            linear = 0.0
            angular = -self.turn_speed
            duration = 2.67 * math.radians(value) / self.turn_speed

        else:
            self.get_logger().error('Unknown movement command.')
            return

        self.get_logger().info(
            f'{command} {value:g}: '
            f'moving for {duration:.2f} seconds'
        )

        start_time = time.monotonic()

        try:
            while time.monotonic() - start_time < duration:
                self.publish_velocity(linear, angular)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.04)
        finally:
            self.stop()

        self.get_logger().info('Motion complete.')


def main(args=None):
    rclpy.init(args=args)
    node = MotionCommand()
    commands = []

    try:
        print('\n=== FridgeBot Motion Command ===')
        print('Enter movements to build a sequence:')
        print('  forward 1      (meters)')
        print('  backward 0.5   (meters)')
        print('  left 90        (degrees)')
        print('  right 45       (degrees)')
        print('\nCommands:')
        print('  start  - execute the queued sequence')
        print('  cancel - clear the queued sequence')
        print('  help   - show these instructions')
        print('\nThe robot will NOT move until you type start.\n')

        while rclpy.ok():
            try:
                user_input = input('motion> ').strip().lower()

                if user_input == 'start':
                    if not commands:
                        print('No commands queued. Add movements first.')
                        continue
                    break

                if user_input == 'cancel':
                    commands.clear()
                    print('Queue cleared. The robot has not moved.')
                    continue

                if user_input == 'help':
                    print(
                        'Enter forward <meters>, backward <meters>, '
                        'left <degrees>, or right <degrees>.'
                    )
                    print('Type start to execute or cancel to clear.')
                    continue

                parts = user_input.split()

                if len(parts) != 2:
                    print('Format: command number (e.g. forward 1)')
                    continue

                command = parts[0]

                if command not in (
                    'forward', 'backward', 'left', 'right'
                ):
                    print('Use forward, backward, left, or right.')
                    continue

                try:
                    value = float(parts[1])
                except ValueError:
                    print('Please enter a valid number.')
                    continue

                if not math.isfinite(value) or value <= 0:
                    print('Enter a finite number greater than zero.')
                    continue

                commands.append((command, value))

                units = (
                    'meters'
                    if command in ('forward', 'backward')
                    else 'degrees'
                )

                print(
                    f'Added command {len(commands)}: '
                    f'{command} {value:g} {units}'
                )

            except (KeyboardInterrupt, EOFError):
                print('\nInput interrupted. No queued movements executed.')
                return

        print(f'\nStarting sequence of {len(commands)} commands...')

        for i, (command, value) in enumerate(commands, start=1):
            print(f'\n[{i}/{len(commands)}] {command} {value:g}')
            node.execute_command(command, value)

            if i < len(commands):
                time.sleep(1.5)  # Wait 1.5 seconds between commands

        print('\nSequence complete.')

    finally:
        # Always request zero velocity when exiting
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
