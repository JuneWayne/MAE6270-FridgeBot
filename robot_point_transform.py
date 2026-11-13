#!/usr/bin/env python3
"""Transform detected face points to base_link and print readable live XYZ."""
import time
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class RobotPointTransform(Node):
    def __init__(self):
        super().__init__('eriche_robot_point')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointStamped, '/eriche/target/point_base', 10)
        self.subscription = self.create_subscription(
            PointStamped, '/eriche/target/point', self.process, 10)
        self.last_print = 0.0
        self.last_warning = 0.0
        self.last_received = time.monotonic()
        self.stale_reported = False
        self.timer = self.create_timer(0.5, self.check_stale)

    def process(self, message):
        now = time.monotonic()
        try:
            transform = self.buffer.lookup_transform(
                'base_link', message.header.frame_id,
                Time.from_msg(message.header.stamp))
            output = do_transform_point(message, transform)
        except TransformException as error:
            if now - self.last_warning > 2.0:
                self.get_logger().warning(f'Waiting for camera-to-base TF: {error}')
                self.last_warning = now
            return
        output.header.frame_id = 'base_link'
        output.header.stamp = message.header.stamp
        self.publisher.publish(output)
        self.last_received = now
        self.stale_reported = False
        if now - self.last_print >= 0.5:
            point = output.point
            print(f'BASE_LINK  X={point.x:+.3f} m  Y={point.y:+.3f} m  '
                  f'Z={point.z:+.3f} m', flush=True)
            self.last_print = now

    def check_stale(self):
        if time.monotonic() - self.last_received > 1.0 and not self.stale_reported:
            print('STALE: no successfully transformed detection for >1 s', flush=True)
            self.stale_reported = True


def main():
    rclpy.init()
    node = RobotPointTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
