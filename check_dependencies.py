#!/usr/bin/env python3
"""Check the Python dependencies after sourcing ROS 2 Jazzy."""
import importlib


def main():
    missing = []
    modules = (
        'rclpy', 'cv2', 'numpy', 'cv_bridge', 'sensor_msgs.msg',
        'geometry_msgs.msg', 'tf2_ros', 'message_filters',
        'tf2_geometry_msgs', 'visualization_msgs.msg',
    )
    for name in modules:
        try:
            importlib.import_module(name)
            print(f'OK       {name}')
        except ImportError as error:
            print(f'MISSING  {name}: {error}')
            missing.append(name)
    return bool(missing)


if __name__ == '__main__':
    raise SystemExit(main())
