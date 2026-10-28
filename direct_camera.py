# Capture webcam images with OpenCV and publish them to ROS 2 for RViz.

# Read the camera device path from the start command
import argparse

import time  # Measure the actual publishing rate.
import cv2  # Capture, decode, and resize webcam images using OpenCV.
from cv_bridge import CvBridge  # Convert OpenCV images into ROS Image messages.
import rclpy  # Use ROS 2 from Python.
from rclpy.qos import QoSProfile, ReliabilityPolicy  # Configure message delivery.
from sensor_msgs.msg import Image  # Import the standard ROS image message type.

CAMERA_DEVICE = '/dev/video0'  # Change this if Linux assigns a different camera path.
OUTPUT_SIZE = (320, 240)  # Publish smaller images to reduce ROS traffic; width, height.


def main():  # Run the camera publisher.
    # Select the camera device and pass the remaining arguments to ROS
    parser = argparse.ArgumentParser(description='Publish USB camera images on /image_raw')
    parser.add_argument('--device', default=CAMERA_DEVICE, help='Linux camera device path')
    camera_args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node('direct_camera')  # Name this ROS node direct_camera.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)  # Keep one image; allow drops.
    publisher = node.create_publisher(Image, '/image_raw', qos)  # Send images on /image_raw.
    bridge = CvBridge()  # Create the OpenCV-to-ROS image converter.
    # Open the selected Linux camera device
    camera = cv2.VideoCapture(camera_args.device, cv2.CAP_V4L2)

    try:  # Ensure the camera and ROS resources are released if capture stops.
        if not camera.isOpened():  # Check that the camera opened successfully.
            raise RuntimeError('Cannot open camera; check USB attachment and permissions.')  # Report failure.

        if not camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')):  # Request MJPEG and check acceptance.
            node.get_logger().warning('Camera rejected MJPEG format.')  # Report rejection without stopping.
        if not camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640):  # Request a 640-pixel capture width and check acceptance.
            node.get_logger().warning('Camera rejected width 640.')  # Report rejection without stopping.
        if not camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480):  # Request a 480-pixel capture height and check acceptance.
            node.get_logger().warning('Camera rejected height 480.')  # Report rejection without stopping.
        if not camera.set(cv2.CAP_PROP_FPS, 30):  # Request 30 FPS; acceptance does not guarantee actual FPS.
            node.get_logger().warning('Camera rejected 30 FPS.')  # Report rejection without stopping.
        # Report the selected device and image topic
        node.get_logger().info(f'Camera {camera_args.device} -> /image_raw; press Ctrl+C to stop')
        count = 0  # Count frames published during each measurement interval.
        start = time.monotonic()  # Record the measurement interval's start time.

        while rclpy.ok():  # Keep capturing until ROS shuts down.
            ok, frame = camera.read()  # Capture and decode the next frame into pixel data.
            if not ok:  # Detect a failed frame read or camera disconnection.
                raise RuntimeError('Camera frame read failed; check the USB connection.')  # Stop on failure.
            if not rclpy.ok():  # Check whether shutdown happened while waiting for a frame.
                break  # Do not publish after ROS has shut down.

            frame = cv2.resize(frame, OUTPUT_SIZE, interpolation=cv2.INTER_AREA)  # Resize for ROS.
            msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')  # Package pixels as a ROS Image.
            msg.header.stamp = node.get_clock().now().to_msg()  # Timestamp after capture, not exposure.
            msg.header.frame_id = 'camera'  # Label the coordinate frame; this does not create a TF.
            publisher.publish(msg)  # Send the image to subscribers such as RViz.

            count += 1  # Include this published frame in the FPS calculation.
            elapsed = time.monotonic() - start  # Calculate seconds since the interval began.
            if elapsed >= 5:  # Report the average publishing rate every five seconds.
                node.get_logger().info(f'Publishing {count / elapsed:.1f} FPS')  # Not RViz's display FPS.
                start = time.monotonic()  # Start a fresh measurement interval.
                count = 0  # Reset the frame count for that interval.

    except KeyboardInterrupt:  # Handle Ctrl+C without displaying a traceback.
        pass  # Continue to the cleanup section below.
    except (RuntimeError, cv2.error) as error:  # Handle capture or image-processing errors.
        node.get_logger().error(str(error))  # Print the reason capture stopped.
        raise SystemExit(1)  # Exit with a failure status after cleanup.
    finally:  # Run cleanup on normal shutdown, Ctrl+C, or a capture error.
        camera.release()  # Release the webcam so another program can use it.
        node.destroy_node()  # Release this node's ROS resources.
        rclpy.try_shutdown()  # Shut down ROS if it is still running.


if __name__ == '__main__':  # Run only when this file is executed, not imported.
    main()  # Start the camera publisher.
