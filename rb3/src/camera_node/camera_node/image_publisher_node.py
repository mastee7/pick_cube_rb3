#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import os
import time

class ImagePublisherSaver(Node):
    """
    A ROS 2 node that captures images from a camera, publishes them as
    sensor_msgs/Image, and saves them to a specified directory.
    """
    def __init__(self):
        super().__init__('image_publisher_saver_node')

        # --- Parameters ---
        # Declare parameters with descriptions for clarity
        param_desc_cam_idx = ParameterDescriptor(description='Index of the camera device (e.g., 0 for /dev/video0)')
        param_desc_topic = ParameterDescriptor(description='Topic name to publish the raw image')
        param_desc_save_path = ParameterDescriptor(description='Directory path to save captured images')
        param_desc_interval = ParameterDescriptor(description='Image capture interval in seconds')
        param_desc_filename_prefix = ParameterDescriptor(description='Prefix for saved image filenames')

        self.declare_parameter('camera_index', 0, param_desc_cam_idx)
        self.declare_parameter('publish_topic', 'camera/image_raw', param_desc_topic)
        self.declare_parameter('save_path', os.path.expanduser('~/ros2_images'), param_desc_save_path)
        self.declare_parameter('capture_interval_sec', 1.0, param_desc_interval)
        self.declare_parameter('filename_prefix', 'rb3_capture_', param_desc_filename_prefix)

        # Get parameter values
        self.camera_index = self.get_parameter('camera_index').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.save_path = self.get_parameter('save_path').value
        self.capture_interval = self.get_parameter('capture_interval_sec').value
        self.filename_prefix = self.get_parameter('filename_prefix').value

        # --- Initialization ---
        self.get_logger().info(f"Initializing node...")
        self.get_logger().info(f"Using camera index: {self.camera_index}")
        self.get_logger().info(f"Publishing images to: {self.publish_topic}")
        self.get_logger().info(f"Saving images to: {self.save_path}")
        self.get_logger().info(f"Capture interval: {self.capture_interval} seconds")

        # Create save directory if it doesn't exist
        try:
            os.makedirs(self.save_path, exist_ok=True)
            self.get_logger().info(f"Ensured save directory exists: {self.save_path}")
        except OSError as e:
            self.get_logger().error(f"Failed to create save directory '{self.save_path}': {e}")
            # Decide how to handle this - shutdown or try to continue without saving?
            rclpy.shutdown()
            return

        # ROS Publisher
        self.publisher_ = self.create_publisher(Image, self.publish_topic, 10) # QoS depth 10

        # CvBridge
        self.bridge = CvBridge()

        # OpenCV Camera Initialization
        self.cap = cv2.VideoCapture(self.camera_index)

        # Construct the GStreamer pipeline string
        # You might want to make parts of this (width, height, format, framerate) parameters later
        gstreamer_pipeline = (
            "qtiqmmfsrc ! "
            "video/x-raw,format=NV12,width=1280,height=720,framerate=30/1 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! appsink"
        )

        self.get_logger().info(f"Attempting to open camera using GStreamer pipeline: {gstreamer_pipeline}")

        # OpenCV Camera Initialization using the GStreamer pipeline
        # Explicitly specify CAP_GSTREAMER
        self.cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error(f"Error: Could not open video device {self.camera_index}. "
                                    "Check camera connection and permissions (/dev/video*).")
            rclpy.shutdown() # Shutdown if camera cannot be opened
            return

        # Set camera properties (optional, example for frame size)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        # height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        # self.get_logger().info(f"Camera resolution set to: {int(width)}x{int(height)}")


        self.get_logger().info(f"Successfully opened camera device: {self.camera_index}")

        # ROS Timer for periodic capture
        self.timer = self.create_timer(self.capture_interval, self.timer_callback)
        self.get_logger().info(f"Started timer to capture every {self.capture_interval} seconds.")
        self.get_logger().info("Image Publisher and Saver Node is ready.")


    def timer_callback(self):
        """
        Called periodically by the timer. Captures a frame, publishes it, and saves it.
        """
        # Capture frame-by-frame
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warn("Failed to grab frame from camera. Skipping cycle.")
            return

        # Get current ROS time for timestamping
        current_stamp = self.get_clock().now().to_msg()

        # --- Publish the image ---
        try:
            # Convert OpenCV image (BGR format by default) to ROS Image message
            # Common encoding for color images in ROS is 'bgr8'
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image_msg.header.stamp = current_stamp
            # Assign a frame_id, important for TF and other ROS tools
            ros_image_msg.header.frame_id = 'camera_color_optical_frame' # Example frame ID

            self.publisher_.publish(ros_image_msg)
            self.get_logger().debug(f"Published image with timestamp: {current_stamp.sec}.{current_stamp.nanosec}")
        except Exception as e:
             self.get_logger().error(f"Failed to convert or publish frame: {e}")


        # --- Save the image ---
        try:
            # Generate filename using timestamp for uniqueness
            # Using nanoseconds ensures high probability of unique names even with fast capture rates
            filename = f"{self.filename_prefix}{current_stamp.sec}_{current_stamp.nanosec}.jpg"
            full_path = os.path.join(self.save_path, filename)

            # Save the OpenCV frame (which is a numpy array) to a file
            success = cv2.imwrite(full_path, frame)

            if success:
                self.get_logger().info(f"Saved image to: {full_path}")
            else:
                self.get_logger().warn(f"Failed to write image file to: {full_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save frame: {e}")

    def destroy_node(self):
        """
        Cleanup resources when the node is shutting down.
        """
        self.get_logger().info("Shutting down node...")
        if hasattr(self, 'timer'):
            self.timer.cancel()
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera device released.")
        # Call the superclass's destroy_node method
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None # Initialize node to None
    try:
        node = ImagePublisherSaver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down.")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception: {e}")
        else:
            print(f"Exception during node initialization: {e}")
    finally:
        # Ensure cleanup happens even if errors occur during spin
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
