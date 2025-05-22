#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import time # For unique filenames

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        # --- Parameters ---
        # Define descriptions for parameters
        descr_sub_topic = ParameterDescriptor(description='Input topic for sensor_msgs/Image')
        descr_pub_topic = ParameterDescriptor(description='Output topic for annotated sensor_msgs/Image')
        descr_config = ParameterDescriptor(description='Path to YOLO config file relative to package share/resource/data')
        descr_weights = ParameterDescriptor(description='Path to YOLO weights file relative to package share/resource')
        descr_names = ParameterDescriptor(description='Path to YOLO class names file relative to package share/resource/data')
        descr_out_dir = ParameterDescriptor(description='Directory to save annotated images')
        descr_conf_thresh = ParameterDescriptor(description='Confidence threshold for detection')
        descr_nms_thresh = ParameterDescriptor(description='Non-Maximum Suppression threshold')

        # Declare parameters
        self.declare_parameter('input_image_topic', 'camera/image_raw', descr_sub_topic) # Default to the publisher's default
        self.declare_parameter('output_image_topic', 'yolo/annotated_image', descr_pub_topic)
        self.declare_parameter('config_file', 'data/yolov4-tiny.cfg', descr_config)
        self.declare_parameter('weights_file', 'yolov4-tiny_last.weights', descr_weights)
        self.declare_parameter('names_file', 'data/obj.names', descr_names)
        self.declare_parameter('output_dir', 'yolo_output', descr_out_dir) # Relative path inside workspace install/pkg dir or provide absolute
        self.declare_parameter('confidence_threshold', 0.5, descr_conf_thresh)
        self.declare_parameter('nms_threshold', 0.4, descr_nms_thresh)

        # Get parameter values
        self.input_topic = self.get_parameter('input_image_topic').value
        self.output_topic = self.get_parameter('output_image_topic').value
        config_file_rel = self.get_parameter('config_file').value
        weights_file_rel = self.get_parameter('weights_file').value
        names_file_rel = self.get_parameter('names_file').value
        self.output_dir_param = self.get_parameter('output_dir').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value

        # --- Resolve Paths using Package Share Directory ---
        pkg_share_path = get_package_share_directory('detection_node') # Replace 'yolo_pkg' with your actual package name
        self.config_path = os.path.join(pkg_share_path, 'resource', config_file_rel)
        self.weights_path = os.path.join(pkg_share_path, 'resource', weights_file_rel)
        self.names_path = os.path.join(pkg_share_path, 'resource', names_file_rel)

        # Resolve output directory (handle absolute vs relative paths)
        if os.path.isabs(self.output_dir_param):
             self.output_dir = self.output_dir_param
        else:
             # Place relative paths inside the package's output folder within the source tree for ease of finding
             # Alternatively, place it in ~/.ros/ or some other standard location
             # For simplicity here, we place it relative to the source package's folder location.
             # A better ROS approach might be to use install space, but that complicates finding during dev.
             self.output_dir = os.path.join(os.path.dirname(__file__), '..', self.output_dir_param) # Relative to source file location

        self.get_logger().info(f"Input Topic: {self.input_topic}")
        self.get_logger().info(f"Output Annotated Topic: {self.output_topic}")
        self.get_logger().info(f"Config Path: {self.config_path}")
        self.get_logger().info(f"Weights Path: {self.weights_path}")
        self.get_logger().info(f"Names Path: {self.names_path}")
        self.get_logger().info(f"Output Directory: {self.output_dir}")
        self.get_logger().info(f"Confidence Threshold: {self.conf_threshold}")
        self.get_logger().info(f"NMS Threshold: {self.nms_threshold}")


        # --- Initialization ---
        # Load class names
        try:
            with open(self.names_path, "r") as f:
                self.classes = [line.strip() for line in f.readlines()]
            self.get_logger().info(f"Loaded {len(self.classes)} classes from {self.names_path}")
            # Generate distinct colors for each class
            np.random.seed(42) # for consistency
            self.colors = np.random.randint(0, 255, size=(len(self.classes), 3), dtype="uint8")
        except FileNotFoundError:
            self.get_logger().error(f"Class names file not found at {self.names_path}. Shutting down.")
            raise SystemExit
        except Exception as e:
            self.get_logger().error(f"Error loading class names: {e}. Shutting down.")
            raise SystemExit

        # Load YOLO model using OpenCV's DNN module
        try:
            self.net = cv2.dnn.readNetFromDarknet(self.config_path, self.weights_path)
            # Consider hardware acceleration (requires OpenCV built with support)
            # self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA) # Example
            # self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA) # Example
            # self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV) # Default
            # self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU) # Default
            self.get_logger().info("YOLO model loaded successfully.")
        except cv2.error as e:
             self.get_logger().error(f"Failed to load YOLO model: {e}")
             self.get_logger().error(f"Config path: {self.config_path}")
             self.get_logger().error(f"Weights path: {self.weights_path}")
             raise SystemExit
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred loading YOLO model: {e}")
            raise SystemExit

        # Set up CvBridge
        self.bridge = CvBridge()

        # Ensure the output directory exists
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            self.get_logger().info(f"Ensured output directory exists: {self.output_dir}")
        except OSError as e:
            self.get_logger().error(f"Could not create output directory {self.output_dir}: {e}")
            # Decide if this is fatal or if we can continue without saving
            # For now, let's continue but log the error

        # Create a subscriber for the image topic
        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10 # QoS profile depth
            # Consider using specific QoS profiles if needed, e.g. rclpy.qos.qos_profile_sensor_data
        )

        # Create a publisher for the annotated image
        self.annotated_image_pub = self.create_publisher(Float64MultiArray, 'coords_topic', 10)

        self.get_logger().info("YOLO Detection Node is ready.")

    def image_callback(self, msg):
        self.get_logger().debug(f"Received image message with timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Get timestamp for unique filename
        secs = msg.header.stamp.sec
        nanos = msg.header.stamp.nanosec

        # Perform YOLO object detection
        annotated_image, coords  = self.detect_objects(cv_image)

        # --- Save the annotated image ---
        if annotated_image is not None:
            try:
                # Generate unique filename using timestamp
                output_filename = f'output_{secs}_{nanos}.png'
                output_path = os.path.join(self.output_dir, output_filename)
                cv2.imwrite(output_path, annotated_image)
                self.get_logger().info(f"Saved annotated image to: {output_path}")
            except IOError as e:
                 self.get_logger().error(f"Could not save image to {output_path}: {e}")
            except Exception as e:
                 self.get_logger().error(f"Error saving image: {e}")

            # --- Publish the annotated image ---
            try:
                #annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
                #annotated_msg.header = msg.header # Use the same timestamp and frame_id
                print(coords)
                annotated_msg =  Float64MultiArray()
                annotated_msg.data = coords
                self.annotated_image_pub.publish(annotated_msg)
                self.get_logger().debug("Published Coords Oooohh yeahhh.")
            except CvBridgeError as e:
                self.get_logger().error(f"CvBridge Error publishing annotated image: {e}")
            except Exception as e:
                self.get_logger().error(f"Error publishing annotated image: {e}")

    def detect_objects(self, image):
        (h, w) = image.shape[:2]
        annotated_image = image.copy() # Work on a copy to keep original intact if needed

        # Create a blob from the image and perform a forward pass
        # Input size (416, 416) is common for YOLOv4-tiny
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)

        try:
            # Get the output layer names
            layer_names = self.net.getLayerNames()
            # Correct way to get output layer indices and names for OpenCV >= 4.x
            out_layer_indices = self.net.getUnconnectedOutLayers()
            # Check if the indices are returned as a 2D array [[idx]] or 1D array [idx]
            if isinstance(out_layer_indices, np.ndarray) and out_layer_indices.ndim > 1:
                 output_layers = [layer_names[i[0] - 1] for i in out_layer_indices]
            else:
                 output_layers = [layer_names[i - 1] for i in out_layer_indices.flatten()]

            # Run the forward pass to get detection outputs
            layer_outputs = self.net.forward(output_layers)
        except Exception as e:
            self.get_logger().error(f"DNN forward pass failed: {e}")
            return None # Return None if detection fails

        # Initialize lists for detected bounding boxes, confidences, and class IDs
        boxes = []
        confidences = []
        class_ids = []

        outputs = []
        # Loop over each of the layer outputs
        for output in layer_outputs:
            # Loop over each of the detections
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                # Filter out weak predictions
                if confidence > self.conf_threshold:
                    # Scale bounding box coordinates back relative to the image size
                    box = detection[0:4] * np.array([w, h, w, h])
                    (centerX, centerY, width, height) = box.astype("int")

                    # Use center coordinates to derive the top-left corner
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-maxima suppression
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)

        # Ensure at least one detection exists
        if len(idxs) > 0:
            # Loop over the indexes we are keeping
            for i in idxs.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w_box, h_box) = (boxes[i][2], boxes[i][3])

                # Get color for the class
                color = [int(c) for c in self.colors[class_ids[i]]]

                # Draw bounding box and label
                cv2.rectangle(annotated_image, (x, y), (x + w_box, y + h_box), color, 2)
                text = f"{self.classes[class_ids[i]]}: {confidences[i]:.2f}" # Shorter confidence format
                cv2.putText(annotated_image, text, (x, y - 5 if y - 5 > 5 else y + 15), # Adjust text position if near top
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                outputs.extend([float(class_ids[i]), (x + x + w_box)/2, (y + y + h_box)/2])
            self.get_logger().debug(f"Detected {len(idxs)} objects after NMS.")
        else:
             self.get_logger().debug("No objects detected meeting thresholds.")


        return annotated_image, outputs


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetectionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        print("YOLO Detection Node shutting down.")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception in main: {e}")
        else:
            print(f"Exception during node initialization: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
