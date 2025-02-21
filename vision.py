import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from your_custom_msgs.msg import Detection2DArray  # or create your own
from cv_bridge import CvBridge

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription_ = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.publisher_ = self.create_publisher(
            Detection2DArray, '/vision/detections', 10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Run your AI inference here
        detections = Detection2DArray()
        # Fill detections with bounding boxes, classes, etc.
        self.publisher_.publish(detections)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
