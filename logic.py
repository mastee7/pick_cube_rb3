import rclpy
from rclpy.node import Node
from your_custom_msgs.msg import Detection2DArray
from std_msgs.msg import Float64MultiArray

class LogicNode(Node):
    def __init__(self):
        super().__init__('logic_node')
        self.subscription_ = self.create_subscription(
            Detection2DArray, '/vision/detections', self.detections_callback, 10
        )
        self.motor_pub_ = self.create_publisher(
            Float64MultiArray, '/dofbot/target_joints', 10
        )

    def detections_callback(self, detections):
        # Decide what to do based on detections
        # E.g., find the largest bounding box, compute some logic
        # Suppose we want to move the arm to a “pickup” configuration
        msg = Float64MultiArray()
        # For instance, [base_angle, shoulder_angle, elbow_angle, ...]
        msg.data = [90.0, 45.0, 30.0, 0.0, 0.0, 0.0]
        self.motor_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
