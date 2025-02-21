import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial

class ControlNode(Node):
    def __init__(self):
        super().__init__('dofbot_driver_node')
        self.subscription_ = self.create_subscription(
            Float64MultiArray, '/dofbot/target_joints', self.joint_command_callback, 10
        )
        # Open serial to DOFBOT servo controller
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    def joint_command_callback(self, msg):
        # Convert angles to servo commands
        angles = msg.data  # e.g. [90.0, 45.0, 30.0, ...]
        self.send_servo_commands(angles)

    def send_servo_commands(self, angles):
        # Example: #1P1500, #2P1400, etc. for each servo
        # This depends heavily on the servo controller protocol
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
