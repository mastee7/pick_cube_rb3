#!/usr/bin/env python3
import sys
sys.path.append("/home/jetson/Desktop/Code/Dofbot/0.py_install/Arm_Lib")

from Arm_Lib import Arm_Device
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

# === Arm-related function stubs (Replace with real ones) ===
# from your_arm_library import Arm_device, move_to_xy, arm_clamp_block, move_reset, move_top
def clamp(val, min_val=0.0, max_val=1.0):
    """Clamp val into the range [min_val, max_val]."""
    return max(min_val, min(val, max_val))

def bilinear_angle(x, y, idx):
    """
    Bilinear interpolation of the 'idx'-th servo angle across the four corners.
      idx = 0 => servo #1, idx = 1 => servo #2, etc.
    """
    return ( (1 - x)*(1 - y)  *p_Green[idx]
           +      x  *(1 - y) *p_Blue[idx]
           + (1 - x)*     y   *p_Red[idx]
           +      x  *     y  *p_Yellow[idx] )

def move_to_xy(x, y, arm, s_time=1000):
    print(f"Moving to ({x}, {y}) with s_time={s_time}")
    # 1) Clamp x,y so that 0 <= x <= 1, 0 <= y <= 1
    x = clamp(x, 0.0, 1.0)
    y = clamp(y, 0.0, 1.0)

    # 2) Interpolate each servo angle #1..#5
    s1 = bilinear_angle(x, y, 0)
    s2 = bilinear_angle(x, y, 1)
    s3 = bilinear_angle(x, y, 2)
    s4 = bilinear_angle(x, y, 3)
    s5 = bilinear_angle(x, y, 4)

    # For servo #6, let's assume a fixed angle, e.g. 90 degrees:
    s6 = 180

    # 3) Round everything to an integer so the servo library is happy
    s1 = round(s1)
    s2 = round(s2*0.96)
    s3 = round(s3)
    s4 = round(s4*0.96)
    s5 = round(s5)
    # s6 is already an integer
    print(f"Interpolated angles => s1={s1}, s2={s2}, s3={s3}, s4={s4}, s5={s5}, s6={s6}")
    p = [s1, s2, s3, s4, s5, s6]
    for i in range(5):
        id = i + 1
        if id == 5:
            time.sleep(.1)
            arm.Arm_serial_servo_write(id, p[i], int(s_time*1.2))
        else :
            arm.Arm_serial_servo_write(id, p[i], s_time)
        time.sleep(.01)
    time.sleep(s_time/1000)

def arm_clamp_block(state, arm):
    print(f"Clamp {'closed' if state else 'opened'}")
    if state == 0:
        arm.Arm_serial_servo_write(6, 60, 400)
    else:
        arm.Arm_serial_servo_write(6, 140, 200)

    time.sleep(.5)

def move_reset(arm):
    print("Arm reset")
    p, s_time = [90, 130, 0, 0, 9], 1000
    for i in range(5):
        id = i + 1
        if id == 5:
            time.sleep(.1)
            arm.Arm_serial_servo_write(id, p[i], int(s_time*1.2))
        else :
            arm.Arm_serial_servo_write(id, p[i], s_time)
        time.sleep(.01)
    time.sleep(s_time/1000)

def move_top(arm):
    print("Arm moved to top position")
    p, s_time = [90, 80, 50, 50, 270], 1000
    for i in range(5):
        id = i + 1
        if id == 5:
            time.sleep(.1)
            arm.Arm_serial_servo_write(id, p[i], int(s_time*1.2))
        else :
            arm.Arm_serial_servo_write(id, p[i], s_time)
        time.sleep(.01)
    time.sleep(s_time/1000)

# === Coordinate Mapping ===
def real_world_to_pixel(rx, ry):
    PIXEL_LEFT_BOTTOM = (506.5, 515.0)
    PIXEL_RIGHT_TOP = (781.0, 301.0)
    REAL_LEFT_BOTTOM = (0.0, 0.0)
    REAL_RIGHT_TOP = (1.0, 1.0)

    delta_pixel_x = PIXEL_RIGHT_TOP[0] - PIXEL_LEFT_BOTTOM[0]
    delta_pixel_y = PIXEL_RIGHT_TOP[1] - PIXEL_LEFT_BOTTOM[1]
    delta_real_x = REAL_RIGHT_TOP[0] - REAL_LEFT_BOTTOM[0]
    delta_real_y = REAL_RIGHT_TOP[1] - REAL_LEFT_BOTTOM[1]

    fraction_x = (rx - REAL_LEFT_BOTTOM[0]) / delta_real_x
    fraction_y = (ry - REAL_LEFT_BOTTOM[1]) / delta_real_y

    px = fraction_x * delta_pixel_x + PIXEL_LEFT_BOTTOM[0]
    py = fraction_y * delta_pixel_y + PIXEL_LEFT_BOTTOM[1]
    return (px, py)

# === Action Sequence ===
def execute_pick_and_place(plan, arm):
    if len(plan) != 4:
        raise ValueError("Plan must be a list of four values: [x_init, y_init, x_tgt, y_tgt]")

    x_init, y_init, x_tgt, y_tgt = plan
    x1, y1 = real_world_to_pixel(x_init, y_init)
    x2, y2 = real_world_to_pixel(x_tgt, y_tgt)

    arm_clamp_block(0, arm)
    move_reset(arm)
    move_to_xy(x1, y1, arm, s_time=1000)
    time.sleep(1)
    arm_clamp_block(1, arm)
    move_top(arm)
    move_to_xy(x2, y2, arm, s_time=1000)
    time.sleep(1)
    arm_clamp_block(0, arm)

# === ROS2 Node ===
class NanoSubscriber(Node):
    def __init__(self):
        super().__init__('nano_message_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'rb3_topic',
            self.listener_callback,
            10)
        self.get_logger().info('NanoSubscriber started, listening to "rb3_topic"...')

        self.arm = Arm_Device()
        self.running = False

    def listener_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"Received coordinates: {data}")
        
        if len(data) != 4:
            self.get_logger().error("Invalid message length. Expecting [x_init, y_init, x_tgt, y_tgt]")
            return

        if self.running:
            self.get_logger().info("Currently executing, skipping new command")
            return

        self.running = True
        try:
            execute_pick_and_place(data, self.arm)
        except Exception as e:
            self.get_logger().error(f"Error during execution: {e}")
        finally:
            self.running = False

def main(args=None):
    rclpy.init(args=args)
    node = NanoSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

