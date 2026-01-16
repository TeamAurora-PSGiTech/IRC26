import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class MotorInterface(Node):
    def __init__(self):
        super().__init__("motor_interface")
        self.serial = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1)
        self.subscriber = self.create_subscription(Twist, topic='cmd_vel', callback=self.cmd_vel_callback, qos_profile=10)
        self.array = [0.0, 0.0]
        self.wheel_rad = 0.0125
        self.wheel_sep = 0.600
        self.command_speed = 138  # fixed magnitude sent to the motor controller
        self.create_rate(1)

    def cmd_vel_callback(self, msg):
        angular = msg.angular.z
        linear = msg.linear.x
        self.w_r = (linear / self.wheel_rad) + ((angular * self.wheel_sep) / (2.0 * self.wheel_rad))
        self.w_l = (linear / self.wheel_rad) - ((angular * self.wheel_sep) / (2.0 * self.wheel_rad))
        self.get_logger().info(f"Computed wheel velocities -> Right: {self.w_r}, Left: {self.w_l}")

        # Always send a fixed magnitude while preserving direction for turning/reversing.
        out_r = self.command_speed if self.w_r > 0 else -self.command_speed if self.w_r < 0 else 0
        out_l = self.command_speed if self.w_l > 0 else -self.command_speed if self.w_l < 0 else 0

        self.get_logger().info(f"Sending -> Right: {out_r}, Left: {out_l}")
        data = f"{out_l},{-out_r}\n"
        self.serial.write(data.encode())

def main(args = None):
    rclpy.init(args=args)
    interface = MotorInterface()
    rclpy.spin(interface)
    interface.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()