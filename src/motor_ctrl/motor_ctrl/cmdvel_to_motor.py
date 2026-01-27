import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelToMotor(Node):
    def __init__(self):
        super().__init__('cmdvel_to_motor')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmdvel_cb,
            10
        )

        # Robot parameters (CHANGE LATER if needed)
        self.wheel_base = 0.3   # distance between wheels (meters)
        self.max_speed = 5.0    # max motor speed (your scale)

        self.get_logger().info("cmd_vel → motor node started")

    def cmdvel_cb(self, msg: Twist):
        v = msg.linear.x      # forward velocity
        w = msg.angular.z     # angular velocity

        # Differential drive equations
        left_speed  = v - (w * self.wheel_base / 2.0)
        right_speed = v + (w * self.wheel_base / 2.0)

        # Clamp (safety)
        left_speed = max(-self.max_speed, min(left_speed, self.max_speed))
        right_speed = max(-self.max_speed, min(right_speed, self.max_speed))

        # DEBUG OUTPUT (IMPORTANT)
        self.get_logger().info(
            f"LEFT: {left_speed:.2f} | RIGHT: {right_speed:.2f}"
        )

        # 🚨 LATER we will send this to interface.py


def main():
    rclpy.init()
    node = CmdVelToMotor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

