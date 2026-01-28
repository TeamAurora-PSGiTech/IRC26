import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyTwist(Node):
    def __init__(self):
        super().__init__('joy_twist')

        self.sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- MAPPING ----
        self.enable_button = 0      # ✅ verified enable button
        self.linear_axis = 1        # forward / backward
        self.angular_axis = 0       # left / right

        # D-pad axes
        self.linear_speed_axis = 7  # UP / DOWN  → w / x
        self.angular_speed_axis = 6 # RIGHT / LEFT → e / c

        # ---- SPEED VALUES ----
        self.linear_scale = 0.5
        self.angular_scale = 1.0

        # ---- LIMITS ----
        self.min_linear = 0.05
        self.max_linear = 5.0
        self.min_angular = 0.1
        self.max_angular = 5.0

        # ---- DEBOUNCE STATE (IMPORTANT) ----
        self.last_linear_dpad = 0.0
        self.last_angular_dpad = 0.0

    def joy_cb(self, msg):
        twist = Twist()

        # ---- DEADMAN ----
        if msg.buttons[self.enable_button] == 0:
            self.pub.publish(twist)
            return

        # ---- LINEAR SPEED (w / x) ----
        linear_dpad = msg.axes[self.linear_speed_axis]
        if linear_dpad != self.last_linear_dpad:
            if linear_dpad == 1.0:
                self.linear_scale *= 1.1
            elif linear_dpad == -1.0:
                self.linear_scale *= 0.9

        self.last_linear_dpad = linear_dpad

        # Clamp
        self.linear_scale = max(
            self.min_linear, min(self.linear_scale, self.max_linear))

        # ---- ANGULAR SPEED (e / c) ----
        angular_dpad = msg.axes[self.angular_speed_axis]
        if angular_dpad != self.last_angular_dpad:
            if angular_dpad == 1.0:
                self.angular_scale *= 1.1
            elif angular_dpad == -1.0:
                self.angular_scale *= 0.9

        self.last_angular_dpad = angular_dpad

        # Clamp
        self.angular_scale = max(
            self.min_angular, min(self.angular_scale, self.max_angular))

        # ---- MOTION ----
        twist.linear.x = msg.axes[self.linear_axis] * self.linear_scale
        twist.angular.z = msg.axes[self.angular_axis] * self.angular_scale

        self.pub.publish(twist)


def main():
    rclpy.init()
    node = JoyTwist()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

