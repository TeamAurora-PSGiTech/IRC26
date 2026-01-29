import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from arm_ctrl.msg import Arm
import time


class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.pub = self.create_publisher(Arm, 'arm_cmd', 10)
        self.arm = Arm()

        self.bindings = {
            'w': ('link1',  1),
            's': ('link1', -1),
            'e': ('link2',  1),
            'd': ('link2', -1),
            'r': ('link3',  1),
            'f': ('link3', -1),
        }

        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

        self.get_logger().info("Wayland-safe keyboard teleop running")

    def spin(self):
        try:
            while rclpy.ok():
                key = sys.stdin.read(1)   # BLOCKING, single press
                if key in self.bindings:
                    joint, value = self.bindings[key]
                    setattr(self.arm, joint, value)

                    # publish pulse
                    self.pub.publish(self.arm)
                    time.sleep(0.15)  # pulse duration

                    # stop
                    setattr(self.arm, joint, 0)
                    self.pub.publish(self.arm)

        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def main():
    rclpy.init()
    node = ArmTeleop()
    try:
        node.spin()  # NOT rclpy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
