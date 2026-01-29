import sys
import pynput
import rclpy
from rclpy.node import Node
from arm_ctrl_msgs.msg import Arm
import time

KEY_BINDINGS = {
    'w': ('link1',  1),
    's': ('link1', -1),
    'e': ('link2',  1),
    'd': ('link2', -1),
    'r': ('link3',  1),
    'f': ('link3', -1),
    ",": ('base',  1),
    ".": ('base', -1),
    "i": ('servo_dir',  (1, 1)),
    "k": ('servo_dir',  (-1, -1)),
    "j": ('servo_dir',  (-1, 1)),
    "l": ('servo_dir',  (1, -1)),
    "u": ('tool', 1),
    "o": ('tool', 0),
    " ": ("stop", None),
}


class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.pub = self.create_publisher(Arm, 'arm_cmd', 10)
        self.get_logger().info("Use w/s, e/d, r/f to control link1, link2, link3, ,/. for base rotation, i/k/j/l for servo directions, u/o for tool, space to stop.")
        self.arm = Arm()
        self._timer = self.create_timer(0.1, self.timer_callback)
        self.listener = pynput.keyboard.Listener(on_press=self.on_press)
        self.listener.start()
    def on_press(self, key):
        try:
            char = key.char
        except AttributeError:
            return

        if char in KEY_BINDINGS:
            command = KEY_BINDINGS[char]
            if command[0] == 'stop':
                self.arm = Arm()  # Reset all commands
            elif command[0] == 'servo_dir':
                self.arm.servo1_dir += command[1][0]
                self.arm.servo2_dir += command[1][1]
            else:
                current_value = getattr(self.arm, command[0])
                setattr(self.arm, command[0], current_value + command[1])

            self.get_logger().info(f'Key pressed: {char}, updated arm command: {self.arm}')

    def timer_callback(self):
        self.pub.publish(self.arm)

def main(args=None):
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