import rclpy
from rclpy.node import Node
import serial

from arm_ctrl_msgs.msg import Arm


class ArmInterface(Node):
    def __init__(self):
        super().__init__("arm_interface")
        self.serial = serial.Serial("/dev/ttyACM1", baudrate=9600, timeout=1)
        self.subscriber = self.create_subscription(
            Arm,
            topic="arm_cmd",
            callback=self.arm_cmd_callback,
            qos_profile=10,
        )
        self.create_rate(2)

    def arm_cmd_callback(self, msg: Arm):
        # Arduino expects single angle values (0-180)
        servo1_angle = int(msg.servo1_angle[0]) if msg.servo1_angle else 5
        servo2_angle = int(msg.servo2_angle[0]) if msg.servo2_angle else 5

        # Format: base,link1,link2,link3,tool,servo1_dir,servo2_dir,servo1_angle,servo2_angle
        data = (
            f"{int(msg.base)},"
            f"{int(msg.link1)},{int(msg.link2)},{int(msg.link3)},"
            f"{int(msg.tool)},"
            f"{int(msg.servo1_dir)},{int(msg.servo2_dir)},"
            f"{servo1_angle},{servo2_angle}\n"
        )

        self.get_logger().info(
            f"Arm cmd -> base={msg.base} link1={msg.link1} link2={msg.link2} link3={msg.link3} "
            f"tool={msg.tool} servo1_dir={msg.servo1_dir} servo2_dir={msg.servo2_dir} "
            f"servo1_angle={servo1_angle} servo2_angle={servo2_angle}"
        )

        self.serial.write(data.encode())


def main(args=None):
    rclpy.init(args=args)
    interface = ArmInterface()
    rclpy.spin(interface)
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
