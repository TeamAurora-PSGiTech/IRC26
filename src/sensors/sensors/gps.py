import rclpy
from rclpy.node import Node
import serial
import pynmea2
import yaml
import os

from sensor_msgs.msg import NavSatFix


class GPSInterface(Node):
    def __init__(self):
        super().__init__("gps_interface")

        self.serial_port = "/dev/ttyACM0"
        try:
            self.serial = serial.Serial(self.serial_port, baudrate=115200, timeout=1)
        except serial.SerialException as exc:
            self.serial = None
            self.get_logger().error(f"Unable to open {self.serial_port}: {exc}")

        self.publisher = self.create_publisher(NavSatFix, 'gps', 10)

        self.coord = [float('nan'), float('nan'), 0.0]

        # ---- YAML related ----
        self.yaml_path = os.path.expanduser(
            "~/IRC26/src/config/navsat.yaml"
        )
        self.datum_written = False   # <---- VERY IMPORTANT FLAG

        self.timer = self.create_timer(0.1, self.gps_callback)

    def gps_callback(self):
        if self.serial is None or not self.serial.is_open:
            self.get_logger().warn("Serial connection unavailable; skipping read.")
            return

        try:
            raw = self.serial.readline().decode(errors="ignore").strip()
        except serial.SerialException as exc:
            self.get_logger().warn(f"Serial read failed: {exc}")
            return

        if not raw.startswith("$GNRMC"):
            return

        try:
            nmea_msg = pynmea2.parse(raw)
        except pynmea2.ParseError:
            return

        if getattr(nmea_msg, "status", "V") != "A":
            self.get_logger().info("No GPS fix yet.")
            return

        lat = nmea_msg.latitude
        lon = nmea_msg.longitude

        # ---------- WRITE DATUM ONCE ----------
        if not self.datum_written:
            self.write_datum_to_yaml(lat, lon)
            self.datum_written = True
        # ------------------------------------

        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps"
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = 0.0

        self.publisher.publish(gps_msg)

        self.get_logger().info(f"Latitude: {lat}, Longitude: {lon}")

    def write_datum_to_yaml(self, lat, lon):
        try:
            with open(self.yaml_path, "r") as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to read YAML: {e}")
            return

        try:
            data["navsat_transform"]["ros__parameters"]["datum"] = [
                round(lat, 7),
                round(lon, 7),
                0.0
            ]
        except KeyError as e:
            self.get_logger().error(f"YAML structure error: {e}")
            return

        try:
            with open(self.yaml_path, "w") as f:
                yaml.safe_dump(data, f, default_flow_style=False)
            self.get_logger().info(
                f"Datum written to YAML: [{lat}, {lon}, 0.0]"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to write YAML: {e}")


def main(args=None):
    rclpy.init(args=args)
    interface = GPSInterface()
    rclpy.spin(interface)
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
