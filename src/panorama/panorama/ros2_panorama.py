import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Imu, NavSatFix
from cv_bridge import CvBridge

import cv2
import numpy as np
import threading
import sys
import json
import math
from datetime import datetime

# ===================== UTILS =====================

def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return yaw % 360.0


def yaw_to_cardinal(yaw):
    """
    Relative cardinal directions (competition-safe)
    Offset calibrated for RealSense mounting.
    """
    corrected_yaw = (yaw - 65) % 360

    if corrected_yaw < 90:
        return "W"
    elif corrected_yaw < 180:
        return "S"
    elif corrected_yaw < 270:
        return "E"
    else:
        return "N"


def cylindrical_warp_with_depth(img, depth, f=1400):
    h, w = img.shape[:2]
    h_d, w_d = depth.shape[:2]

    cyl_img = np.zeros_like(img)
    cyl_depth = np.zeros_like(depth)

    for y in range(h):
        for x in range(w):
            theta = (x - w / 2) / f
            h_ = (y - h / 2) / f

            X = np.sin(theta)
            Y = h_
            Z = np.cos(theta)

            x_i = int(f * X / Z + w / 2)
            y_i = int(f * Y / Z + h / 2)

            x_i = max(0, min(x_i, w - 1))
            y_i = max(0, min(y_i, h - 1))
            cyl_img[y, x] = img[y_i, x_i]

    for y in range(h_d):
        for x in range(w_d):
            theta = (x - w_d / 2) / f
            h_ = (y - h_d / 2) / f

            X = np.sin(theta)
            Y = h_
            Z = np.cos(theta)

            x_i = int(f * X / Z + w_d / 2)
            y_i = int(f * Y / Z + h_d / 2)

            x_i = max(0, min(x_i, w_d - 1))
            y_i = max(0, min(y_i, h_d - 1))
            cyl_depth[y, x] = depth[y_i, x_i]

    return cyl_img, cyl_depth


def calculate_scale_from_depth(depth_image):
    valid = depth_image[(depth_image > 0) & (depth_image < 100.0)]
    if len(valid) == 0:
        return None

    return {
        "min_distance_m": float(np.min(valid)),
        "max_distance_m": float(np.max(valid)),
        "mean_distance_m": float(np.mean(valid)),
        "median_distance_m": float(np.median(valid)),
        "std_distance_m": float(np.std(valid))
    }

# ===================== ROS NODE =====================

class PanoramaNode(Node):
    def __init__(self):
        super().__init__("panorama_node")

        self.bridge = CvBridge()

        self.latest_frame = None
        self.latest_depth = None
        self.current_yaw = None
        self.gps_fix = None

        self.frames = []
        self.depths = []
        self.frame_metadata = []
        self.single_count = 0

        # Approx RealSense intrinsics
        self.fx = 615.0
        self.fy = 615.0
        self.cx = 320.0
        self.cy = 240.0

        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_cb,
            10
        )

        self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.depth_cb,
            10
        )

        self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_cb,
            10
        )

        self.create_subscription(
            NavSatFix,
            "/fix",
            self.gps_cb,
            10
        )

        self.get_logger().info("=== DEPTH-AWARE PANORAMA MODE ===")
        self.get_logger().info("a → capture SINGLE image")
        self.get_logger().info("s → capture panorama frame")
        self.get_logger().info("q → stitch panorama & exit")

        threading.Thread(target=self.key_listener, daemon=True).start()

    # ---------------- Callbacks ----------------

    def image_cb(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.current_yaw is not None:
            text = f"Yaw: {self.current_yaw:.1f}° ({yaw_to_cardinal(self.current_yaw)})"
            cv2.putText(
                self.latest_frame, text, (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )

        cv2.imshow("RealSense Live", self.latest_frame)
        cv2.waitKey(1)

    def depth_cb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.latest_depth = depth.astype(np.float32) / 1000.0

    def imu_cb(self, msg):
        self.current_yaw = quat_to_yaw(msg.orientation)

    def gps_cb(self, msg):
        self.gps_fix = msg

    # ---------------- Controls ----------------

    def key_listener(self):
        while rclpy.ok():
            cmd = sys.stdin.readline().strip()
            if cmd == "a":
                self.save_single()
            elif cmd == "s":
                self.capture_panorama_frame()
            elif cmd == "q":
                self.finish()
                break

    # ---------------- Save Single ----------------

    def save_single(self):
        if self.latest_frame is None or self.latest_depth is None:
            self.get_logger().warn("Waiting for camera/depth")
            return

        self.single_count += 1
        img = f"single_image_{self.single_count}.jpg"
        depth = f"single_depth_{self.single_count}.npy"
        meta = f"single_image_{self.single_count}.json"

        cv2.imwrite(img, self.latest_frame)
        np.save(depth, self.latest_depth)

        data = {
            "yaw_deg": self.current_yaw,
            "cardinal": yaw_to_cardinal(self.current_yaw),
            "gps": self.format_gps(),
            "scale": calculate_scale_from_depth(self.latest_depth),
            "timestamp": datetime.now().isoformat()
        }

        with open(meta, "w") as f:
            json.dump(data, f, indent=4)

        self.get_logger().info(f"Saved {img}")

    # ---------------- Panorama ----------------

    def capture_panorama_frame(self):
        self.frames.append(self.latest_frame.copy())
        self.depths.append(self.latest_depth.copy())

        self.frame_metadata.append({
            "yaw_deg": self.current_yaw,
            "cardinal": yaw_to_cardinal(self.current_yaw),
            "timestamp": datetime.now().isoformat()
        })

        self.get_logger().info(f"Captured frame {len(self.frames)}")

    def finish(self):
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()

    # ---------------- GPS ----------------

    def format_gps(self):
        if self.gps_fix is None or self.gps_fix.status.status < 0:
            return None

        return {
            "lat": self.gps_fix.latitude,
            "lon": self.gps_fix.longitude,
            "alt": self.gps_fix.altitude
        }

# ===================== MAIN =====================

def main():
    rclpy.init()
    node = PanoramaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

