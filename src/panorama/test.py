import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import os
from datetime import datetime

# ===================== UTILS =====================

def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy)) % 360

def yaw_to_cardinal(y):
    if y < 45 or y >= 315: return "N"
    if y < 135: return "E"
    if y < 225: return "S"
    return "W"

def cylindrical_warp(img, f=900):
    h, w = img.shape[:2]
    cyl = np.zeros_like(img)
    for y in range(h):
        for x in range(w):
            theta = (x - w / 2) / f
            h_ = (y - h / 2) / f
            X = math.sin(theta)
            Z = math.cos(theta)
            x_i = int(f * X / Z + w / 2)
            y_i = int(f * h_ / Z + h / 2)
            if 0 <= x_i < w and 0 <= y_i < h:
                cyl[y, x] = img[y_i, x_i]
    return cyl

def stitch_images(frames):
    stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    status, pano = stitcher.stitch(frames)
    if status == cv2.Stitcher_OK:
        return pano
    warped = [cylindrical_warp(f) for f in frames]
    status, pano = stitcher.stitch(warped)
    return pano if status == cv2.Stitcher_OK else None

def depth_statistics(depths):
    vals = np.concatenate([d[d > 0] for d in depths])
    vals = vals[vals < 100.0]
    if len(vals) == 0:
        return None
    return {
        "min": float(vals.min()),
        "mean": float(vals.mean()),
        "max": float(vals.max())
    }

# ===================== OVERLAYS =====================

def draw_overlays(pano, meta, depth_stats):
    h, w = pano.shape[:2]

    # --- Cardinal direction scale ---
    for i in range(0, 361, 45):
        x = int((i / 360) * w)
        cv2.line(pano, (x, 0), (x, 30), (255,255,255), 2)
        cv2.putText(pano, yaw_to_cardinal(i),
                    (x+5, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255,255,255), 2)

    # --- GPS per frame ---
    for m in meta:
        x = int((m["yaw"] / 360) * w)
        txt = f"{m['lat']:.5f},{m['lon']:.5f}"
        cv2.putText(pano, txt, (x-80, h-90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,255), 1)

    # --- Elevation + Accuracy ---
    base = meta[len(meta)//2]
    elev_txt = f"Elevation: {base['alt']:.1f} m | GPS Accuracy: ±{base['acc']:.1f} m"
    cv2.putText(pano, elev_txt, (20, h-60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # --- Depth scale ---
    if depth_stats:
        depth_txt = (
            f"Depth (m): min {depth_stats['min']:.2f} | "
            f"mean {depth_stats['mean']:.2f} | "
            f"max {depth_stats['max']:.2f}"
        )
        cv2.putText(pano, depth_txt, (20, h-30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # --- Timestamp ---
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cv2.putText(pano, f"Captured: {timestamp}",
                (w-350, h-20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    # --- Stratigraphy (example overlay) ---
    layers = [
        ("Sandstone", 120, 170),
        ("Shale", 170, 230),
        ("Limestone", 230, 300)
    ]
    for name, y1, y2 in layers:
        cv2.rectangle(pano, (0, y1), (w, y2), (255,255,255), 1)
        cv2.putText(pano, name, (20, y1+30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    return pano

# ===================== ROS NODE =====================

class PanoramaNode(Node):
    def __init__(self):
        super().__init__("panorama_node")

        self.bridge = CvBridge()

        self.rgb = None
        self.depth = None
        self.yaw = None
        self.gps = None

        self.frames = []
        self.depths = []
        self.meta = []

        self.out_dir = os.path.expanduser("~/panorama_output")
        os.makedirs(self.out_dir, exist_ok=True)

        # Subscribers
        self.create_subscription(Image, "/camera/color/image_raw", self.rgb_cb, 10)
        self.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw", self.depth_cb, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_cb, 10)
        self.create_subscription(NavSatFix, "/fix", self.gps_cb, 10)

        # Publisher
        self.panorama_pub = self.create_publisher(Image, "/panorama/image", 10)

        self.get_logger().info("Panorama node running")
        self.get_logger().info("Press 's' to capture | 'q' to stitch & publish")

    def rgb_cb(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.yaw is not None:
            cv2.putText(self.rgb,
                        f"Yaw: {self.yaw:.1f}° ({yaw_to_cardinal(self.yaw)}) | Frames: {len(self.frames)}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)

        cv2.imshow("Live View", self.rgb)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            self.capture()
        elif key == ord('q'):
            self.finish()
            self.shutdown()

    def depth_cb(self, msg):
        d = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth = d.astype(np.float32) / 1000.0

    def imu_cb(self, msg):
        self.yaw = quat_to_yaw(msg.orientation)

    def gps_cb(self, msg):
        if msg.status.status < 0:
            return
        acc = math.sqrt(msg.position_covariance[0] + msg.position_covariance[4]) \
              if msg.position_covariance_type != 0 else 0.0
        self.gps = (msg.latitude, msg.longitude, msg.altitude, acc)

    def capture(self):
        if None in (self.rgb, self.depth, self.yaw, self.gps):
            self.get_logger().warn("Waiting for all sensors...")
            return

        idx = len(self.frames) + 1
        self.frames.append(self.rgb.copy())
        self.depths.append(self.depth.copy())
        self.meta.append({
            "yaw": self.yaw,
            "lat": self.gps[0],
            "lon": self.gps[1],
            "alt": self.gps[2],
            "acc": self.gps[3]
        })

        cv2.imwrite(f"{self.out_dir}/frame_{idx:02d}.jpg", self.rgb)
        self.get_logger().info(f"Captured frame {idx}")

    def finish(self):
        if len(self.frames) < 2:
            self.get_logger().error("Need at least 2 frames")
            return

        ordered = sorted(zip(self.frames, self.meta), key=lambda x: x[1]["yaw"])
        frames = [f for f,_ in ordered]
        meta = [m for _,m in ordered]

        pano = stitch_images(frames)
        if pano is None:
            self.get_logger().error("Stitching failed")
            return

        stats = depth_statistics(self.depths)
        pano = draw_overlays(pano, meta, stats)

        out_path = f"{self.out_dir}/panorama_final.jpg"
        cv2.imwrite(out_path, pano)

        pano_msg = self.bridge.cv2_to_imgmsg(pano, encoding="bgr8")
        self.panorama_pub.publish(pano_msg)

        self.get_logger().info(f"Panorama saved & published → {out_path}")
        cv2.imshow("PANORAMA", pano)
        cv2.waitKey(0)

    def shutdown(self):
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = PanoramaNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
