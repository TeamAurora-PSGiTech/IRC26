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
    Updated cardinal direction mapping: SWNE rotation.
    Real life 0° = RealSense 65°
    Subtract 65° offset, then map: 0-90°=S, 90-180°=W, 180-270°=N, 270-360°=E
    Rotated 90° clockwise from original ESWN to SWNE.
    """
    # Apply offset correction
    corrected_yaw = (yaw - 65) % 360
    
    # Cardinal mapping rotated 90° clockwise: S, W, N, E
    if corrected_yaw < 90:
        return "W"
    elif corrected_yaw < 180:
        return "S"
    elif corrected_yaw < 270:
        return "E"
    else:
        return "N"


def cylindrical_warp(img, f=1400):
    h, w = img.shape[:2]
    cyl = np.zeros_like(img)

    for y in range(h):
        for x in range(w):
            theta = (x - w / 2) / f
            h_ = (y - h / 2) / f

            X = np.sin(theta)
            Y = h_
            Z = np.cos(theta)

            x_i = int(f * X / Z + w / 2)
            y_i = int(f * Y / Z + h / 2)

            if 0 <= x_i < w and 0 <= y_i < h:
                cyl[y, x] = img[y_i, x_i]

    return cyl

def cylindrical_warp_with_depth(img, depth, f=1400):
    """
    Warp both RGB and depth images cylindrically
    Fixed boundary checking with clipping to prevent index out of bounds
    """
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

            # Clip indices to valid range for RGB image
            x_i = max(0, min(x_i, w - 1))
            y_i = max(0, min(y_i, h - 1))
            
            cyl_img[y, x] = img[y_i, x_i]
    
    # Process depth separately with its own dimensions
    for y in range(h_d):
        for x in range(w_d):
            theta = (x - w_d / 2) / f
            h_ = (y - h_d / 2) / f

            X = np.sin(theta)
            Y = h_
            Z = np.cos(theta)

            x_i = int(f * X / Z + w_d / 2)
            y_i = int(f * Y / Z + h_d / 2)

            # Clip indices to valid range for depth image
            x_i = max(0, min(x_i, w_d - 1))
            y_i = max(0, min(y_i, h_d - 1))
            
            cyl_depth[y, x] = depth[y_i, x_i]

    return cyl_img, cyl_depth

def build_panorama_with_depth(frames, depths):
    """
    Build panorama from RGB and depth frames
    For depth, we simply concatenate horizontally since cylindrical warp already aligns them
    """
    # Warp frames
    warped_frames = []
    warped_depths = []
    
    for frame, depth in zip(frames, depths):
        w_frame, w_depth = cylindrical_warp_with_depth(frame, depth)
        warped_frames.append(w_frame)
        warped_depths.append(w_depth)

    # Stitch RGB
    stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    status, pano = stitcher.stitch(warped_frames)

    if status != cv2.Stitcher_OK:
        return None, None

    # For depth: simple horizontal concatenation works better than feature-based stitching
    # since depth images lack the texture features needed for matching
    try:
        # Find minimum height
        min_height = min([d.shape[0] for d in warped_depths])
        
        # Resize all depths to same height
        resized_depths = []
        for d in warped_depths:
            if d.shape[0] != min_height:
                scale = min_height / d.shape[0]
                new_width = int(d.shape[1] * scale)
                d_resized = cv2.resize(d, (new_width, min_height))
                resized_depths.append(d_resized)
            else:
                resized_depths.append(d)
        
        # Concatenate horizontally
        pano_depth = np.hstack(resized_depths)
        
    except Exception as e:
        print(f"⚠️ Depth concatenation failed: {e}")
        pano_depth = None

    # Crop black borders from RGB
    gray = cv2.cvtColor(pano, cv2.COLOR_BGR2GRAY)
    ys, xs = np.where(gray > 0)
    pano = pano[ys.min():ys.max(), xs.min():xs.max()]
    
    # Crop depth to match (roughly)
    if pano_depth is not None:
        # Resize depth to match panorama dimensions
        pano_depth = cv2.resize(pano_depth, (pano.shape[1], pano.shape[0]))

    # Adjust aspect ratio
    h, w = pano.shape[:2]
    target_w = 3 * h
    if w > target_w:
        x = (w - target_w) // 2
        pano = pano[:, x:x + target_w]
        if pano_depth is not None:
            pano_depth = pano_depth[:, x:x + target_w]

    return pano, pano_depth

def create_point_cloud(depth_image, rgb_image, fx, fy, cx, cy):
    """
    Create 3D point cloud from depth image
    Returns: points (Nx3), colors (Nx3)
    """
    h, w = depth_image.shape
    points = []
    colors = []
    
    for v in range(h):
        for u in range(w):
            z = depth_image[v, u]
            if z > 0:  # Valid depth
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])
                colors.append(rgb_image[v, u][::-1] / 255.0)  # BGR to RGB, normalize
    
    return np.array(points), np.array(colors)

def calculate_scale_from_depth(depth_image):
    """
    Calculate scale information from depth data
    Returns dict with depth statistics
    Filter out invalid/extreme depth values
    """
    valid_depths = depth_image[depth_image > 0]
    
    if len(valid_depths) == 0:
        return None
    
    # Filter out extreme values (likely errors) - keep depths < 100m
    valid_depths = valid_depths[valid_depths < 100.0]
    
    if len(valid_depths) == 0:
        return None
    
    return {
        "min_distance_m": float(np.min(valid_depths)),
        "max_distance_m": float(np.max(valid_depths)),
        "mean_distance_m": float(np.mean(valid_depths)),
        "median_distance_m": float(np.median(valid_depths)),
        "std_distance_m": float(np.std(valid_depths))
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

        # RealSense D455 intrinsics (approximate - adjust if needed)
        self.fx = 615.0  # Focal length X
        self.fy = 615.0  # Focal length Y
        self.cx = 320.0  # Principal point X
        self.cy = 240.0  # Principal point Y

        # Subscriptions
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

        print("\n=== DEPTH-AWARE PANORAMA MODE ===")
        print("a → capture SINGLE image + depth + GPS + JSON")
        print("s → capture ONE frame with depth for panorama")
        print("q → stitch panorama with scale + save point cloud & exit")
        print("\nNote: Make sure depth topic is publishing!")
        print("Check: ros2 topic echo /camera/camera/depth/image_rect_raw\n")

        threading.Thread(target=self.key_listener, daemon=True).start()

    # ---------------- Callbacks ----------------

    def image_cb(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.current_yaw is not None:
            card = yaw_to_cardinal(self.current_yaw)
            
            text1 = f"Yaw: {self.current_yaw:.1f} deg ({card})"
            text2 = f"Panorama frames: {len(self.frames)}"
            text3 = f"Depth: {'OK' if self.latest_depth is not None else 'WAITING'}"
            
            cv2.putText(self.latest_frame, text1, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(self.latest_frame, text2, (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
            cv2.putText(self.latest_frame, text3, (20, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                        (0, 255, 0) if self.latest_depth is not None else (0, 0, 255), 2)

        cv2.imshow("RealSense Live", self.latest_frame)
        cv2.waitKey(1)

    def depth_cb(self, msg):
        # Depth is in mm for RealSense, convert to meters
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.latest_depth = self.latest_depth.astype(np.float32) / 1000.0  # mm to meters

    def imu_cb(self, msg):
        self.current_yaw = quat_to_yaw(msg.orientation)

    def gps_cb(self, msg):
        self.gps_fix = msg

    # ---------------- Controls ----------------

    def key_listener(self):
        while True:
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
        if self.latest_frame is None or self.current_yaw is None:
            print("⚠️ Waiting for camera / IMU")
            return

        if self.latest_depth is None:
            print("⚠️ Waiting for depth data")
            return

        self.single_count += 1
        img_name = f"single_image_{self.single_count}.jpg"
        depth_name = f"single_depth_{self.single_count}.npy"
        json_name = f"single_image_{self.single_count}.json"

        cv2.imwrite(img_name, self.latest_frame)
        np.save(depth_name, self.latest_depth)

        # Calculate scale info
        scale_info = calculate_scale_from_depth(self.latest_depth)

        meta = {
            "type": "single_image",
            "image": img_name,
            "depth_file": depth_name,
            "yaw_deg": self.current_yaw,
            "cardinal": yaw_to_cardinal(self.current_yaw),
            "gps": self.format_gps(),
            "scale": scale_info,
            "camera_intrinsics": {
                "fx": self.fx,
                "fy": self.fy,
                "cx": self.cx,
                "cy": self.cy
            },
            "timestamp": datetime.now().isoformat()
        }

        with open(json_name, "w") as f:
            json.dump(meta, f, indent=4)

        print(f"📸 Saved {img_name} + {depth_name} + {json_name}")
        if scale_info:
            print(f"   Depth range: {scale_info['min_distance_m']:.2f}m - {scale_info['max_distance_m']:.2f}m")

    # ---------------- Capture Panorama Frame ----------------

    def capture_panorama_frame(self):
        if self.latest_frame is None or self.current_yaw is None:
            print("⚠️ Waiting for camera / IMU")
            return

        if self.latest_depth is None:
            print("⚠️ Waiting for depth data")
            return

        # Store frame, depth, and metadata
        self.frames.append(self.latest_frame.copy())
        self.depths.append(self.latest_depth.copy())
        
        scale_info = calculate_scale_from_depth(self.latest_depth)
        
        frame_meta = {
            "frame_number": len(self.frames),
            "yaw_deg": self.current_yaw,
            "cardinal": yaw_to_cardinal(self.current_yaw),
            "gps": self.format_gps(),
            "scale": scale_info,
            "timestamp": datetime.now().isoformat()
        }
        
        self.frame_metadata.append(frame_meta)
        
        depth_str = f" (depth: {scale_info['mean_distance_m']:.2f}m)" if scale_info else ""
        print(f"📸 Frame {len(self.frames)} - {self.current_yaw:.1f}° ({yaw_to_cardinal(self.current_yaw)}){depth_str}")

    # ---------------- Finish Panorama ----------------

    def finish(self):
        cv2.destroyAllWindows()

        if len(self.frames) >= 2:
            print(f"\n🔨 Stitching {len(self.frames)} frames with depth data...")
            pano, pano_depth = build_panorama_with_depth(self.frames, self.depths)

            if pano is not None:
                # Save RGB panorama
                cv2.imwrite("panorama.jpg", pano)
                print("✅ Saved panorama.jpg")

                # Save depth panorama
                if pano_depth is not None:
                    np.save("panorama_depth.npy", pano_depth)
                    
                    # Create depth visualization
                    depth_vis = cv2.normalize(pano_depth, None, 0, 255, cv2.NORM_MINMAX)
                    depth_vis = depth_vis.astype(np.uint8)
                    depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    cv2.imwrite("panorama_depth_viz.jpg", depth_colored)
                    print("✅ Saved panorama_depth.npy and panorama_depth_viz.jpg")

                    # Calculate overall scale
                    overall_scale = calculate_scale_from_depth(pano_depth)
                else:
                    overall_scale = None
                    print("⚠️ Depth stitching failed, saved RGB only")

                # Generate point cloud from panorama
                if pano_depth is not None:
                    print("🔨 Generating point cloud...")
                    points, colors = create_point_cloud(
                        pano_depth, pano, 
                        self.fx, self.fy, self.cx, self.cy
                    )
                    
                    # Save as PLY
                    self.save_point_cloud_ply("panorama_pointcloud.ply", points, colors)
                    print(f"✅ Saved point cloud with {len(points)} points")

                # Create metadata
                meta = {
                    "type": "panorama",
                    "image": "panorama.jpg",
                    "depth_file": "panorama_depth.npy" if pano_depth is not None else None,
                    "depth_visualization": "panorama_depth_viz.jpg" if pano_depth is not None else None,
                    "point_cloud": "panorama_pointcloud.ply" if pano_depth is not None else None,
                    "total_frames": len(self.frames),
                    "yaw_start": self.frame_metadata[0]["yaw_deg"],
                    "yaw_end": self.frame_metadata[-1]["yaw_deg"],
                    "cardinal_start": self.frame_metadata[0]["cardinal"],
                    "cardinal_end": self.frame_metadata[-1]["cardinal"],
                    "yaw_sweep": (self.frame_metadata[-1]["yaw_deg"] - self.frame_metadata[0]["yaw_deg"]) % 360,
                    "scale": overall_scale,
                    "camera_intrinsics": {
                        "fx": self.fx,
                        "fy": self.fy,
                        "cx": self.cx,
                        "cy": self.cy
                    },
                    "frames": self.frame_metadata,
                    "timestamp": datetime.now().isoformat()
                }

                with open("panorama.json", "w") as f:
                    json.dump(meta, f, indent=4)

                print("✅ Saved panorama.json")
                print(f"   Frames: {len(self.frames)}")
                print(f"   Yaw: {self.frame_metadata[0]['yaw_deg']:.1f}° → {self.frame_metadata[-1]['yaw_deg']:.1f}°")
                print(f"   Cardinal: {self.frame_metadata[0]['cardinal']} → {self.frame_metadata[-1]['cardinal']}")
                if overall_scale:
                    print(f"   Depth range: {overall_scale['min_distance_m']:.2f}m - {overall_scale['max_distance_m']:.2f}m")
            else:
                print("❌ Panorama stitching failed")
        elif len(self.frames) == 1:
            print("❌ Need at least 2 frames for panorama")
        else:
            print("⚠️ No frames captured")

        self.destroy_node()
        rclpy.shutdown()

    # ---------------- Point Cloud Saving ----------------

    def save_point_cloud_ply(self, filename, points, colors):
        """
        Save point cloud in PLY format
        """
        with open(filename, 'w') as f:
            # Header
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")
            
            # Data
            for point, color in zip(points, colors):
                r, g, b = (color * 255).astype(int)
                f.write(f"{point[0]} {point[1]} {point[2]} {r} {g} {b}\n")

    # ---------------- GPS Formatting ----------------

    def format_gps(self):
        if self.gps_fix is None or self.gps_fix.status.status < 0:
            return {
                "latitude": None,
                "longitude": None,
                "elevation_m": None,
                "accuracy_m": None
            }

        accuracy = None
        if self.gps_fix.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            accuracy = math.sqrt(
                self.gps_fix.position_covariance[0] +
                self.gps_fix.position_covariance[4]
            )

        return {
            "latitude": self.gps_fix.latitude,
            "longitude": self.gps_fix.longitude,
            "elevation_m": self.gps_fix.altitude,
            "accuracy_m": accuracy
        }

# ===================== MAIN =====================

def main():
    rclpy.init()
    node = PanoramaNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
