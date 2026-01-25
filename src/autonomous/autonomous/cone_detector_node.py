#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ConeDetectorNode(Node):
    def __init__(self):
        super().__init__('cone_detector')
        
        # Load YOLO model
        self.model = YOLO('/home/nikash/Documents/IRC26/src/autonomous/best (2).pt')
        self.bridge = CvBridge()
        
        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS for camera topics
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Subscribers
        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, qos)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, qos)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.info_callback, qos)
        
        # Publisher
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.image_pub = self.create_publisher(Image, '/cone_detector/annotated_image', 10)
        
        # Storage
        self.latest_color = None
        self.latest_depth = None
        self.intrinsics = None
        
        self.get_logger().info('Cone detector node initialized')
    
    def info_callback(self, msg):
        self.intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5]
        }
        self.get_logger().info('Camera intrinsics received')
    
    def color_callback(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_images()
    
    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.get_logger().info("Depth received")
        self.process_images()
    
    def process_images(self):
        if self.latest_color is None or self.intrinsics is None:
            return
        
        # Save current frame for debugging
        import cv2
        cv2.imwrite('/tmp/debug_frame.jpg', self.latest_color)
        
        self.get_logger().info("Running detection...")
        # Run detection
        results = self.model.predict(self.latest_color, imgsz=960, conf=0.25, iou=0.6, max_det=10)
        
        self.get_logger().info(f"Detection results: {len(results[0].boxes) if results and results[0].boxes else 0} detections")
        
        if results and len(results) > 0:
            boxes = results[0].boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    coords = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = coords
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    self.get_logger().info(f"Detected cone at ({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}) with conf {box.conf[0].cpu().numpy():.3f}")
                    
                    if self.latest_depth is not None:
                        # Get depth at center
                        if center_y < self.latest_depth.shape[0] and center_x < self.latest_depth.shape[1]:
                            depth = self.latest_depth[center_y, center_x] / 1000.0  # mm to m
                            if depth > 0:
                                # Deproject to 3D
                                X = (center_x - self.intrinsics['cx']) * depth / self.intrinsics['fx']
                                Y = (center_y - self.intrinsics['cy']) * depth / self.intrinsics['fy']
                                Z = depth
                                
                                # Transform to map frame
                                self.publish_goal(X, Y, Z)
                                break  # Set goal for first cone
        
        # Publish annotated image
        if results and results[0].boxes is not None and len(results[0].boxes) > 0:
            annotated_image = self.latest_color.copy()
            for box in results[0].boxes:
                coords = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = coords
                cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8'))
    
    def publish_goal(self, x, y, z):
        # Create pose in camera frame
        pose_camera = PoseStamped()
        pose_camera.header.frame_id = 'camera_depth_optical_frame'  # Adjust if needed
        pose_camera.header.stamp = rclpy.time.Time().to_msg()  # Use latest available time for TF
        pose_camera.pose.position.x = x
        pose_camera.pose.position.y = y
        pose_camera.pose.position.z = z
        pose_camera.pose.orientation.w = 1.0  # No rotation
        
        try:
            # Transform to map frame
            transform = self.tf_buffer.lookup_transform('map', pose_camera.header.frame_id, rclpy.time.Time())
            pose_map = tf2_geometry_msgs.do_transform_pose_stamped(pose_camera, transform)
            
            # Publish goal
            self.goal_pub.publish(pose_map)
            self.get_logger().info(f'Published goal: {pose_map.pose.position}')
        except Exception as e:
            self.get_logger().error(f'TF error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()