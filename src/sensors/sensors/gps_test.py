#!/usr/bin/env python3
"""
GPS Conversion Test Tool

Tests GPS to map coordinate conversion and visualizes in RViz.

Usage:
    ros2 run sensors gps_test --ros-args -p lat:=<lat> -p lon:=<lon>
    
Or it will continuously convert your current GPS position.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from robot_localization.srv import FromLL
import math


class GPSTestNode(Node):
    def __init__(self):
        super().__init__('gps_test')
        
        # Parameters for test GPS coordinates
        self.declare_parameter('lat', 0.0)
        self.declare_parameter('lon', 0.0)
        
        # Service client
        self.fromLL_client = self.create_client(FromLL, '/fromLL')
        
        # Publishers for visualization
        self.point_pub = self.create_publisher(PointStamped, '/gps_test/map_point', 10)
        self.marker_pub = self.create_publisher(Marker, '/gps_test/marker', 10)
        
        # Subscribe to current GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps',
            self.gps_callback,
            10
        )
        
        self.current_gps = None
        self.test_mode = False
        
        # Check if test coordinates were provided
        test_lat = self.get_parameter('lat').value
        test_lon = self.get_parameter('lon').value
        
        if test_lat != 0.0 and test_lon != 0.0:
            self.test_mode = True
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'TEST MODE: Converting GPS ({test_lat}, {test_lon})')
            self.get_logger().info('=' * 60)
            self.create_timer(2.0, lambda: self.test_conversion(test_lat, test_lon))
        else:
            self.get_logger().info('=' * 60)
            self.get_logger().info('CONTINUOUS MODE: Monitoring current GPS position')
            self.get_logger().info('Will convert and display your current GPS in real-time')
            self.get_logger().info('=' * 60)
            self.get_logger().info('')
            self.get_logger().info('To test a specific coordinate:')
            self.get_logger().info('  ros2 run sensors gps_test --ros-args -p lat:=11.065 -p lon:=77.092')
        
    def gps_callback(self, msg: NavSatFix):
        """Continuously convert current GPS position"""
        if self.test_mode:
            return
            
        self.current_gps = msg
        
        # Convert current position
        if msg.latitude != 0.0 and msg.longitude != 0.0:
            self.convert_and_display(msg.latitude, msg.longitude, is_current=True)
    
    def test_conversion(self, lat: float, lon: float):
        """Test a specific GPS coordinate once"""
        self.convert_and_display(lat, lon, is_current=False)
        
    def convert_and_display(self, lat: float, lon: float, is_current: bool):
        """Convert GPS to map coordinates and display"""
        
        # Wait for service
        if not self.fromLL_client.wait_for_service(timeout_sec=1.0):
            if not is_current:
                self.get_logger().error('❌ /fromLL service not available!')
                self.get_logger().error('   Start: ros2 launch sensors navsat_transform.launch.py')
            return
        
        # Create request
        request = FromLL.Request()
        request.ll_point.latitude = lat
        request.ll_point.longitude = lon
        request.ll_point.altitude = 0.0
        
        # Call service
        future = self.fromLL_client.call_async(request)
        future.add_done_callback(
            lambda f: self.handle_response(f, lat, lon, is_current)
        )
    
    def handle_response(self, future, lat, lon, is_current):
        """Handle conversion response"""
        try:
            response = future.result()
            x = response.map_point.x
            y = response.map_point.y
            z = response.map_point.z
            
            # Calculate distance from origin
            dist = math.sqrt(x**2 + y**2)
            
            # Print results
            if is_current:
                self.get_logger().info('━' * 60)
                self.get_logger().info('📍 CURRENT POSITION:')
            else:
                self.get_logger().info('')
                self.get_logger().info('🎯 TEST COORDINATE CONVERSION:')
                self.get_logger().info('━' * 60)
            
            self.get_logger().info(f'  GPS Input:')
            self.get_logger().info(f'    Latitude:  {lat:.7f}°')
            self.get_logger().info(f'    Longitude: {lon:.7f}°')
            self.get_logger().info('')
            self.get_logger().info(f'  Map Output:')
            self.get_logger().info(f'    X: {x:8.2f} m')
            self.get_logger().info(f'    Y: {y:8.2f} m')
            self.get_logger().info(f'    Z: {z:8.2f} m')
            self.get_logger().info('')
            self.get_logger().info(f'  Distance from datum: {dist:.2f} m')
            self.get_logger().info('━' * 60)
            
            # Publish for visualization
            self.publish_visualization(x, y, z, lat, lon, is_current)
            
        except Exception as e:
            self.get_logger().error(f'❌ Conversion failed: {e}')
    
    def publish_visualization(self, x, y, z, lat, lon, is_current):
        """Publish markers for RViz visualization"""
        
        # Publish point
        point_msg = PointStamped()
        point_msg.header.frame_id = 'map'
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = z
        self.point_pub.publish(point_msg)
        
        # Publish marker
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'gps_test'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        if is_current:
            # Blue for current position
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
        else:
            # Green for test position
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        
        marker.color.a = 1.0
        marker.lifetime.sec = 0  # Permanent
        
        self.marker_pub.publish(marker)
        
        # Add text label
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = 'gps_test_label'
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = z + 0.5
        text_marker.pose.orientation.w = 1.0
        
        text_marker.text = f"({lat:.5f}, {lon:.5f})\n({x:.1f}m, {y:.1f}m)"
        text_marker.scale.z = 0.3
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        self.marker_pub.publish(text_marker)


def main(args=None):
    rclpy.init(args=args)
    node = GPSTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down GPS Test')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
