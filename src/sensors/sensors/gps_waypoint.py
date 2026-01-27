#!/usr/bin/env python3
"""
GPS Waypoint Navigation Node

This node accepts GPS coordinates (latitude, longitude) and navigates the rover
to that location using Nav2.

Usage:
    ros2 run sensors gps_waypoint --ros-args -p target_lat:=<lat> -p target_lon:=<lon>
    
Or publish to the /gps_goal topic:
    ros2 topic pub /gps_goal sensor_msgs/msg/NavSatFix "{latitude: <lat>, longitude: <lon>}"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL
import math


class GPSWaypointNavigator(Node):
    def __init__(self):
        super().__init__('gps_waypoint_navigator')
        
        # Declare parameters for target GPS coordinates
        self.declare_parameter('target_lat', 0.0)
        self.declare_parameter('target_lon', 0.0)
        
        # Service client to convert GPS to local coordinates
        self.fromLL_client = self.create_client(FromLL, '/fromLL')
        
        # Action client for Nav2 navigation
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Subscriber for GPS goal topic (alternative to parameters)
        self.gps_goal_sub = self.create_subscription(
            NavSatFix,
            '/gps_goal',
            self.gps_goal_callback,
            10
        )
        
        # Current GPS subscriber (for reference/logging)
        self.current_gps_sub = self.create_subscription(
            NavSatFix,
            '/gps',
            self.current_gps_callback,
            10
        )
        
        self.current_gps = None
        self.goal_sent = False
        
        self.get_logger().info('GPS Waypoint Navigator initialized')
        self.get_logger().info('Waiting for /fromLL service (from navsat_transform)...')
        
        # Check for initial parameters
        target_lat = self.get_parameter('target_lat').value
        target_lon = self.get_parameter('target_lon').value
        
        if target_lat != 0.0 and target_lon != 0.0:
            self.get_logger().info(f'Target GPS from parameters: ({target_lat}, {target_lon})')
            # Wait for service and then navigate
            self.create_timer(1.0, lambda: self.navigate_to_gps(target_lat, target_lon))
        else:
            self.get_logger().info('No target GPS provided via parameters.')
            self.get_logger().info('Publish target to /gps_goal topic or use:')
            self.get_logger().info('  ros2 run sensors gps_waypoint --ros-args -p target_lat:=<lat> -p target_lon:=<lon>')

    def current_gps_callback(self, msg: NavSatFix):
        """Store current GPS for reference"""
        self.current_gps = msg
        
    def gps_goal_callback(self, msg: NavSatFix):
        """Handle GPS goal from topic"""
        self.get_logger().info(f'Received GPS goal: lat={msg.latitude}, lon={msg.longitude}')
        self.navigate_to_gps(msg.latitude, msg.longitude)
        
    def navigate_to_gps(self, latitude: float, longitude: float):
        """Convert GPS to local coordinates and send navigation goal"""
        if self.goal_sent:
            self.get_logger().warn('Goal already sent, ignoring...')
            return
            
        # Wait for the FromLL service
        if not self.fromLL_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/fromLL service not available!')
            self.get_logger().error('Make sure navsat_transform is running:')
            self.get_logger().error('  ros2 launch sensors navsat_transform.launch.py')
            return
            
        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 /navigate_to_pose action not available!')
            self.get_logger().error('Make sure Nav2 is running.')
            return
        
        # Create FromLL request
        request = FromLL.Request()
        request.ll_point.latitude = latitude
        request.ll_point.longitude = longitude
        request.ll_point.altitude = 0.0
        
        self.get_logger().info(f'Converting GPS ({latitude}, {longitude}) to map coordinates...')
        
        # Call the service
        future = self.fromLL_client.call_async(request)
        future.add_done_callback(
            lambda f: self.fromLL_response_callback(f, latitude, longitude)
        )
        
    def fromLL_response_callback(self, future, latitude, longitude):
        """Handle response from FromLL service"""
        try:
            response = future.result()
            map_point = response.map_point
            
            self.get_logger().info(f'GPS ({latitude}, {longitude}) -> Map (x={map_point.x:.2f}, y={map_point.y:.2f})')
            
            # Log distance from current position if available
            if self.current_gps:
                dist = self.haversine_distance(
                    self.current_gps.latitude, self.current_gps.longitude,
                    latitude, longitude
                )
                self.get_logger().info(f'Approximate distance to target: {dist:.1f} meters')
            
            # Send navigation goal
            self.send_nav_goal(map_point.x, map_point.y)
            
        except Exception as e:
            self.get_logger().error(f'FromLL service call failed: {e}')
            
    def send_nav_goal(self, x: float, y: float):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # No rotation preference
        
        self.get_logger().info(f'Sending navigation goal to: x={x:.2f}, y={y:.2f}')
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
        self.goal_sent = True
        
    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal was rejected!')
            self.goal_sent = False
            return
            
        self.get_logger().info('Navigation goal accepted! Rover is moving...')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
        
    def nav_feedback_callback(self, feedback_msg):
        """Handle Nav2 feedback"""
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {remaining:.2f} m')
        
    def nav_result_callback(self, future):
        """Handle Nav2 result"""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('✓ Navigation succeeded! Rover arrived at GPS waypoint.')
        else:
            self.get_logger().warn(f'Navigation finished with status: {result.status}')
        self.goal_sent = False
        
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS coordinates in meters"""
        R = 6371000  # Earth's radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_phi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down GPS Waypoint Navigator')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
