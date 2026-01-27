#!/usr/bin/env python3
"""
GPS to Nav2 Goal Translator

Converts GPS coordinates to map frame by computing offset from initial GPS
position (captured when odom is at origin). Then sends Nav2 navigation goal.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLL

# Earth radius in meters (WGS84 approximation)
EARTH_RADIUS = 6371000.0


class GPSNavTranslator(Node):
    def __init__(self):
        super().__init__('gps_nav_translator')

        # Initial GPS reference (captured when odom ~= 0,0)
        self.initial_gps = None
        self.initial_gps_set = False

        # Current odom pose
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Navsat_transform service client (global GPS -> map conversion)
        self.from_ll_client = self.create_client(FromLL, 'fromLL')
        self.use_from_ll = False

        self.get_logger().info('GPS Nav Translator initialized. Waiting for initial GPS fix...')

    def odom_callback(self, msg: Odometry):
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y

    def gps_callback(self, msg: NavSatFix):
        # Capture initial GPS when odom is near origin
        if not self.initial_gps_set:
            if abs(self.current_odom_x) < 0.5 and abs(self.current_odom_y) < 0.5:
                self.initial_gps = (msg.latitude, msg.longitude)
                self.initial_gps_set = True
                self.get_logger().info(
                    f'Initial GPS reference set: lat={msg.latitude:.7f}, lon={msg.longitude:.7f}')

    def gps_to_local(self, dest_lat: float, dest_lon: float) -> tuple:
        """
        Convert destination GPS to local map frame coordinates.
        Uses simple equirectangular projection (good for small distances).
        
        Returns (x, y) in meters relative to initial GPS position.
        """
        if not self.initial_gps_set:
            self.get_logger().error('Initial GPS not set yet!')
            return None

        init_lat, init_lon = self.initial_gps

        # Convert to radians
        lat1 = math.radians(init_lat)
        lat2 = math.radians(dest_lat)
        lon1 = math.radians(init_lon)
        lon2 = math.radians(dest_lon)

        # Equirectangular approximation
        # x = longitude difference * cos(avg latitude) * earth radius
        # y = latitude difference * earth radius
        avg_lat = (lat1 + lat2) / 2.0

        delta_lon = lon2 - lon1
        delta_lat = lat2 - lat1

        x = delta_lon * math.cos(avg_lat) * EARTH_RADIUS
        y = delta_lat * EARTH_RADIUS

        return (x, y)

    def gps_to_map_from_ll(self, dest_lat: float, dest_lon: float) -> tuple:
        if not self.from_ll_client.service_is_ready():
            self.get_logger().warn('fromLL service not available; using local offset conversion.')
            return None

        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=dest_lat, longitude=dest_lon, altitude=0.0)
        future = self.from_ll_client.call_async(req)

        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > 2.0:
                self.get_logger().warn('fromLL service timed out; using local offset conversion.')
                return None
            time.sleep(0.05)

        if future.result() is None:
            self.get_logger().warn('fromLL service failed; using local offset conversion.')
            return None

        map_point = future.result().map_point
        return (map_point.x, map_point.y)

    def send_nav_goal(self, dest_lat: float, dest_lon: float, yaw: float = 0.0):
        """
        Convert GPS destination to local frame and send Nav2 goal.
        
        Args:
            dest_lat: Destination latitude in degrees
            dest_lon: Destination longitude in degrees
            yaw: Desired final orientation in radians (default 0)
        """
        if self.use_from_ll:
            local_coords = self.gps_to_map_from_ll(dest_lat, dest_lon)
            if local_coords is None:
                local_coords = self.gps_to_local(dest_lat, dest_lon)
        else:
            local_coords = self.gps_to_local(dest_lat, dest_lon)
        if local_coords is None:
            return False

        x, y = local_coords
        self.get_logger().info(f'GPS ({dest_lat:.7f}, {dest_lon:.7f}) -> Local ({x:.2f}, {y:.2f})')

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return False

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion (only rotation around Z)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending Nav2 goal: x={x:.2f}, y={y:.2f}')

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation complete!')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Optionally log progress
        pass


def main(args=None):
    rclpy.init(args=args)
    node = GPSNavTranslator()

    import threading
    import time

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("Enter initial GPS reference (leave blank to use navsat_transform /fromLL if available)")
    try:
        init_lat_str = input("Initial Latitude: ").strip()
        init_lon_str = input("Initial Longitude: ").strip()

        if init_lat_str and init_lon_str:
            node.initial_gps = (float(init_lat_str), float(init_lon_str))
            node.initial_gps_set = True
            print(
                f"Initial GPS set (manual): {node.initial_gps[0]:.7f}, {node.initial_gps[1]:.7f}")
            node.use_from_ll = False
        else:
            if node.from_ll_client.wait_for_service(timeout_sec=2.0):
                node.use_from_ll = True
                print("Using navsat_transform /fromLL for global GPS conversion.")
            else:
                print("fromLL not available. Waiting for initial GPS fix (odom must be near origin)...")
                while rclpy.ok() and not node.initial_gps_set:
                    time.sleep(0.5)
    except ValueError:
        print("Invalid initial GPS input.")
        if node.from_ll_client.wait_for_service(timeout_sec=2.0):
            node.use_from_ll = True
            print("Using navsat_transform /fromLL for global GPS conversion.")
        else:
            print("fromLL not available. Waiting for GPS fix instead...")
            while rclpy.ok() and not node.initial_gps_set:
                time.sleep(0.5)

    if not rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
        return

    if node.initial_gps_set:
        print(f"Initial GPS set: {node.initial_gps[0]:.7f}, {node.initial_gps[1]:.7f}")

    try:
        while rclpy.ok():
            print("\n--- Enter destination GPS ---")
            try:
                lat_str = input("Latitude: ").strip()
                if not lat_str:
                    continue
                lon_str = input("Longitude: ").strip()
                if not lon_str:
                    continue

                dest_lat = float(lat_str)
                dest_lon = float(lon_str)

                print(f"Navigating to: {dest_lat:.7f}, {dest_lon:.7f}")
                node.send_nav_goal(dest_lat, dest_lon)

            except ValueError:
                print("Invalid input. Enter numeric latitude and longitude.")
            except EOFError:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
