import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import yaml
import os

LANDMARK_FILE = os.path.expanduser(
    "~/IRC26/src/landmark/config/landmarks.yaml"
)

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(
            Marker, '/landmark_markers', 10
        )
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        if not os.path.exists(LANDMARK_FILE):
            return

        with open(LANDMARK_FILE, 'r') as f:
            data = yaml.safe_load(f) or {}

        for lm in data.get('landmarks', []):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = "landmarks"
            marker.id = lm['id']
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = lm['x']
            marker.pose.position.y = lm['y']
            marker.pose.orientation.w = 1.0

            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color.g = 1.0
            marker.color.a = 1.0

            self.publisher.publish(marker)

def main():
    rclpy.init()
    rclpy.spin(MarkerPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

