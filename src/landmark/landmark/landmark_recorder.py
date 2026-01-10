import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
import yaml
import os
import sys
import select

LANDMARK_FILE = os.path.expanduser(
    "~/IRC26/src/landmark/config/landmarks.yaml"
)

class LandmarkRecorder(Node):
    def __init__(self):
        super().__init__('landmark_recorder')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(
            Marker, '/landmark_markers', 10
        )

        self.get_logger().info("Press 'l' to save landmark")

    def load_landmarks(self):
        if not os.path.exists(LANDMARK_FILE):
            return []

        with open(LANDMARK_FILE, 'r') as f:
            data = yaml.safe_load(f)

        if data is None:
            return []

        return data.get('landmarks', [])

    def save_landmarks(self, landmarks):
        os.makedirs(os.path.dirname(LANDMARK_FILE), exist_ok=True)
        with open(LANDMARK_FILE, 'w') as f:
            yaml.safe_dump({'landmarks': landmarks}, f)

    def publish_marker(self, lm):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "landmarks"
        marker.id = lm['id']
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = lm['x']
        marker.pose.position.y = lm['y']
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def record_landmark(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
        except Exception:
            self.get_logger().error("TF lookup failed")
            return

        x = t.transform.translation.x
        y = t.transform.translation.y

        landmarks = self.load_landmarks()
        lm = {'id': len(landmarks), 'x': x, 'y': y}

        landmarks.append(lm)
        self.save_landmarks(landmarks)
        self.publish_marker(lm)

        self.get_logger().info(
            f"Saved landmark {lm['id']} at x={x:.2f}, y={y:.2f}"
        )

    def check_keyboard(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            if sys.stdin.read(1) == 'l':
                self.record_landmark()

def main():
    rclpy.init()
    node = LandmarkRecorder()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        node.check_keyboard()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

