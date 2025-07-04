import rclpy
from rclpy.node import Node
from deconfliction_msgs.msg import DroneTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import time
import random


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        self.publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.create_subscription(DroneTrajectory, 'primary_mission', self.handle_trajectory, 10)
        self.create_subscription(DroneTrajectory, 'simulated_drones', self.handle_trajectory, 10)

        self.drone_colors = {}
        self.drone_paths = {}  # drone_id: list of (timestamp, (x,y,z))
        self.current_markers = {}  # drone_id: current moving marker

        self.timer = self.create_timer(1.0, self.update_positions)
        start_time = time.time()

    def handle_trajectory(self, msg: DroneTrajectory):
        # Store interpolated positions
        if msg.drone_id not in self.drone_colors:
            self.drone_colors[msg.drone_id] = self.random_color()

        self.drone_paths[msg.drone_id] = self.interpolate(msg)
        self.get_logger().info(f"Received path for {msg.drone_id} with {len(self.drone_paths[msg.drone_id])} steps.")

    def update_positions(self):
        now = time.time()
        marker_array = MarkerArray()

        for drone_id, path in self.drone_paths.items():
            for t, pos in path:
                if abs(t - now) < 0.5:  # Find the point closest to now
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.ns = f"{drone_id}_live"
                    marker.id = hash(drone_id) % 10000
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.4
                    marker.color = self.drone_colors[drone_id]
                    marker.pose.position.x = pos[0]
                    marker.pose.position.y = pos[1]
                    marker.pose.position.z = pos[2]
                    marker.lifetime.sec = 1
                    marker_array.markers.append(marker)

        self.publisher_.publish(marker_array)

    def interpolate(self, msg):
        """Interpolate waypoints at 1-second intervals."""
        result = []
        wps = msg.waypoints
        if len(wps) < 2:
            return result
        for i in range(len(wps) - 1):
            wp1, wp2 = wps[i], wps[i+1]
            t1, t2 = wp1.timestamp, wp2.timestamp
            steps = int(t2 - t1)
            for s in range(steps + 1):
                t = t1 + s
                ratio = (t - t1) / (t2 - t1) if t2 != t1 else 0
                x = wp1.x + (wp2.x - wp1.x) * ratio
                y = wp1.y + (wp2.y - wp1.y) * ratio
                z = wp1.z + (wp2.z - wp1.z) * ratio
                result.append((t, (x, y, z)))
        return result

    def random_color(self):
        return ColorRGBA(r=random.random(), g=random.random(), b=random.random(), a=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
