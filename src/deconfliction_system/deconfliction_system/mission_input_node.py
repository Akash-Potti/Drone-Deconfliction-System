import rclpy
from rclpy.node import Node
from deconfliction_msgs.msg import DroneTrajectory, Waypoint
import time

class MissionInputNode(Node):
    """
    A ROS2 Node that publishes a one-time primary mission trajectory
    to the 'primary_mission' topic for deconfliction analysis.
    """

    def __init__(self):
        super().__init__('mission_input_node')

        # === Publisher ===
        self.publisher_ = self.create_publisher(DroneTrajectory, 'primary_mission', 10)

        # === Timer to trigger one-time publish ===
        self.timer = self.create_timer(1.0, self.publish_once)
        self.published = False

    def publish_once(self):
        """
        Publish the primary drone mission once and shut down the node.
        """
        if self.published:
            return

        msg = DroneTrajectory()
        msg.drone_id = "primary_drone"

        # Current wall time for absolute timestamps
        now = time.time()

        # Define 3D trajectory waypoints
        waypoints = [
            (0.0, 0.0, 2.0, now),
            (10.0, 0.0, 10.0, now + 10),
            (10.0, 10.0, 2.0, now + 20)
        ]

        # Create Waypoint messages
        for x, y, z, t in waypoints:
            wp = Waypoint(x=x, y=y, z=z, timestamp=t)
            msg.waypoints.append(wp)

        # Publish trajectory
        self.publisher_.publish(msg)
        self.get_logger().info("Published primary mission (once).")
        self.published = True

        # Shutdown after publishing
        rclpy.shutdown()

# === Entry point ===
def main(args=None):
    rclpy.init(args=args)
    node = MissionInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
