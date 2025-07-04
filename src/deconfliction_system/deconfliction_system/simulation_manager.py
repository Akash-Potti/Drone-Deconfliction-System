import rclpy
from rclpy.node import Node
from deconfliction_msgs.msg import DroneTrajectory, Waypoint
import time

class SimulationManager(Node):
    """
    A ROS2 node that simulates multiple drone trajectories by publishing
    them to the 'simulated_drones' topic periodically.
    
    This is useful for testing deconfliction systems against dynamic or
    pre-defined drone flight paths.
    """

    def __init__(self):
        super().__init__('simulation_manager')

        # === Publisher ===
        self.publisher_ = self.create_publisher(DroneTrajectory, 'simulated_drones', 10)

        # === Timer ===
        # Triggers every 3 seconds to simulate and publish all drones' paths
        self.timer = self.create_timer(3.0, self.publish_simulation)

    def publish_simulation(self):
        """
        Publishes predefined flight paths for multiple drones.
        Includes one intentional conflict path ("sim_drone_conflict").
        """

        # Define the drone IDs and their waypoint sequences (relative times)
        drones = {
            "sim_drone_1": [(5, 0, 2, 0), (5, 10, 2, 10), (5, 20, 2, 20)],
            "sim_drone_2": [(15, 15, 2, 0), (10, 10, 2, 10), (5, 5, 2, 20)],
            "sim_drone_conflict": [(10, 5, 2, 0), (10, 15, 2, 10), (10, 25, 2, 20)],
        }

        now = time.time()  # Current time as the base for timestamps

        # Publish each drone's path
        for drone_id, path in drones.items():
            msg = DroneTrajectory()
            msg.drone_id = drone_id

            for x, y, z, t in path:
                wp = Waypoint(x=float(x), y=float(y), z=float(z), timestamp=now + t)
                msg.waypoints.append(wp)

            self.publisher_.publish(msg)
            self.get_logger().info(f"Published path for: {drone_id}")

def main(args=None):
    rclpy.init(args=args)
    node = SimulationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
