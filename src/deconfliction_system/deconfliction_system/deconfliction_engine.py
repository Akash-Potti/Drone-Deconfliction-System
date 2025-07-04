import rclpy
from rclpy.node import Node
from deconfliction_msgs.msg import DroneTrajectory, Waypoint
from deconfliction_msgs.srv import DeconflictCheck
import math
import json

class DeconflictionEngine(Node):
    """
    A ROS2 Node to detect conflicts between drone trajectories in space-time.
    It listens to a primary mission and other simulated drone paths,
    checks for 4D proximity conflicts, and responds to service queries.
    """

    def __init__(self):
        super().__init__('deconfliction_engine')

        # === Configuration ===
        self.SAFETY_RADIUS = 3.0  # meters
        self.TIME_RESOLUTION = 1.0  # seconds

        # === State ===
        self.primary_mission = None
        self.simulated_drones = []
        self.conflicts = []

        # === Communication Interfaces ===
        self.create_subscription(DroneTrajectory, 'primary_mission', self.primary_callback, 10)
        self.create_subscription(DroneTrajectory, 'simulated_drones', self.sim_callback, 10)
        self.srv = self.create_service(DeconflictCheck, 'check_deconfliction', self.handle_query)

    # === Callback for primary mission ===
    def primary_callback(self, msg):
        self.primary_mission = msg
        self.check_for_conflicts()

    # === Callback for simulated drones ===
    def sim_callback(self, msg):
        if msg.drone_id not in [d.drone_id for d in self.simulated_drones]:
            self.simulated_drones.append(msg)
            self.check_for_conflicts()

    # === Conflict detection ===
    def check_for_conflicts(self):
        if not self.primary_mission or not self.simulated_drones:
            return

        self.get_logger().info("Checking for conflicts...")
        self.conflicts = []  # Clear previous conflicts
        primary_points = self.interpolate_trajectory(self.primary_mission)

        for drone in self.simulated_drones:
            if drone.drone_id == self.primary_mission.drone_id:
                continue

            sim_points = self.interpolate_trajectory(drone)

            for t1, p_pos in primary_points.items():
                for t2, s_pos in sim_points.items():
                    if abs(t1 - t2) <= self.TIME_RESOLUTION / 2:
                        dist = self.distance(p_pos, s_pos)
                        if dist < self.SAFETY_RADIUS:
                            self.get_logger().warn(
                                f"Conflict with {drone.drone_id} at time {t1:.1f}s, distance = {dist:.2f}m"
                            )

                            # Save conflict info
                            conflict_point = {
                                'timestamp': round(t1, 2),
                                'x': round((p_pos[0] + s_pos[0]) / 2, 2),
                                'y': round((p_pos[1] + s_pos[1]) / 2, 2),
                                'z': round((p_pos[2] + s_pos[2]) / 2, 2),
                                'drones': [self.primary_mission.drone_id, drone.drone_id]
                            }
                            self.conflicts.append(conflict_point)

        if not self.conflicts:
            self.get_logger().info("No conflicts detected.")

        self.export_to_file()

    # === Linear interpolation between trajectory waypoints ===
    def interpolate_trajectory(self, drone_msg):
        waypoints = drone_msg.waypoints
        if len(waypoints) < 2:
            return {}

        path = {}
        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            t1, t2 = wp1.timestamp, wp2.timestamp
            steps = int((t2 - t1) / self.TIME_RESOLUTION)

            for s in range(steps + 1):
                t = t1 + s * self.TIME_RESOLUTION
                ratio = (t - t1) / (t2 - t1)
                x = wp1.x + (wp2.x - wp1.x) * ratio
                y = wp1.y + (wp2.y - wp1.y) * ratio
                z = wp1.z + (wp2.z - wp1.z) * ratio
                path[t] = (x, y, z)

        return path

    # === Euclidean 3D distance ===
    def distance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    # === Service callback to externally check a new mission ===
    def handle_query(self, request, response):
        primary_points = self.interpolate_trajectory(request.request_mission)
        result = "clear"
        conflict_drones = []
        conflict_positions = []

        for drone in self.simulated_drones:
            sim_points = self.interpolate_trajectory(drone)

            for t1, p_pos in primary_points.items():
                for t2, s_pos in sim_points.items():
                    if abs(t1 - t2) <= self.TIME_RESOLUTION / 2:
                        dist = self.distance(p_pos, s_pos)
                        if dist < self.SAFETY_RADIUS:
                            result = "conflict detected"
                            conflict_drones.append(drone.drone_id)
                            conflict_positions.append(
                                Waypoint(x=p_pos[0], y=p_pos[1], z=p_pos[2], timestamp=t1)
                            )

        response.status = result
        response.conflicting_drones = list(set(conflict_drones))
        response.conflict_points = conflict_positions
        return response

    # === Export all trajectories + conflicts to a JSON file for visualization ===
    def export_to_file(self):
        drones = [self.primary_mission] + self.simulated_drones
        data = {}

        for drone in drones:
            data[drone.drone_id] = [
                {
                    'x': wp.x,
                    'y': wp.y,
                    'z': wp.z,
                    'timestamp': wp.timestamp
                } for wp in drone.waypoints
            ]

        data['conflicts'] = self.conflicts

        with open("trajectory_data.json", "w") as f:
            json.dump(data, f, indent=2)

        self.get_logger().info("Exported data with conflicts to trajectory_data.json")

# === Main entry point ===
def main(args=None):
    rclpy.init(args=args)
    node = DeconflictionEngine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
