import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker, MarkerArray
from scipy.ndimage import binary_erosion
from enum import Enum
import numpy as np
import math

class State(Enum):
    IDLE = 1
    EXPLORING = 2
    MOVING = 3

class Exploration(Node):
    """
    A ROS2 node for autonomous exploration using a frontier (border) approach.
    
    Instead of evaluating every free cell based on information gain, this version first
    finds unvisited free cells that form a frontier (i.e. cells adjacent to visited cells).
    Then, it selects from those frontier cells a candidate goal. In particular, it prefers
    candidates that lie within the sensor's forward field-of-view and chooses the farthest one,
    so as to “push” the exploration boundary outward.
    """
    def __init__(self):
        super().__init__('exploration')

        # Subscribers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, 'map', self.map_update, 10)
        self.goal_status_subscriber = self.create_subscription(
            GoalStatusArray, '/follow_path/_action/status', self.goal_status, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.update_position, 10)

        # Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'candidate_markers', 10)

        # Variables
        self.map_array = None         # Full-resolution occupancy grid (reshaped from msg.data)
        self.map_info = None          # Map meta-information
        self.downsampled_map = None   # Binary free-space map (downsampled & eroded)
        self.visited_grid = None      # Same shape as downsampled_map; visited cells marked as 1
        self.state = State.IDLE
        self.goal_reached = True
        self.robot_x_pose = 0.0
        self.robot_y_pose = 0.0

        # Parameters:
        # The border_range is not used for candidate computation here (the frontier is defined by unvisited neighbor cells),
        # but you can think of it as the approximate distance from the robot that we wish to “push” the frontier.
        self.border_range = 2.0  
        # Sensor FOV (we limit candidate selection to the front arc)
        self.SENSOR_FOV = math.radians(100)
        
        # Timer for periodic updates (every 5 seconds)
        self.timer = self.create_timer(5.0, self.timer_callback)

        # Downsample factor: e.g., factor=2 means each cell in the downsampled map covers 2x2 original cells.
        self.downsample_factor = 2

        # If no goal has been set, default the robot's forward heading to 0 (facing +x).
        self.robot_orientation = 0.0

    def map_update(self, msg):
        """Updates the map, creates a binary free-space map, downsamples and erodes it."""
        self.map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

        # Create a binary free-space map:
        # OccupancyGrid: 0 => free, -1 => unknown, 100 => occupied.
        binary_map = np.zeros_like(self.map_array, dtype=bool)
        binary_map[self.map_array == 0] = True

        # Downsample for efficiency.
        self.downsampled_map = self.downsample_map(binary_map, self.downsample_factor)

        # Erode to ensure candidates lie well inside free space.
        if self.downsampled_map is not None:
            struct_elem = np.ones((3, 3), dtype=bool)
            self.downsampled_map = binary_erosion(self.downsampled_map, structure=struct_elem)
        self.downsampled_map = self.downsampled_map.astype(int)

        # Initialize the visited grid (same shape as downsampled_map).
        if self.map_info is not None:
            self.visited_grid = np.zeros_like(self.downsampled_map)

    def downsample_map(self, map_array, factor):
        """Downsamples the given 2D boolean map by the specified factor."""
        height, width = map_array.shape
        new_height, new_width = height // factor, width // factor
        downsampled = np.zeros((new_height, new_width), dtype=map_array.dtype)
        for y in range(new_height):
            for x in range(new_width):
                cell_mean = np.mean(map_array[y*factor:(y+1)*factor, x*factor:(x+1)*factor])
                downsampled[y, x] = 1 if cell_mean > 0.5 else 0
        return downsampled

    def timer_callback(self):
        """Periodically updates the state and selects a new goal if needed."""
        if self.state == State.IDLE:
            self.state = State.EXPLORING

        elif self.state == State.EXPLORING:
            if self.goal_reached:
                frontier = self.find_frontier_candidates()
                if frontier:
                    candidate = self.select_frontier_candidate(frontier)
                    if candidate:
                        self.publish_goal(candidate)
                        self.publish_candidate_markers(frontier)
                    else:
                        self.get_logger().info("No valid frontier candidate selected.")
                else:
                    self.get_logger().info("No frontier candidates detected. Exploration may be complete.")
            else:
                self.get_logger().info("Waiting for current goal to be reached.")

        elif self.state == State.MOVING:
            if self.goal_reached:
                self.state = State.EXPLORING

    def goal_status(self, msg):
        """Handles updates from the goal status topic."""
        if msg.status_list:
            current_status = msg.status_list[-1].status
            if current_status == 2:  # ACTIVE
                self.goal_reached = False
            elif current_status == 4:  # SUCCEEDED
                self.goal_reached = True
                self.state = State.EXPLORING

    def update_position(self, msg):
        """Updates the robot's current position and marks nearby cells as visited."""
        self.robot_x_pose = msg.pose.pose.position.x
        self.robot_y_pose = msg.pose.pose.position.y
        self.update_visited_grid(self.robot_x_pose, self.robot_y_pose)
        # Mark a circular area around the robot as visited.
        self.mark_area_visited(sensor_range=1)

    def update_visited_grid(self, x, y):
        """Marks the cell corresponding to the robot's current position as visited in the downsampled visited grid."""
        if self.visited_grid is None or self.map_info is None:
            return
        grid_resolution = self.map_info.resolution * self.downsample_factor
        grid_x = int((x - self.map_info.origin.position.x) / grid_resolution)
        grid_y = int((y - self.map_info.origin.position.y) / grid_resolution)
        if 0 <= grid_x < self.visited_grid.shape[1] and 0 <= grid_y < self.visited_grid.shape[0]:
            self.visited_grid[grid_y, grid_x] = 1

    def mark_area_visited(self, sensor_range=1):
        """
        Marks cells within sensor_range (in meters) of the robot's current position as visited
        in the downsampled visited grid.
        """
        if self.visited_grid is None or self.map_info is None:
            return
        grid_resolution = self.map_info.resolution * self.downsample_factor
        grid_x = int((self.robot_x_pose - self.map_info.origin.position.x) / grid_resolution)
        grid_y = int((self.robot_y_pose - self.map_info.origin.position.y) / grid_resolution)
        grid_range = int(np.ceil(sensor_range / grid_resolution))
        for i in range(-grid_range, grid_range + 1):
            for j in range(-grid_range, grid_range + 1):
                cell_x = grid_x + i
                cell_y = grid_y + j
                if 0 <= cell_x < self.visited_grid.shape[1] and 0 <= cell_y < self.visited_grid.shape[0]:
                    if np.hypot(i * grid_resolution, j * grid_resolution) <= sensor_range:
                        self.visited_grid[cell_y, cell_x] = 1

    # === Frontier Candidate Detection and Selection ===
    def find_frontier_candidates(self):
        """
        Scans the downsampled visited grid and free-space map to find frontier cells.
        A cell (in grid indices) is considered a frontier if it is free (downsampled_map==1),
        not yet visited, and has at least one visited neighbor.
        Returns a list of (x, y) grid indices.
        """
        candidates = []
        if self.visited_grid is None or self.downsampled_map is None:
            return candidates
        height, width = self.visited_grid.shape
        for y in range(height):
            for x in range(width):
                if self.downsampled_map[y, x] == 1 and self.visited_grid[y, x] == 0:
                    # Check 8-connected neighbors for at least one visited cell.
                    neighbor_found = False
                    for j in range(max(0, y-1), min(height, y+2)):
                        for i in range(max(0, x-1), min(width, x+2)):
                            if i == x and j == y:
                                continue
                            if self.visited_grid[j, i] == 1:
                                neighbor_found = True
                                break
                        if neighbor_found:
                            break
                    if neighbor_found:
                        candidates.append((x, y))
        self.get_logger().info(f"Detected {len(candidates)} frontier candidates.")
        return candidates

    def select_frontier_candidate(self, frontier):
        """
        Converts frontier candidates (grid indices) to world coordinates and selects one.
        Among candidates that lie within the robot's sensor forward arc
        (i.e. within ±SENSOR_FOV/2 of the current heading), choose the farthest candidate.
        If none fall in the forward cone, choose the farthest candidate overall.
        Returns world coordinates (x, y) of the selected candidate.
        """
        if not frontier:
            return None
        grid_resolution = self.map_info.resolution * self.downsample_factor
        candidate_worlds = []
        for (x, y) in frontier:
            world_x = x * grid_resolution + self.map_info.origin.position.x + grid_resolution/2.0
            world_y = y * grid_resolution + self.map_info.origin.position.y + grid_resolution/2.0
            candidate_worlds.append((world_x, world_y))
        
        # Filter candidates within the sensor's forward field-of-view.
        candidates_in_fov = []
        for candidate in candidate_worlds:
            dx = candidate[0] - self.robot_x_pose
            dy = candidate[1] - self.robot_y_pose
            candidate_angle = math.atan2(dy, dx)
            dtheta = abs(math.atan2(math.sin(candidate_angle - self.robot_orientation),
                                    math.cos(candidate_angle - self.robot_orientation)))
            if dtheta <= self.SENSOR_FOV/2:
                candidates_in_fov.append(candidate)
        
        # Define a helper to compute Euclidean distance.
        def dist(candidate):
            return math.hypot(candidate[0] - self.robot_x_pose, candidate[1] - self.robot_y_pose)
        
        if candidates_in_fov:
            chosen = max(candidates_in_fov, key=dist)
        else:
            chosen = max(candidate_worlds, key=dist)
        
        if chosen:
            self.get_logger().info(f"Selected frontier candidate goal: {chosen} (distance {dist(chosen):.2f})")
        return chosen

    def publish_goal(self, goal):
        """Publishes a new goal to the goal topic."""
        if goal:
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position = Point(x=goal[0], y=goal[1], z=0.0)
            self.goal_publisher.publish(goal_msg)
            self.state = State.MOVING
            self.get_logger().info(f"Published new goal: {goal}")

    def publish_candidate_markers(self, candidates):
        """Publishes markers for visualization of frontier candidate cells."""
        marker_array = MarkerArray()
        marker_id = 0
        grid_resolution = self.map_info.resolution * self.downsample_factor
        for (x, y) in candidates:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            # Convert grid indices to world coordinates.
            marker.pose.position.x = x * grid_resolution + self.map_info.origin.position.x + grid_resolution/2.0
            marker.pose.position.y = y * grid_resolution + self.map_info.origin.position.y + grid_resolution/2.0
            marker.pose.position.z = 0.0
            marker.scale.x = grid_resolution * 0.8
            marker.scale.y = grid_resolution * 0.8
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)
            marker_id += 1
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    exploration_node = Exploration()
    rclpy.spin(exploration_node)
    exploration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

