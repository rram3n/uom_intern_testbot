#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import math
import numpy as np
import heapq

# Define the Cell class
class Cell:
    def __init__(self):
      # Parent cell's row index
        self.parent_i = 0
    # Parent cell's column index
        self.parent_j = 0
 # Total cost of the cell (g + h)
        self.f = float('inf')
    # Cost from start to this cell
        self.g = float('inf')
    # Heuristic cost from this cell to destination
        self.h = 0

# Check if a cell is valid (within the grid)
def is_valid(row, col, row_max, col_max):
    return (row >= 0) and (row < row_max) and (col >= 0) and (col < col_max)

# Check if a cell is unblocked
def is_unblocked(grid, row, col):
    return grid[row][col] == 0

# Check if a cell is the destination
def is_destination(row, col, dest):
    return row == dest[1] and col == dest[0]

# Calculate the heuristic value of a cell (Euclidean distance to destination)
def calculate_h_value(row, col, dest):
    return ((row - dest[1]) ** 2 + (col - dest[0]) ** 2) ** 0.5

# Trace the path from source to destination
def trace_path(cell_details, dest):
    print("The Path is ")
    path = []
    row = dest[1]
    col = dest[0]

    # Trace the path from destination to source using parent cells
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((col, row))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row = temp_row
        col = temp_col

    # Add the source cell to the path
    path.append((col, row))
    # Reverse the path to get the path from source to destination
    path.reverse()

    # Print the path
    for i in path:
        print("->", i, end=" ")
    print()
    return path

# Implement the A* search algorithm
def a_star_search(grid, src, dest):
    row_max = len(grid)
    column_max = len(grid[0])
    # Check if the source and destination are valid
    if not is_valid(src[1], src[0], row_max, column_max) or not is_valid(dest[1], dest[0], row_max, column_max):
        print("Source or destination is invalid")
        return

    # Check if the source and destination are unblocked
    if not is_unblocked(grid, src[1], src[0]) or not is_unblocked(grid, dest[1], dest[0]):
        print("Source or the destination is blocked")
        return

    # Check if we are already at the destination
    if is_destination(src[1], src[0], dest):
        print("We are already at the destination")
        return

    # Initialize the closed list (visited cells)
    closed_list = [[False for _ in range(column_max)] for _ in range(row_max)]
    # Initialize the details of each cell
    cell_details = [[Cell() for _ in range(column_max)] for _ in range(row_max)]

    # Initialize the start cell details
    i = src[1]
    j = src[0]
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].h = 0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    # Initialize the open list (cells to be visited) with the start cell
    open_list = []
    heapq.heappush(open_list, (0.0, i, j))

    # Initialize the flag for whether destination is found
    found_dest = False

    # Main loop of A* search algorithm
    while len(open_list) > 0:
        # Pop the cell with the smallest f value from the open list
        p = heapq.heappop(open_list)

        # Mark the cell as visited
        i = p[1]
        j = p[2]
        closed_list[i][j] = True

        # For each direction, check the successors
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0),
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dir in directions:
            new_i = i + dir[1]
            new_j = j + dir[0]

            # If the successor is valid, unblocked, and not visited
            if is_valid(new_i, new_j, row_max, column_max) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                # If the successor is the destination
                if is_destination(new_i, new_j, dest):
                    # Set the parent of the destination cell
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    print("The destination cell is found")
                    # Trace and print the path from source to destination
                    path = trace_path(cell_details, dest)
                    found_dest = True
                    return path
                else:
                    # Calculate the new f, g, and h values
                    g_new = cell_details[i][j].g + 1.0
                    h_new = calculate_h_value(new_i, new_j, dest)
                    f_new = g_new + h_new

                    # If the cell is not in the open list or the new f value is smaller
                    if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                        # Add the cell to the open list
                        heapq.heappush(open_list, (f_new, new_i, new_j))
                        # Update the cell details
                        cell_details[new_i][new_j].f = f_new
                        cell_details[new_i][new_j].g = g_new
                        cell_details[new_i][new_j].h = h_new
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j

    # If the destination is not found after visiting all cells
    if not found_dest:
        print("Failed to find the destination cell")

class PathPlanner(Node):

    def __init__(self):
        super().__init__('a_star_pathfinder')

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.current_pose_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, '/path', 10)  # Path publisher

        self.timer = self.create_timer(0.5, self.go_to_goal)

        self.map_data = None
        self.path = []
        self.grid_resolution = 0.1  # meters per cell
        self.grid_width = 0
        self.grid_height = 0
        self.grid_origin = (0, 0)
        self.clearance_distance = 0.4

        self.current_robot_position = None
        self.goal_position = None
        self.reached_goal = False
        self.need_orientation_correction = False
        self.read_map = False

    def map_callback(self, msg):
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height
        self.grid_resolution = msg.info.resolution
        self.grid_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.data = msg.data
        #self.map_data = np.array(msg.data).reshape((self.grid_height, self.grid_width))
        #print("non inflated \n", self.map_data)

        # converts map data into 2D array
        self.map_data = [[0 for _ in range(self.grid_width)] for _ in range(self.grid_height)]
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                self.map_data[i][j] = self.data[i * self.grid_width + j]

        # 0 = free space
        # 100 = obstacle
        # -1 = unknown space

        self.inflate_obstacles()
        if self.read_map is False:
            self.get_logger().info(f"Map received with dimensions: {self.grid_width}x{self.grid_height}")
            self.read_map = True
            '''
            for row in self.map_data:
                print(row)
            '''

            #print(self.map_data)
            #self.get_logger().info(f"Map: \n {self.map_data}")
            self.get_logger().info(f"Ready to receive goal")
        #print(self.map_data)

    def current_pose_callback(self, msg):
        self.current_robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
        )

    def goal_pose_callback(self, msg):
        self.goal_position = (
            msg.pose.position.x,
            msg.pose.position.y,
            self.quaternion_to_yaw(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            )
        )

        formatted_goal_position = self.format_position(self.goal_position)
        self.get_logger().info(f"New goal position received: {formatted_goal_position}")
        
        formatted_current_position = self.format_position(self.current_robot_position)
        current_x, current_y, current_yaw = self.current_robot_position
        self.get_logger().info(f"Starting position: {formatted_current_position}")

        # define starting position and goal position
        start = (self.current_robot_position[0], self.current_robot_position[1])
        goal = (self.goal_position[0], self.goal_position[1])

        grid_start = self.world_to_grid(start)
        gird_goal = self.world_to_grid(goal)

        self.get_logger().info(f"Calculating Path")
        self.path = a_star_search(self.map_data, grid_start, gird_goal)
        if self.path is None:
            self.get_logger().info(f"Cannot find path")
        else:
            self.get_logger().info(f"Path found")
            self.reached_goal = False  # Reset goal status
            self.need_orientation_correction = False # Reset orientation status
            self.read_map = True

        self.publish_path()  # Publish the path when a new goal is received

    def go_to_goal(self):

        # if the robot positions are not yet defined
        if self.current_robot_position is None or self.goal_position is None or self.path is None:
            return
        
        # if goal has been reached
        if len(self.path) == 0:
            if self.reached_goal is False:
                self.stop_robot()
                self.get_logger().info("Goal has been reached.")
                self.reached_goal = True  # Mark goal as reached
            elif self.reached_goal is True and self.need_orientation_correction is False:
                self.orient_robot()
            return

        # if goal has not been reached yet
        current_pos_world_x, current_pos_world_y, current_yaw = self.current_robot_position

        goal_grid_pos = self.path[0]

        goal_pos_world_x, goal_pos_world_y = self.grid_to_world(goal_grid_pos)

        distance_to_goal = math.sqrt((goal_pos_world_x - current_pos_world_x) ** 2 + (goal_pos_world_y - current_pos_world_y) ** 2)
        angle_to_goal = math.atan2(goal_pos_world_y - current_pos_world_y, goal_pos_world_x - current_pos_world_x)
        angle_diff = self.normalize_angle(angle_to_goal - current_yaw)

        twist_msg = Twist()

        # Adjust orientation if necessary
        if abs(angle_diff) > 0.1:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5 * angle_diff / abs(angle_diff)
        else:
            # Move towards the waypoint
            twist_msg.linear.x = min(0.5, distance_to_goal)
            twist_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist_msg)

        # If close enough to the current waypoint, pop it from the path
        if distance_to_goal < 0.1:
            self.path.pop(0)
            self.get_logger().info(f"Reached waypoint: ({goal_pos_world_x:.3f}, {goal_pos_world_y:.3f})")

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    def orient_robot(self):
        current_x, current_y, current_yaw = self.current_robot_position
        goal_x, goal_y, goal_yaw = self.goal_position

        angle_diff_goal_orient = self.normalize_angle(goal_yaw - current_yaw)
        if abs(angle_diff_goal_orient) > 0.1:
            twist_msg = Twist()
            twist_msg.angular.z = 0.5 * angle_diff_goal_orient / abs(angle_diff_goal_orient)
            self.cmd_vel_publisher.publish(twist_msg)
        else: # oriented correctly
            self.stop_robot()
            self.get_logger().info("Robot correctly oriented.")
            self.need_orientation_correction = True
            
    def publish_path(self):
        if self.path:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'

            for (grid_x, grid_y) in self.path:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = 'map'
                world_x, world_y = self.grid_to_world((grid_x, grid_y))
                pose_stamped.pose.position.x = world_x
                pose_stamped.pose.position.y = world_y
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path_msg.poses.append(pose_stamped)

            self.path_publisher.publish(path_msg)
            self.get_logger().info("Published path with {} points".format(len(path_msg.poses)))

    def inflate_obstacles(self):
        inflated_map = self.map_data.copy()
        #clearance_cells = int(self.clearance_distance / self.grid_resolution)
        clearance_cells = 5
        obstacle_cells = []

        # first convert it to 1 is obstcle and 0 is free space
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if self.map_data[y][x] == -1: # unknown cell
                    self.map_data[y][x] = 1
                if self.map_data[y][x] == 100:  # Obstacle cell
                    self.map_data[y][x] = 1


        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if self.map_data[y][x] == 1: # obstacle cell
                    obstacle_cells.append((x,y))
                    
        for coor in obstacle_cells:
            x = coor[0]
            y = coor[1]
            for dy in range(-clearance_cells, clearance_cells + 1):
                for dx in range(-clearance_cells, clearance_cells + 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                        inflated_map[ny][nx] = 1

        self.map_data = inflated_map

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def format_position(self, position):
        return (
            f"{position[0]:.3f}",
            f"{position[1]:.3f}",
            f"{position[2]:.3f}"
        )

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def world_to_grid(self, world_position):
        grid_x = int((world_position[0] - self.grid_origin[0]) / self.grid_resolution)
        grid_y = int((world_position[1] - self.grid_origin[1]) / self.grid_resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_position):
        world_x = grid_position[0] * self.grid_resolution + self.grid_origin[0]
        world_y = grid_position[1] * self.grid_resolution + self.grid_origin[1]
        return (world_x, world_y)

def main(args=None):
    rclpy.init(args=args)
    pose_tracker = PathPlanner()
    rclpy.spin(pose_tracker)
    pose_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
