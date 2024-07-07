#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import heapq
import math
import numpy as np

class AStarNode:
    def __init__(self, position, cost, priority):
        self.position = position
        self.cost = cost
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority

class AStarPathfinder(Node):
    def __init__(self):
        super().__init__('a_star_pathfinder')
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.map_data = None
        self.goal_position = None
        self.current_position = None
        self.current_orientation = None
        self.path = []
        self.grid_resolution = 0.1  # meters per cell
        self.grid_width = 0
        self.grid_height = 0
        self.grid_origin = (0, 0)

        self.clearance_distance = 0.4  # Clearance around obstacles in meters

    def map_callback(self, msg):
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height
        self.grid_resolution = msg.info.resolution
        self.grid_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_data = np.array(msg.data).reshape((self.grid_height, self.grid_width))
        self.inflate_obstacles()

    def goal_pose_callback(self, msg):
        self.goal_position = (
            int((msg.pose.position.x - self.grid_origin[0]) / self.grid_resolution),
            int((msg.pose.position.y - self.grid_origin[1]) / self.grid_resolution)
        )
        self.get_logger().info(f"Goal position set to: {self.goal_position}")

    def odom_callback(self, msg):
        self.current_position = (
            int((msg.pose.pose.position.x - self.grid_origin[0]) / self.grid_resolution),
            int((msg.pose.pose.position.y - self.grid_origin[1]) / self.grid_resolution)
        )
        self.current_orientation = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

    def control_loop(self):
        if self.goal_position and self.current_position:
            if not self.path:
                self.path = self.a_star(self.current_position, self.goal_position)
                if self.path:
                    self.get_logger().info(f"Path found: {self.path}")
                else:
                    self.get_logger().warn("No path found to the goal.")
            else:
                if self.move_along_path():
                    self.path = []  # Clear path when goal is reached

    def a_star(self, start, goal):
        open_list = []
        heapq.heappush(open_list, AStarNode(start, 0, 0))
        came_from = {}
        cost_so_far = {start: 0}

        while open_list:
            current_node = heapq.heappop(open_list).position

            if current_node == goal:
                return self.reconstruct_path(came_from, start, goal)

            for neighbor in self.get_neighbors(current_node):
                new_cost = cost_so_far[current_node] + 1  # Assume uniform cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(goal, neighbor)
                    heapq.heappush(open_list, AStarNode(neighbor, new_cost, priority))
                    came_from[neighbor] = current_node

        return None  # No path found

    def get_neighbors(self, node):
        neighbors = []
        x, y = node
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]:  # 8 directions
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height and self.map_data[ny, nx] == 0:
                neighbors.append((nx, ny))
        return neighbors

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

    def reconstruct_path(self, came_from, start, goal):
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def inflate_obstacles(self):
        inflated_map = self.map_data.copy()
        clearance_cells = int(self.clearance_distance / self.grid_resolution)

        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if self.map_data[y, x] == 100:  # Obstacle cell
                    for dy in range(-clearance_cells, clearance_cells + 1):
                        for dx in range(-clearance_cells, clearance_cells + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                                inflated_map[ny, nx] = 100

        self.map_data = inflated_map

    def move_along_path(self):
        if not self.path:
            return False

        next_node = self.path[0]
        next_x, next_y = next_node
        current_x, current_y = self.current_position

        dx = (next_x * self.grid_resolution + self.grid_origin[0]) - (current_x * self.grid_resolution + self.grid_origin[0])
        dy = (next_y * self.grid_resolution + self.grid_origin[1]) - (current_y * self.grid_resolution + self.grid_origin[1])

        angle_to_goal = math.atan2(dy, dx)
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_orientation)

        twist = Twist()
        if abs(angle_diff) > 0.1:
            # Rotate to face the next waypoint
            twist.linear.x = 0.0
            twist.angular.z = 0.5 * angle_diff / abs(angle_diff)
        else:
            # Move forward when facing the correct direction
            twist.linear.x = min(0.2, distance_to_goal)
            twist.angular.z = 0.0
            if distance_to_goal < 0.1:
                # Reached the waypoint
                self.path.pop(0)

        self.cmd_vel_publisher.publish(twist)
        return len(self.path) == 0

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalizes the angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        """Stops the robot and logs a message once."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Goal has been reached.")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPathfinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
