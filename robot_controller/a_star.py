#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
import heapq

class AStarPlanner(Node):

    def __init__(self):
        super().__init__('a_star_planner')

        # Subscribers
        self.current_pose_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.current_pose_callback,
            10
        )

        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize state
        self.current_position = None
        self.goal_position = None
        self.obstacles = []
        self.grid_resolution = 0.2  # Adjust as per the resolution of your grid
        self.grid_size = 100  # Adjust based on the environment size

    #updates current position of robot from /odom 
    def current_pose_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    #updates goal position of robot from /goal_pose
    def goal_pose_callback(self, msg):
        self.goal_position = (
            msg.pose.position.x,
            msg.pose.position.y
        )
        self.get_logger().info(f"Goal Set Pose: {self.goal_position}")
        self.plan_path()

    #updates map data from topic /map
    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_info = msg.info

    #checks if current position are initialised propertly before dollowing the path
    def plan_path(self):
        if self.current_position is None or self.goal_position is None or self.map_data is None:
            return

        path = self.a_star_search(self.current_position, self.goal_position)
        if path:
            self.follow_path(path)

    def a_star_search(self, start, goal):
        start_node = (self.grid_position(start), 0, self.heuristic(start, goal))
        goal_node = self.grid_position(goal)
        open_set = []
        heapq.heappush(open_set, start_node)
        came_from = {}
        g_score = {start_node[0]: 0}
        f_score = {start_node[0]: self.heuristic(start, goal)}

        while open_set:
            current_node = heapq.heappop(open_set)
            current_pos = current_node[0]
            if current_pos == goal_node:
                return self.reconstruct_path(came_from, current_pos)

            for neighbor in self.get_neighbors(current_pos):
                tentative_g_score = g_score[current_pos] + self.distance(current_pos, neighbor)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current_pos
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_node)
                    if neighbor not in [node[0] for node in open_set]:
                        heapq.heappush(open_set, (neighbor, g_score[neighbor], f_score[neighbor]))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return [self.world_position(pos) for pos in path]

    def get_neighbors(self, position):
        neighbors = [
            (position[0] + 1, position[1]),
            (position[0] - 1, position[1]),
            (position[0], position[1] + 1),
            (position[0], position[1] - 1)
        ]
        return [n for n in neighbors if self.is_walkable(n)]

    def is_walkable(self, position):
        index = self.grid_to_index(position)
        if index < 0 or index >= len(self.map_data):
            return False
        return self.map_data[index] < 50  # Assuming 0-100 scale, where <50 is walkable

    def grid_position(self, world_pos):
        x = int((world_pos[0] - self.map_info.origin.position.x) / self.map_info.resolution)
        y = int((world_pos[1] - self.map_info.origin.position.y) / self.map_info.resolution)
        return (x, y)

    def world_position(self, grid_pos):
        x = grid_pos[0] * self.map_info.resolution + self.map_info.origin.position.x
        y = grid_pos[1] * self.map_info.resolution + self.map_info.origin.position.y
        return (x, y)

    def grid_to_index(self, grid_pos):
        x, y = grid_pos
        return y * self.map_info.width + x

    def distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

    def heuristic(self, pos1, pos2):
        return self.distance(pos1, pos2)

    def follow_path(self, path):
        for point in path:
            twist_msg = Twist()
            twist_msg.linear.x = 0.5
            angle = math.atan2(point[1] - self.current_position[1], point[0] - self.current_position[0])
            twist_msg.angular.z = angle
            self.cmd_vel_publisher.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=1)

def main(args=None):
    rclpy.init(args=args)
    a_star_planner = AStarPlanner()
    rclpy.spin(a_star_planner)
    a_star_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()