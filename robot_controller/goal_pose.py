#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class PoseTracker(Node):

    def __init__(self):
        super().__init__('pose_tracker')

        # Subscribe to the robot's current position (odometry)
        self.current_pose_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Change to '/amcl_pose' if needed
            self.current_pose_callback,
            10
        )

        # Subscribe to the 2D goal pose from RViz
        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Correct topic for RViz 2D goal pose
            self.goal_pose_callback,
            10
        )

        # Store the last known positions to detect changes
        self.last_robot_position = None
        self.last_goal_position = None

    def current_pose_callback(self, msg):
        current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        formatted_current_position = self.format_position(current_position)

        if self.should_log_position(current_position):
            self.get_logger().info(f"Robot position: {formatted_current_position}")
            self.last_robot_position = current_position

    def goal_pose_callback(self, msg):
        goal_position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )

        formatted_goal_position = self.format_position(goal_position)

        if self.last_goal_position is None or goal_position != self.last_goal_position:
            self.get_logger().info(f"New goal position received: {formatted_goal_position}")
            self.last_goal_position = goal_position

    def should_log_position(self, current_position):
        if self.last_robot_position is None:
            return True

        # Calculate the Euclidean distance between the last and current positions
        distance = math.sqrt(
            (current_position[0] - self.last_robot_position[0]) ** 2 +
            (current_position[1] - self.last_robot_position[1]) ** 2
        )

        # Calculate the magnitude of the previous position
        prev_magnitude = math.sqrt(
            self.last_robot_position[0] ** 2 +
            self.last_robot_position[1] ** 2
        )

        # If the previous magnitude is zero, always log
        if prev_magnitude == 0:
            return True

        # Calculate 5% of the previous magnitude
        threshold = 0.05 * prev_magnitude

        # Return True if the distance exceeds the threshold
        return distance > threshold
    
    def format_position(self, position):
        """Format the position to three decimal places."""
        return (
            f"{position[0]:.3f}",
            f"{position[1]:.3f}",
            f"{position[2]:.3f}",
            f"{position[3]:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    pose_tracker = PoseTracker()
    rclpy.spin(pose_tracker)
    pose_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
