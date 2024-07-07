#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math

class PoseTracker(Node):

    def __init__(self):
        super().__init__('pose_tracker')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

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

        self.timer = self.create_timer(0.1, self.go_to_goal)

        self.current_robot_position = None
        self.last_robot_position = None
        self.goal_position = None
        self.reached_goal = False
        self.need_orientation_correction = False

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

        if not self.reached_goal:
            formatted_current_position = self.format_position(self.current_robot_position)
            current_x, current_y, current_yaw = self.current_robot_position

            if self.last_robot_position is None:
                self.get_logger().info(f"Starting position: {formatted_current_position}")
            else:
                last_x, last_y, last_yaw = self.last_robot_position
                if abs(current_x - last_x) > 0.001 or abs(current_y - last_y) > 0.001 or abs(current_yaw - last_yaw) > 0.001:
                    self.get_logger().info(f"Current robot position: {formatted_current_position}")

        self.last_robot_position = self.current_robot_position

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
        self.reached_goal = False  # Reset goal status

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

    def go_to_goal(self):
        if self.current_robot_position is None or self.goal_position is None:
            return

        current_x, current_y, current_yaw = self.current_robot_position
        goal_x, goal_y, goal_yaw = self.goal_position

        distance_to_goal = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)
        angle_diff = self.normalize_angle(angle_to_goal - current_yaw)

        if distance_to_goal < 0.1:
            if not self.reached_goal:
                self.stop_robot()
                self.get_logger().info("Goal has been reached.")
                self.reached_goal = True  # Mark goal as reached

                self.need_orientation_correction = True
        else:
            twist_msg = Twist()
            if not self.reached_goal:
                if abs(angle_diff) > 0.1:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.5 * angle_diff / abs(angle_diff)
                else:
                    twist_msg.linear.x = min(0.5, distance_to_goal)
                    twist_msg.angular.z = 0.0

                self.publisher_.publish(twist_msg)

        # Check if orientation correction is needed
        if self.need_orientation_correction:
            self.orient_robot()

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)

    def orient_robot(self):
        current_x, current_y, current_yaw = self.current_robot_position
        goal_x, goal_y, goal_yaw = self.goal_position

        angle_diff_goal_orient = self.normalize_angle(goal_yaw - current_yaw)
        if abs(angle_diff_goal_orient) > 0.1:
            twist_msg = Twist()
            twist_msg.angular.z = 0.5 * angle_diff_goal_orient / abs(angle_diff_goal_orient)
            self.publisher_.publish(twist_msg)
        else:
            self.stop_robot()
            self.get_logger().info("Robot correctly oriented.")
            self.need_orientation_correction = False  # Clear the flag

def main(args=None):
    rclpy.init(args=args)
    pose_tracker = PoseTracker()
    rclpy.spin(pose_tracker)
    pose_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
