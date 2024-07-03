#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity = 0.2  # m/s
        self.angular_velocity = 0.4  # rad/s
        self.distance_threshold = 1.0  # meters
        self.safe_distance = 2.0  # meters

    def scan_callback(self, msg):
        # Process laser scan data
        ranges = msg.ranges
        min_distance = min(ranges)

        if min_distance < self.distance_threshold:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.8  # Turn to avoid obstacle
        else:
            self.linear_velocity = 0.2
            self.angular_velocity = 0.4  # Continue circling

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()