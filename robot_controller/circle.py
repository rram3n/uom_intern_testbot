#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CircleNode(Node):
    def __init__(self):
        super().__init__('circle_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_count = 0
        self.linear_velocity = 0.2  # m/s
        self.angular_velocity = 0.4  # rad/s

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    circle_node = CircleNode()
    rclpy.spin(circle_node)
    circle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
