#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math

class RadiationListener(Node):
    def __init__(self):
        super().__init__('radiation_listener')

        # Subscribe to the /odom topic to get the current position
        self.create_subscription(Odometry, '/odom', self.current_pose_callback, 10)
        
        # Publish current position
        self.position_publisher = self.create_publisher(Point, '/current_position', 10)

        # Subscribe to a custom radiation intensity topic
        self.create_subscription(Float32, '/radiation_intensity', self.radiation_callback, 10)

        # Variable to store current position
        self.current_position = None
        self.last_position = None

        # Variable to store current radiation intensity
        self.current_radiation_intensity = None
        self.last_radiation_intensity = None

        self.get_logger().info('Radiation Detect Node has been started.')

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def current_pose_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
        )
        self.publish_current_position()

    def publish_current_position(self):
        position_msg = Point()
        position_msg.x = self.current_position[0]
        position_msg.y = self.current_position[1]
        self.position_publisher.publish(position_msg)

        if self.last_position is not None:
            last_x, last_y, _ = self.last_position
            current_x, current_y, _ = self.current_position
            if abs(current_x - last_x) > 0.1 or abs(current_y - last_y) > 0.1:
                self.get_logger().info(f'Current Position Published: ({position_msg.x:.3f}, {position_msg.y:.3f})')

                self.last_position = self.current_position


    def radiation_callback(self, msg):
        # Update radiation intensity
        self.current_radiation_intensity = msg.data
        self.get_logger().info(f'Received Radiation Intensity: {msg.data:.3f}')

        if self.last_radiation_intensity is not None:
            if abs(self.current_radiation_intensity - self.last_radiation_intensity) > 0.1:
                self.get_logger().info(f'Received Radiation Intensity: {msg.data:.3f}')

                self.last_radiation_intensity = self.current_radiation_intensity
        

def main(args=None):
    rclpy.init(args=args)
    node = RadiationListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
