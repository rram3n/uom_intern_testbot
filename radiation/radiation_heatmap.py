#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import numpy as np

class RadiationSource(Node):
    def __init__(self):
        super().__init__('radiation_source')

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.heatmap_publisher = self.create_publisher(OccupancyGrid, 'radiation_heatmap', 10)

        self.timer = self.create_timer(1.0, self.publish_heatmap)

        self.get_logger().info('Radiation Source Node has been started.')

        # Grid properties
        self.grid_width = 0  # Number of cells in width
        self.grid_height = 0  # Number of cells in height
        self.grid_resolution = 0.0  # Meters per cell
        self.grid_origin = (0,0) # Origin of map

        # Map flag
        self.read_map = False
        self.heatmap_publish_flag = False

        # Radiation modeling
        self.source_strength = 1.0
        self.source_x = 1  # X position of source in meters
        self.source_y = 1  # Y position of source in meters
        self.emission_radius = 6

    def map_callback(self, msg):
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height
        self.grid_resolution = msg.info.resolution
        self.grid_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.data = msg.data

        self.map_data = [[0 for _ in range(self.grid_width)] for _ in range(self.grid_height)]
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                self.map_data[i][j] = self.data[i * self.grid_width + j]

        # convert obstacles
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if self.map_data[y][x] == -1: # unknown cell
                    self.map_data[y][x] = 1
                if self.map_data[y][x] == 100:  # Obstacle cell
                    self.map_data[y][x] = 1

        if self.read_map is False:
            self.read_map = True
            self.get_logger().info(f"Map received with dimensions: {self.grid_width}x{self.grid_height}")

    def radiation_intensity(self, x, y):
        """Calculate radiation intensity based on a Gaussian distribution centered at the source."""
        distance_squared = (x - self.source_x)**2 + (y - self.source_y)**2
        if distance_squared == 0:
            return self.source_strength  # Maximum possible intensity at the source location
        elif math.sqrt(distance_squared) <= self.emission_radius:
            sigma = 10.0  # Standard deviation in meters
            intensity = self.source_strength * math.exp(-distance_squared / (2 * sigma ** 2))
            return intensity
        else: return 0

    def world_to_grid(self, world_position):
        grid_x = int((world_position[0] - self.grid_origin[0]) / self.grid_resolution)
        grid_y = int((world_position[1] - self.grid_origin[1]) / self.grid_resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_position):
        world_x = grid_position[0] * self.grid_resolution + self.grid_origin[0]
        world_y = grid_position[1] * self.grid_resolution + self.grid_origin[1]
        return (world_x, world_y)

    def publish_heatmap(self):
        if self.read_map:
            grid = OccupancyGrid()
            now = self.get_clock().now()
            grid.header = Header()
            grid.header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
            grid.header.frame_id = "map"
            intensities = []
            max_intensity = 0.01  # Prevent division by zero


            for x in range(self.grid_width):
                for y in range(self.grid_height):
                    intensity = self.radiation_intensity(x, y)
                    if not np.isnan(intensity):  # Check for NaN values
                        intensities.append(intensity)
                        max_intensity = max(max_intensity, intensity)

            # Normalize and prepare the data for the grid, checking for NaN values
            grid.data = [int(100 * (i / max_intensity)) if not np.isnan(i) else 0 for i in intensities]
            self.heatmap_publisher.publish(grid)
            #print(grid.data)

            if self.heatmap_publish_flag is False:
                self.get_logger().info('Heatmap published.')
                self.heatmap_publish_flag = True

def main(args=None):
    rclpy.init(args=args)
    node = RadiationSource()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
