#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32,ColorRGBA, Int32MultiArray, Float32MultiArray
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np

class RadiationSource(Node):
    def __init__(self):
        super().__init__('radiation_source')

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Float32MultiArray, '/current_position', self.position_callback, 10)

        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.intensity_publisher = self.create_publisher(Float32MultiArray, '/radiation_intensity', 10)
        self.boundary_publisher = self.create_publisher(Int32MultiArray, '/map_bounds', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Radiation Source Node has been started.')

        # modeling variables
        self.Q = 100.0  # Emission rate
        self.u = 1.0 # Wind speed
        self.sigma_x = 2.0  # Standard deviation in x
        self.sigma_y = 2.0  # Standard deviation in y
        self.radius = 7
        self.num_points = 10000
        self.source_x = 1.0
        self.source_y = 1.0

        # Map properties
        self.grid_width = 0  # Number of cells in width
        self.grid_height = 0  # Number of cells in height
        self.grid_resolution = 0.0  # Meters per cell
        self.grid_origin = (0,0) # Origin of map

        # Flag to control publishing
        self.publish_flag = False
        self.read_map = False

        # Last radiation intensity
        self.last_radiation_intensity =  None
        self.particles_intensity = None
    
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

        boundaries_msg = Int32MultiArray()
        boundaries_msg.data = [self.grid_width, self.grid_height]
        self.boundary_publisher.publish(boundaries_msg)
        print("map bounds published")

    def gaussian_plume(self, x, y, x0, y0):
        distance = math.sqrt((x - x0)**2 + (y - y0)**2)
        exp = math.exp(-(distance**2) / (2 * self.sigma_y**2 * self.sigma_x**2))
        concentration = (self.Q / (2 * math.pi * self.u * self.sigma_y)) * exp
        return concentration

    def normalize_concentration(self, concentration, max_concentration):
        return min(concentration / max_concentration, 1.0)

    def get_color(self, normalized_concentration):
        # Create a gradient from blue (low) to green (high)
        color = ColorRGBA()
        color.r = 0.0
        color.g = normalized_concentration
        color.b = 1.0 - normalized_concentration
        color.a = 1.0
        return color

    def world_to_grid(self, world_position):
        grid_x = int((world_position[0] - self.grid_origin[0]) / self.grid_resolution)
        grid_y = int((world_position[1] - self.grid_origin[1]) / self.grid_resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_position):
        world_x = grid_position[0] * self.grid_resolution + self.grid_origin[0]
        world_y = grid_position[1] * self.grid_resolution + self.grid_origin[1]
        return (world_x, world_y)

    def radiation_plume(self):
        if self.read_map:
            if not self.publish_flag:
                self.publish_flag = True
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "radiation_plume"
                marker.id = 0
                marker.type = Marker.POINTS
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05

                max_intensity = 0.1
                points = []

                ''' spawning for the whole map'''
                for _ in range(self.num_points):
                    grid_x = np.random.randint(0, self.grid_width)
                    grid_y = np.random.randint(0, self.grid_height)
                    world_x, world_y = self.grid_to_world((grid_x, grid_y))
                    #print(world_x,world_y)
                    
                    intensity = self.gaussian_plume(world_x, world_y, self.source_x, self.source_y)
                    if intensity > max_intensity:
                        max_intensity = intensity

                    normalized_concentration = self.normalize_concentration(intensity, max_intensity)
                    pt = Point(x=world_x, y=world_y)
                    marker.points.append(pt)
                    marker.colors.append(self.get_color(normalized_concentration))
                
                ''' spawning for a fixed radius
                for _ in range(self.num_points):
                    r = self.radius * np.sqrt(np.random.random())
                    theta = np.random.random() * 2 * np.pi
                    x = r * np.cos(theta) + self.source_x
                    y = r * np.sin(theta) + self.source_y

                    intensity = self.gaussian_plume(x, y, self.source_x, self.source_y)
                    points.append((intensity, x, y))
                    if max_intensity is None or intensity > max_intensity:
                        max_intensity = intensity

                for intensity, x, y in points:
                    normalized_concentration = self.normalize_concentration(intensity, max_intensity)
                    pt = Point(x=x, y=y)
                    marker.points.append(pt)
                    marker.colors.append(self.get_color(normalized_concentration))
                '''
                self.marker_publisher.publish(marker)
                self.get_logger().info('Radiation plume visualization updated.')
    
    def position_callback(self, msg):
        coordinates = msg.data
        for i in range(len(coordinates)//2):
            x = coordinates[2 * i]
            y = coordinates[2 * i + 1]
            intensity = self.gaussian_plume(x, y,self.source_x, self.source_y)
            self.particles_intensity.append(intensity)
        
    def publish_intensity_array(self):
        intensity_msg = Float32MultiArray()
        intensity_msg.data = self.particles_intensity
        self.intensity_publisher.publish(intensity_msg)

    def timer_callback(self):
        self.radiation_plume()

def main(args=None):
    rclpy.init(args=args)
    node = RadiationSource()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()