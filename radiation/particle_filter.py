#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Int32MultiArray, Float32MultiArray
import numpy as np

class RadiationListener(Node):
    def __init__(self):
        super().__init__('radiation_listener')

        # Subscriptions
        self.create_subscription(Int32MultiArray, '/map_bounds', self.map_bounds_callback, 10)
        self.create_subscription(Float32MultiArray, '/radiation_intensity', self.radiation_callback, 10)

        # Publishers
        self.particle_position_publisher = self.create_publisher(Float32MultiArray, '/particle_position', 10)
        self.goal_estimate_publisher = self.create_publisher(Point, '/goal_estimate_position', 10)

        # Particle filter variables
        self.num_of_particles = 1000
        self.width = None
        self.height = None
        self.particles = None
        self.radiation_intensity = None
        self.weight = np.zeros(self.num_of_particles)
        self.pf = ParticleFilter()

        # Timer for running the particle filter
        self.timer = self.create_timer(0.5, self.particle_filter)
        
        # Initial logging
        self.get_logger().info('Radiation Detect Node has been started.')

    # Gets map bounds and calls initialize particles
    def map_bounds_callback(self, msg):
        # Log and initialize upon receiving map boundaries
        self.width, self.height = msg.data
        self.get_logger().info(f'Map bounds received: Width = {self.width}, Height = {self.height}')
        self.initialize_particles()

    def initialize_particles(self):
        # Initialize particles within the boundaries of the map
        self.particles = self.pf.initialize_particles(self.num_of_particles, self.width, self.height)
        print('Particles initialized')

    def radiation_callback(self, msg):
        # Get radiation array
        if msg.data is None:
            print("particles not initialized")
            return
        self.radiation_intensity = msg.data
        self.get_logger().info(f'Received Radiation Intensity')

    def publish_particles(self):
        particle_msg = Float32MultiArray()
        flat_particles = self.particles.flatten().astype(np.float32)
        particle_msg.data = flat_particles.tolist()  # Convert numpy array to list
        self.particle_position_publisher.publish(particle_msg)

    def particle_filter(self):
        # Skip processing if particles are not initialized
        if self.particles is None:
            print('Waiting for particle initialization...')
            return
        if self.width is None or self.height is None:
            print('Map dimensions are not yet available.')
            return

        self.publish_particles()
        print(self.weight)
        self.weight /= np.sum(self.weight)

        # Update and resample particles based on radiation intensity
        neff = self.pf.neff(self.weight)
        if neff < self.num_of_particles * 0.5:
            self.weight = self.pf.resample(self.num_of_particles, self.particles, self.weight)
            self.pf.mcmc_move(self.num_of_particles, self.particles, self.width, self.height)

        estimated_position = self.pf.estimate(self.particles, self.weight)
        print(f'Estimated Position: {estimated_position}')
        self.publish_goal_estimate(estimated_position)

    def publish_goal_estimate(self, estimated_position):
        # Publish the estimated goal position
        goal_msg = Point(x=estimated_position[0], y=estimated_position[1], z=0.0)
        self.goal_estimate_publisher.publish(goal_msg)
        self.get_logger().info(f'Published Goal Estimate Position: {goal_msg.x}, {goal_msg.y}')


class ParticleFilter:
    def initialize_particles(self, num_particles, width, height):
        ''' Initialize particles to random positions within the defined boundary. '''
        particles = np.empty((num_particles, 2))
        particles[:, 0] = np.random.uniform(0, width, num_particles)
        particles[:, 1] = np.random.uniform(0, height, num_particles)
        return particles

    def predict(self, num_particles, particles, width, height, move_dist=1):
        ''' Predict the next state of the particles based on the previous state with random movements. '''
        movements = np.random.randn(num_particles, 2) * move_dist
        particles += movements
        particles[:, 0] = np.clip(particles[:, 0], 0, width)
        particles[:, 1] = np.clip(particles[:, 1], 0, height)

    def update(self,concentrations):
        ''' Update particle weights based on the measured concentration values. '''
        weights = concentrations
        weights /= np.sum(weights)  # Normalize
        return weights

    def resample(self, num_particles, particles, weights):
        ''' Resample particles according to their weights to focus on high probability areas. '''
        cumulative_sum = np.cumsum(weights)
        cumulative_sum[-1] = 1.0  # Ensure the sum of weights is exactly one
        indexes = np.searchsorted(cumulative_sum, np.random.random(num_particles))

        # resample according to indexes
        particles = particles[indexes]
        weights = np.ones(num_particles) / num_particles
        return weights

    def neff(self, weights):
        return 1. / np.sum(np.square(weights))

    def mcmc_move(self, num_particles, particles, width, height, std_dev=0.1):
        """
        Apply a Markov Chain Monte Carlo move to each particle to introduce small random changes.
        std_dev: Standard deviation of the Gaussian noise added for the MCMC move.
        """
        noise = np.random.normal(0, std_dev, size=(num_particles, 2))
        particles += noise
        particles[:, 0] = np.clip(particles[:, 0], 0, width)
        particles[:, 1] = np.clip(particles[:, 1], 0, height)

    def estimate(self, particles, weights):
        ''' Estimate the current position of the source based on the particles. '''
        mean = np.average(particles, weights=weights, axis=0)
        return mean

def main(args=None):
    rclpy.init(args=args)
    node = RadiationListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
