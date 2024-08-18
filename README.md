## Radiation Source Estimation and Seeking Robot in ROS2 Humble

# Description
This project was developed during a nine-week summer internship under Professor Zhongguo Li at The University of Manchester. It aims to simulate a differential drive robot in Gazebo Classic using ROS2 Humble, designed to estimate the location of a gamma radiation source in an infected environment and plan the robot's path to reach this source. The project utilizes particle filtering to estimate radiation sources and compares two path planning algorithms: A* and a Reinforcement Learning based path planning algorithm.
# Usage
To run this simulation, run the following files:

1. Launching Gazebo

Initialize the environment with Gazebo by running:

```
ros2 launch uom_intern_testbot launch_sim.launch.py world:=./src/uom_intern_testbot/worlds/maze.world
```

2. Visualizing with RViz

To visualize the robot's perspective within the environment, launch RViz using:

```
ros2 launch uom_intern_testbot launch_sim.launch.py world:=./src/uom_intern_testbot/worlds/maze.world
```

3. Mapping with SLAM Toolbox

Use the SLAM toolbox to load a pre-saved map of the custom world in Gazebo:

```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/uom_intern_testbot/config/mapper_params_online_async.yaml use_sim_time:=true
```

4. Initializing the Radiation Source

Start the simulation of the radiation source by running:

```
ros2 run uom_intern_testbot radiation_source.py
```

5. Path Planning method

The A* path planner is launched using (currently in development):

```
ros2 run uom_intern_testbot a_star_particle_filter.py
```

For the Reinforcement Learning based path planner (currently in development):
```
ros2 run uom_intern_testbot rl_particle_filter.py
```