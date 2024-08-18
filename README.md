## Radiation Source Estimation and Seeking Robot in ROS2 Humble

# Description
This project is created for my 9 week long summer internship under Professor Zhongguo Li at The University of Manchester. This project aims to simulate a differential drive robot designed to estimate a source of a gamma radiation infected environment and plan a path for the robot to reach the radiation source. This repository contains the simulation of this robot in Gazebo Classic using ROS2 Humble. The radiation source is estimated using particle filtering method. This project aims to compare two path planning algorithms: A* and Reinforcement Learning.
# Usage
To run this simulation, run the following files:
1. Gazebo

Gazebo is used to intialize the environment and it is launched using:

```
ros2 launch uom_intern_testbot launch_sim.launch.py world:=./src/uom_intern_testbot/worlds/maze.world
```

2. RVIZ

RVIZ is used to visualize the robot's view of the environment and it is launched using:

ros2 launch uom_intern_testbot launch_sim.launch.py world:=./src/uom_intern_testbot/worlds/maze.world

3. SLAM toolbox

The SLAM toolbox is used to map a pre-saved map of the world in Gazebo. The custom map is launched using:

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/uom_intern_testbot/config/mapper_params_online_async.yaml use_sim_time:=true

4. Radiation source

To initialize the radiation source, run the radiation_source.py using:

ros2 run uom_intern_testbot radiation_source.py

5. Path Planning method

The A* path planner can be launched using:

ros2 run uom_intern_testbot particle_filter.py

The reinforcement learning based path planner can be launched using:

work in progress...
