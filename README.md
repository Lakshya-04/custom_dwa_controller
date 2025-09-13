# tb3_fortress_sim


A small ROS2 Humble package to launch Ignition Fortress (gz_sim/ign gazebo), start a basic ros_gz_bridge for `/cmd_vel`, `/odom`, `/scan` and `/clock`, and spawn a TurtleBot3 burger into the world.


## Setup


1. Put this package inside your ROS2 workspace `~/ros2_ws/src/`.
2. Download the model SDF (or run the script):


```bash
cd ~/ros2_ws/src/tb3_fortress_sim
./scripts/download_tb3_model.sh

# Custom DWA Local Planner for ROS 2 Humble

This package implements a custom Dynamic Window Approach (DWA) local planner from scratch as a `nav2_core::Controller` plugin for the ROS 2 Navigation Stack (Nav2).

## Features
- Implements the DWA algorithm for local motion planning.
- Samples velocities within the robot's dynamic constraints.
- Simulates trajectories and evaluates them using a multi-objective cost function:
  - **Goal Distance:** Steers the robot toward the final goal.
  - **Path Distance:** Encourages the robot to follow the global plan.
  - **Obstacle Avoidance:** Prevents collisions using the Nav2 costmap.
- Publishes sampled trajectories to RViz for visualization and debugging.

## 1. Build Instructions

Navigate to your ROS 2 workspace and build the package:
```bash
cd ~/your_ros2_ws
colcon build --packages-select custom_dwa_local_planner



references 
https://www.youtube.com/watch?v=OEkIWiLqCsk
https://github.com/blackcoffeerobotics/vector_pursuit_controller
https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/launch/slam.launch.py
https://github.com/turtlebot/turtlebot4_simulator/blob/humble/turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py
https://www.youtube.com/watch?v=cW_9KRL_rA0
https://www.youtube.com/watch?v=16TJ6NiHo6A
