# tb3_fortress_sim


A small ROS2 Humble package to launch Ignition Fortress (gz_sim/ign gazebo), start a basic ros_gz_bridge for `/cmd_vel`, `/odom`, `/scan` and `/clock`, and spawn a TurtleBot3 burger into the world.


## Setup


1. Put this package inside your ROS2 workspace `~/ros2_ws/src/`.
2. Download the model SDF (or run the script):


```bash
cd ~/ros2_ws/src/tb3_fortress_sim
./scripts/download_tb3_model.sh

references 
https://www.youtube.com/watch?v=OEkIWiLqCsk
https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/launch/slam.launch.py
https://github.com/turtlebot/turtlebot4_simulator/blob/humble/turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py
        