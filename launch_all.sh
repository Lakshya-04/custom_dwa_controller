#!/bin/bash
# filepath: /home/hmm/custom_d_wa_controller/launch_all.sh

# Use the absolute path to your workspace to avoid issues with the ~ shortcut
WS_PATH="/home/hmm/custom_dwa_controller"

# Define the full path to your params file
# This evaluates the path once, correctly, before the terminals launch
PARAMS_FILE_PATH=$(source $WS_PATH/install/setup.bash && ros2 pkg prefix custom_dwa_local_planner)/share/custom_dwa_local_planner/config/tb4_custom_nav2_params.yaml

echo "Launching Gazebo simulation..."
x-terminal-emulator -e bash -c "source /opt/ros/humble/setup.bash;source $WS_PATH/install/setup.bash; ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py use_sim_time:=true rviz:=true; exec bash"

# --- Add a delay to let Gazebo and the controller_manager start ---
echo "Waiting for simulation to start... (15 seconds)"
sleep 15

echo "Launching SLAM..."
x-terminal-emulator -e bash -c "source /opt/ros/humble/setup.bash;source $WS_PATH/install/setup.bash; ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=true; exec bash"

# --- Add a small delay before Nav2 ---
echo "Waiting a moment before launching Nav2... (5 seconds)"
sleep 5

echo "Launching Nav2 with custom DWA planner..."
x-terminal-emulator -e bash -c "source /opt/ros/humble/setup.bash;source $WS_PATH/install/setup.bash; ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true params_file:='$PARAMS_FILE_PATH'; exec bash"