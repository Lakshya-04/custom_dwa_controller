"""
Launch Ignition Fortress (gz_sim), start ros_gz_bridge parameter_bridge for common topics
and spawn TurtleBot3 SDF model into the world.


Usage:
ros2 launch tb3_fortress_sim bringup.launch.py


This launch assumes you have the turtlebot3 SDF at: $(find tb3_fortress_sim)/models/turtlebot3_burger/model.sdf
If not, run the provided download script in the package's scripts/ folder.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_fortress_sim')
    model_sdf = os.path.join(pkg_share, 'models', 'turtlebot3_burger', 'model.sdf')


    # 1) Launch gz_sim (GUI) using the ros_gz_sim gz_sim.launch.py
    gz_launch = ExecuteProcess(
    cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', 'gz_args:=empty.sdf'],
    output='screen'
    )


    # 2) Start parameter_bridge for common topics
    bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    output='screen',
    arguments=[
    '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Time',
    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
    '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    ]
    )


    # 3) Spawn the TurtleBot3 model after a short delay so gz_sim has time to start
    spawn_cmd = Node(
    package='ros_gz_sim',
    executable='create',
    output='screen',
    arguments=['-file', model_sdf, '-name', 'turtlebot3']
    )


    # TimerAction to delay spawning by 5 seconds
    spawn_after = TimerAction(
    period=5.0,
    actions=[LogInfo(msg='Spawning TurtleBot3 model...'), spawn_cmd]
    )


    ld = LaunchDescription()
    ld.add_action(gz_launch)
    ld.add_action(bridge_node)
    ld.add_action(spawn_after)


    return ld