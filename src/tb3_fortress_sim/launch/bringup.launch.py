# """
# Launch Ignition Fortress (gz_sim), start ros_gz_bridge parameter_bridge for common topics
# and spawn TurtleBot3 SDF model into the world.


# Usage:
# ros2 launch tb3_fortress_sim bringup.launch.py


# This launch assumes you have the turtlebot3 SDF at: $(find tb3_fortress_sim)/models/turtlebot3_burger/model.sdf
# If not, run the provided download script in the package's scripts/ folder.
# """

# import os
# from pathlib import Path

# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, TimerAction, LogInfo
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory



# def generate_launch_description():
#     pkg_share = get_package_share_directory('tb3_fortress_sim')
#     model_sdf = os.path.join(pkg_share, 'models', 'turtlebot3_burger', 'model.sdf')


#     # 1) Launch gz_sim (GUI) using the ros_gz_sim gz_sim.launch.py
#     gz_launch = ExecuteProcess(
#     cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', 'gz_args:=empty.sdf'],
#     output='screen'
#     )


#     # 2) Start parameter_bridge for common topics
#     bridge_node = Node(
#     package='ros_gz_bridge',
#     executable='parameter_bridge',
#     output='screen',
#     arguments=[
#     '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Time',
#     '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
#     '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
#     '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
#     ]
#     )


#     # 3) Spawn the TurtleBot3 model after a short delay so gz_sim has time to start
#     spawn_cmd = Node(
#     package='ros_gz_sim',
#     executable='create',
#     output='screen',
#     arguments=['-file', model_sdf, '-name', 'turtlebot3']
#     )


#     # TimerAction to delay spawning by 5 seconds
#     spawn_after = TimerAction(
#     period=5.0,
#     actions=[LogInfo(msg='Spawning TurtleBot3 model...'), spawn_cmd]
#     )


#     ld = LaunchDescription()
#     ld.add_action(gz_launch)
#     ld.add_action(bridge_node)
#     ld.add_action(spawn_after)


#     return ld

#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Package directories   
    pkg_tb3_fortress_sim = FindPackageShare('tb3_fortress_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name', default='turtlebot3_burger')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')

    # Paths
    world_file = PathJoinSubstitution([pkg_tb3_fortress_sim, 'worlds', 'turtlebot3_world.sdf'])
    model_file = PathJoinSubstitution([pkg_tb3_fortress_sim, 'models', 'turtlebot3_burger', 'model.sdf'])
    bridge_config = PathJoinSubstitution([pkg_tb3_fortress_sim, 'config', 'bridge_params.yaml'])

    # Set Gazebo resource path
    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([pkg_tb3_fortress_sim, 'models'])
    )

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn TurtleBot3
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-file', model_file,
            '-x', x_pose,
            '-y', y_pose, 
            '-z', z_pose
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge using YAML config
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Alternative: Manual bridge (uncomment if YAML config doesn't work)
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    #         '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
    #         '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    #         '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
    #         '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
    #         '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    #         '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    #     ],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='turtlebot3_burger'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),

        # Actions
        set_env_vars,
        gazebo_launch,
        spawn_robot,
        bridge,
    ])