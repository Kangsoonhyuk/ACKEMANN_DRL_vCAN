#!/usr/bin/env python3
"""
Launch: stage5_up world + Ackermann car with lidar & camera
TF: odom -> base_footprint -> base_link -> sensors/wheels
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_car   = get_package_share_directory('ackermann_car_description')
    pkg_world = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(pkg_world, 'worlds', 'turtlebot3_drl_stage5_up', 'waffle.model')
    urdf  = os.path.join(pkg_car, 'urdf', 'ackermann_car.urdf.xacro')

    # xacro -> robot_description string (must be explicitly typed as str)
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose',       default_value='0.0',
                              description='Car spawn x position'),
        DeclareLaunchArgument('y_pose',       default_value='0.0',
                              description='Car spawn y position'),

        # 1. Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world, 'pause': 'false'}.items(),
        ),

        # 2. Gazebo client (GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gzclient.launch.py')
            ),
        ),

        # 3. robot_state_publisher (URDF -> TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),

        # 4. Spawn car into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_ackermann_car',
            output='screen',
            arguments=[
                '-entity',  'ackermann_car',
                '-topic',   'robot_description',
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.05',
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '0.0',
            ],
        ),
    ])
