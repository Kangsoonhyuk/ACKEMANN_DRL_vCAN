import os
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_MERGED_YAML = '/tmp/ackermann_car_controllers.yaml'


def _get_urdf(xacro_file: str, controllers_yaml_path: str) -> str:
    result = subprocess.run(
        ['xacro', xacro_file, f'controllers_yaml:={controllers_yaml_path}'],
        capture_output=True, text=True, check=True,
    )
    return ' '.join(result.stdout.split())


def _build_merged_yaml(urdf: str, base_yaml: str) -> str:
    with open(base_yaml) as f:
        params = yaml.safe_load(f)
    cm = params.setdefault('controller_manager', {}) \
               .setdefault('ros__parameters', {})
    cm['robot_description'] = urdf
    with open(_MERGED_YAML, 'w') as f:
        yaml.dump(params, f, default_flow_style=False, allow_unicode=True)
    return _MERGED_YAML


def generate_launch_description():
    pkg_car    = get_package_share_directory('ackermann_car_description')
    pkg_world  = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    urdf_xacro = os.path.join(pkg_car, 'urdf', 'ackermann_car.urdf.xacro')
    base_yaml  = os.path.join(pkg_car, 'config', 'controllers.yaml')
    world      = os.path.join(pkg_world, 'worlds', 'turtlebot3_drl_stage2', 'waffle.model')

    urdf_for_merge = _get_urdf(urdf_xacro, base_yaml)
    merged_yaml    = _build_merged_yaml(urdf_for_merge, base_yaml)
    urdf_content   = _get_urdf(urdf_xacro, merged_yaml)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'pause': 'false'}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gzclient.launch.py')
        ),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': urdf_content,
        }],
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ackermann_car',
        output='screen',
        arguments=[
            '-entity', 'ackermann_car',
            '-topic',  'robot_description',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.065',
        ],
    )

    jsb_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            name='joint_state_broadcaster_spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        )],
    )

    ackermann_spawner = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            name='ackermann_steering_controller_spawner',
            arguments=['ackermann_steering_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        )],
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel',
                   '/ackermann_steering_controller/reference_unstamped'],
        output='screen',
    )

    tf_odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='tf_odom_relay',
        arguments=['/ackermann_steering_controller/tf_odometry', '/tf'],
        output='screen',
    )

    rviz_config = os.path.join(pkg_car, 'rviz', 'stage5_up.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        gzserver,
        gzclient,
        rsp,
        spawn,
        jsb_spawner,
        ackermann_spawner,
        cmd_vel_relay,
        tf_odom_relay,
        rviz,
    ])
