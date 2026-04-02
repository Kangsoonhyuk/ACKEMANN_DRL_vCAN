import os
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 런치 실행마다 생성되는 임시 merged yaml 경로
_MERGED_YAML = '/tmp/ackermann_car_controllers.yaml'


def _get_urdf(xacro_file: str, controllers_yaml_path: str) -> str:
    """xacro 실행 후 단일 라인으로 압축.

    gazebo_ros2_control 0.4.x 버그 근본 원인:
      '--param robot_description:=<URDF>' 를 rcl 이 YAML plain scalar 로 파싱할 때
      YAML block scalar 규칙상 continuation line 은 이전 줄보다 들여쓰기가 더 깊어야 함.
      URDF 는 모든 줄이 column 0 에서 시작하므로 libyaml 이 첫 줄에서 scalar 를 끊고
      나머지를 별개 토큰으로 파싱 → 파싱 실패 → CM 생성 불가.

    해결: 모든 공백·개행을 단일 스페이스로 압축 → 단일 라인 YAML plain scalar.
    XML 파서(urdfdom, TinyXML2)는 공백 압축된 XML 을 정상 처리한다.
    """
    result = subprocess.run(
        ['xacro', xacro_file, f'controllers_yaml:={controllers_yaml_path}'],
        capture_output=True, text=True, check=True,
    )
    urdf = result.stdout
    # 단일 라인으로 압축 (XML 선언 포함 전체를 공백 기준으로 join)
    return ' '.join(urdf.split())


def _build_merged_yaml(urdf: str, base_yaml: str) -> str:
    """controllers.yaml + robot_description 를 병합한 YAML 파일을 생성.

    gazebo_ros2_control은 '--params-file' 경로로 YAML을 읽음.
    YAML은 XML 특수문자를 올바르게 이스케이핑하므로 controller_manager가
    robot_description을 정상적으로 읽을 수 있음.
    """
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
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    urdf_xacro = os.path.join(pkg_car, 'urdf', 'ackermann_car.urdf.xacro')
    base_yaml  = os.path.join(pkg_car, 'config', 'controllers.yaml')

    # 1) URDF 생성 (임시: base_yaml 경로로 xacro 실행)
    urdf_for_merge = _get_urdf(urdf_xacro, base_yaml)

    # 2) robot_description 포함된 merged yaml 생성
    merged_yaml = _build_merged_yaml(urdf_for_merge, base_yaml)

    # 3) 최종 URDF 생성 (Gazebo 플러그인 <parameters>가 merged yaml을 가리킴)
    urdf_content = _get_urdf(urdf_xacro, merged_yaml)

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')

    # ----------------------------------------------------------------
    # 1. Gazebo
    # ----------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   '',
            'pause':   'false',
            'verbose': 'false',
        }.items(),
    )

    # ----------------------------------------------------------------
    # 2. robot_state_publisher
    # ----------------------------------------------------------------
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

    # ----------------------------------------------------------------
    # 3. 모델 스폰 (URDF topic → Gazebo 내부 변환)
    # ----------------------------------------------------------------
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
            '-z', '0.065',   # wheel_r → 바퀴가 지면에 닿는 높이
        ],
    )

    # ----------------------------------------------------------------
    # 4. 컨트롤러 spawner (Gazebo + 플러그인 기동 대기 후 자동 실행)
    # ----------------------------------------------------------------
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

    # /cmd_vel → /ackermann_steering_controller/reference_unstamped 릴레이
    # (use_stamped_vel: false 이므로 reference_unstamped 토픽 사용)
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel',
                   '/ackermann_steering_controller/reference_unstamped'],
        output='screen',
    )

    # ----------------------------------------------------------------
    # 5. RViz2
    # ----------------------------------------------------------------
    rviz_config = os.path.join(pkg_car, 'launch', 'test_car.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_x,
        declare_y,
        gazebo,
        rsp,
        spawn,
        jsb_spawner,
        ackermann_spawner,
        cmd_vel_relay,
        rviz,
    ])
