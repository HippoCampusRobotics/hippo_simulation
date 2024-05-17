from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def declare_launch_args(launch_description: LaunchDescription) -> None:
    action = DeclareLaunchArgument(
        name='start_gui',
        default_value='false',
        description='Start the gazebo GUI.',
    )
    launch_description.add_action(action)

    package_path = get_package_share_path('hippo_sim')
    world_file = str(package_path / 'models' / 'world' / 'empty.sdf')
    action = DeclareLaunchArgument(name='world_file', default_value=world_file)
    launch_description.add_action(action)


def create_gazebo_action() -> ExecuteProcess:
    return ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-v 2',
            '-r',
            '-s',
            '--headless-rendering',
            LaunchConfiguration('world_file'),
        ],
        output='screen',
        on_exit=Shutdown(),
    )


def create_gazebo_gui_action() -> ExecuteProcess:
    return ExecuteProcess(
        cmd=['ign', 'gazebo', '-g', '-v 1'],
        condition=IfCondition(LaunchConfiguration('start_gui')),
        output='log',
        on_exit=Shutdown(),
    )


def create_spawn_pool_action() -> Node:
    package_path = get_package_share_path('hippo_sim')
    pool_path = package_path / 'models/pool/urdf/pool.xacro'
    pool_description = LaunchConfiguration(
        'pool_description',
        default=Command(
            [
                'ros2 run hippo_sim create_robot_description.py ',
                '--input ',
                str(pool_path),
            ]
        ),
    )
    pool_params = {'pool_description': pool_description}
    return Node(
        package='hippo_sim',
        executable='spawn',
        parameters=[pool_params],
        arguments=[
            '--param',
            'pool_description',
            '--x',
            '1.0',
            '--y',
            '2.0',
            '--z',
            '-1.5',
        ],
        output='screen',
    )


def create_clock_bridge_action() -> Node:
    return Node(
        name='clock_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    actions = [
        create_spawn_pool_action(),
        create_clock_bridge_action(),
        create_gazebo_action(),
        create_gazebo_gui_action(),
    ]
    for action in actions:
        launch_description.add_action(action)
    return launch_description
