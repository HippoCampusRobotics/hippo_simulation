from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def add_generate_pool_node(launch_description: LaunchDescription):
    args = [
        " --tag-poses-file '",
        LaunchConfiguration('tag_poses_file'),
        "'",
        ' --force',
    ]
    cmd = 'ros2 run hippo_sim generate_pool.py'
    sdf = LaunchConfiguration('sdf', default=Command([cmd, *args]))
    params = {'description': sdf}
    action = Node(
        package='hippo_sim',
        executable='spawn',
        parameters=[params],
        arguments=['--param', 'description'],
        output='screen',
    )
    launch_description.add_action(action)


def declare_launch_arguments(launch_description: LaunchDescription):
    package_path = get_package_share_path('hippo_sim')
    default_urdf_path = package_path / 'models/apriltag/urdf/apriltag.xacro'
    default_tag_poses_file = package_path / 'config/tag_poses.yaml'

    action = DeclareLaunchArgument(
        name='urdf_path',
        default_value=str(default_urdf_path),
        description='Absolute path to apriltag .xacro',
    )
    launch_description.add_action(action)
    action = DeclareLaunchArgument(
        name='tag_poses_file',
        default_value=str(default_tag_poses_file),
        description=(
            'Path to the file containing the tag poses of the tags'
            ' to be spawned.'
        ),
    )
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_arguments(launch_description)
    add_generate_pool_node(launch_description)
    return launch_description
