from ament_index_python.packages import get_package_share_path
from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def declare_args(launch_description: LaunchDescription) -> None:
    declare_vehicle_name_and_sim_time(
        launch_description=launch_description, use_sim_time_default='true'
    )
    action = DeclareLaunchArgument(
        'spawn_apriltags',
        default_value='false',
        description='Decides if AprilTags should be spawned. Recommended to be '
        'false to reduce start up time, unless required for testing visual '
        'algorithms.',
    )
    launch_description.add_action(action)


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    package_path = get_package_share_path('hippo_sim')

    declare_args(launch_description=launch_description)

    ############################################################################
    # GAZEBO
    ############################################################################
    path = str(package_path / 'launch/start_gazebo.launch.py')
    source = PythonLaunchDescriptionSource(path)
    action = IncludeLaunchDescription(source)
    launch_description.add_action(action)

    ############################################################################
    # SPAWN APRILTAGS
    ############################################################################
    condition = IfCondition(LaunchConfiguration('spawn_apriltags'))
    path = str(package_path / 'launch/spawn_apriltag_floor.launch.py')
    source = PythonLaunchDescriptionSource(path)
    action = IncludeLaunchDescription(source, condition=condition)
    launch_description.add_action(action)

    ############################################################################
    # SPAWN BLUEROV
    ############################################################################
    path = str(package_path / 'launch/spawn_bluerov.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    action = IncludeLaunchDescription(source, launch_arguments=args.items())
    launch_description.add_action(action)

    return launch_description
