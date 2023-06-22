from ament_index_python.packages import get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def declare_args(launch_description: LaunchDescription):
    action = DeclareLaunchArgument(
        'spawn_apriltags',
        default_value='false',
        description='Decides if AprilTags should be spawned. Recommended to be '
        'false to reduce start up time, unless required for testing visual '
        'algorithms.')
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    package_path = get_package_share_path('hippo_sim')

    declare_args(launch_description=launch_description)

    ############################################################################
    # GAZEBO
    ############################################################################
    path = str(package_path / 'launch/start_gazebo.launch.py'),
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
    # SPAWN HIPPCAMPUS
    ############################################################################
    path = str(package_path / 'launch/spawn_hippocampus.launch.py')
    source = PythonLaunchDescriptionSource(path)
    action = IncludeLaunchDescription(source,
                                      launch_arguments={
                                          'vehicle_name': 'uuv00',
                                          'use_sim_time': 'true'
                                      }.items())
    launch_description.add_action(action)

    return launch_description
