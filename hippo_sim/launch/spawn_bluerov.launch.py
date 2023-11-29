from ament_index_python.packages import get_package_share_path
from hippo_common import launch_helper
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_args(launch_description: LaunchDescription) -> None:
    action = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(action)
    action = DeclareLaunchArgument('use_sim_time')
    launch_description.add_action(action)
    action = DeclareLaunchArgument('use_vertical_camera', default_value='False')
    launch_description.add_action(action)
    action = DeclareLaunchArgument('use_front_camera', default_value='False')
    launch_description.add_action(action)
    action = DeclareLaunchArgument('use_range_sensor', default_value='false')
    launch_description.add_action(action)


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    package_path = get_package_share_path('hippo_sim')

    declare_args(launch_description=launch_description)

    ############################################################################
    # Vertical Camera Bridge
    ############################################################################
    action = launch_helper.create_camera_bridge(
        vehicle_name=LaunchConfiguration('vehicle_name'),
        camera_name='vertical_camera',
        use_camera=LaunchConfiguration('use_vertical_camera'),
        image_name='image_rect')
    launch_description.add_action(action)

    ############################################################################
    # Front Camera Bridge
    ############################################################################
    action = launch_helper.create_camera_bridge(
        vehicle_name=LaunchConfiguration('vehicle_name'),
        camera_name='front_camera',
        use_camera=LaunchConfiguration('use_front_camera'),
        image_name='image_rect')
    launch_description.add_action(action)

    condition = IfCondition(LaunchConfiguration('use_range_sensor'))
    action = Node(executable='range_sensor_bridge_node',
                  package='hippo_gz_plugins',
                  name='range_sensor',
                  namespace=LaunchConfiguration('vehicle_name'),
                  condition=condition)
    launch_description.add_action(action)

    ############################################################################
    # Spawn BlueROV
    ############################################################################
    path = str(package_path / 'launch/spawn_vehicle.launch.py')
    source = PythonLaunchDescriptionSource(path)
    path = str(package_path / 'models/bluerov/urdf/bluerov.xacro')
    args = {
        'model_path': path,
        'use_front_camera': LaunchConfiguration('use_front_camera'),
        'use_vertical_camera': LaunchConfiguration('use_vertical_camera'),
        'use_range_sensor': LaunchConfiguration('use_range_sensor'),
    }
    action = IncludeLaunchDescription(source, launch_arguments=args.items())
    launch_description.add_action(action)
    return launch_description
