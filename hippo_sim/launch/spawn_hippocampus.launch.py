from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from hippo_common import launch_helper
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def declare_args(launch_description: LaunchDescription):
    action = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(action)
    action = DeclareLaunchArgument('use_sim_time')
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    package_path = get_package_share_path('hippo_sim')

    declare_args(launch_description=launch_description)

    ############################################################################
    # Camera Bridge
    ############################################################################
    action = launch_helper.create_camera_bridge(
        vehicle_name=LaunchConfiguration('vehicle_name'),
        camera_name='vertical_camera',
        image_name='image_rect')
    launch_description.add_action(action)

    ############################################################################
    # Spawn HippoCampus
    ############################################################################
    path = str(package_path / 'launch/spawn_vehicle.launch.py')
    source = PythonLaunchDescriptionSource(path)
    path = str(package_path / 'models/hippo3/urdf/hippo3.xacro')
    action = IncludeLaunchDescription(
        source, launch_arguments=dict(model_path=path).items())
    launch_description.add_action(action)

    return launch_description
