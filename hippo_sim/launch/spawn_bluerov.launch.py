from ament_index_python.packages import get_package_share_path
import launch


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    launch_path = str(package_path / 'launch/spawn_vehicle.launch.py'),
    model_path = str(package_path / 'models/bluerov/urdf/bluerov.xacro')

    default_vehicle_name = "bluerov"

    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value=default_vehicle_name,
        description='Vehicle name used as namespace'
    )
    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time_launch_arg',
        default_value=str(True),
        description='Vehicle name used as namespace'
    )

    vehicle_spawner = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch_path),
        launch_arguments=dict(use_sim_time=use_sim_time,
            vehicle_name=vehicle_name,
            model_path=model_path,
            fake_state_estimation=str(True)).items())

    return launch.LaunchDescription([
            use_sim_time_launch_arg,
            vehicle_name_launch_arg,
            vehicle_spawner])
