from ament_index_python.packages import get_package_share_path
import launch
import launch_ros

def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    spawner_path = str(package_path / "launch/spawn_bluerov.launch.py")

    vehicle_name = 'klopsi00'
    use_sim_time = True
    start_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(package_path / "launch/start_gazebo.launch.py")
        )
    )
    launch_files = []

    launch_files.append(launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
            spawner_path),
            launch_arguments=dict(use_sim_time=str(use_sim_time),
                                  vehicle_name=vehicle_name).items()
            )
        )
    launch_files.append(start_gazebo)
    return launch.LaunchDescription([
        *launch_files,
    ])


