from ament_index_python.packages import get_package_share_path
import launch
import launch_ros

def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    spawner_path = str(package_path / "launch/spawn_bluerov.launch.py")
    low_level_ctrl_package_path = get_package_share_path('hippo_control')
    mixer_path = str(low_level_ctrl_package_path / "launch/node_actuator_mixer.launch.py")
    mixer_config_file_path = str(low_level_ctrl_package_path /('config/actuator_mixer_bluerov_default.yaml'))
    ctrl_package_path = get_package_share_path('bluerov_ctrl')
    ctrl_path = str(ctrl_package_path / "launch/node_control.launch.py")
    estimation_package_path = get_package_share_path('bluerov_estimation')
    estimation_path = str(estimation_package_path / "launch/estimation.launch.py")
    traj_gen_path = str(ctrl_package_path / "launch/node_trajectory_gen_eight.launch.py")
    visualization_package_path = get_package_share_path('alpha_visualization')
    visualization_path = str(visualization_package_path / "launch/pose_visualization.launch.py")
    rviz_path = str(package_path / "launch/rviz.launch.py")

    vehicle_name = 'bluerov'
    use_sim_time = True
    start_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(package_path / "launch/start_gazebo.launch.py")
        )
    )
    paths = [spawner_path, traj_gen_path, visualization_path]
    launch_files = []

    for path in paths:

        launch_files.append(launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
            path),
            launch_arguments=dict(use_sim_time=str(use_sim_time),
                                  vehicle_name=vehicle_name).items()
            )
        )
    launch_files.append(launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ctrl_path
        ),
        launch_arguments=dict(use_sim_time=str(use_sim_time),
                              controller_type=str(1)).items()
        )
    )

    launch_files.append(launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                rviz_path
            ),
        launch_arguments=dict(use_sim_time=str(use_sim_time),
                              ).items()
        )
    )
    launch_files.append(launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            mixer_path),
        launch_arguments=dict(use_sim_time=str(use_sim_time),
                              vehicle_name=vehicle_name,
                              mixer_path=mixer_config_file_path).items()
    )
    )

    launch_files.append(launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            estimation_path
        ),
        launch_arguments=dict(use_sim_time=str(use_sim_time),
                              vehicle_name=vehicle_name).items()
    ))

    launch_files.append(start_gazebo)
    return launch.LaunchDescription([
        *launch_files,
        #tf_map_body_publisher

    ])


