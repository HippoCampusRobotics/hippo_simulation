from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    launch_path = str(package_path / 'launch/spawn_vehicle.launch.py'),
    model_path = str(package_path / 'models/bluerov/urdf/bluerov.xacro')
    model_path_rviz = str(package_path / 'models/bluerov/urdf/bluerov_rviz.xacro')

    default_vehicle_name = "bluerov"

    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value = default_vehicle_name,
        description = 'Vehicle name used as namespace'
    )
    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time_launch_arg',
        default_value = str(True),
        description = 'Vehicle name used as namespace'
    )

    vehicle_spawner = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch_path),
        launch_arguments=dict(use_sim_time=use_sim_time,
            vehicle_name=vehicle_name,
            model_path=model_path,
            fake_state_estimation=str(True)).items())
    state_publisher = launch_ros.actions.Node(package='robot_state_publisher',
                                              executable='robot_state_publisher',
                                              name='robot_state_publisher',
                                              namespace=vehicle_name,
                                              output='screen',
                                              parameters=[{'use_sim_time' : use_sim_time,
                                                           'robot_description': launch_ros.descriptions.ParameterValue(
                               launch.substitutions.Command(['xacro ', model_path_rviz, " vehicle_name:=", vehicle_name]), value_type=str)}]) # pi: 3.14159265359

    return launch.LaunchDescription([
            use_sim_time_launch_arg,
            vehicle_name_launch_arg,
            vehicle_spawner,
        state_publisher])
