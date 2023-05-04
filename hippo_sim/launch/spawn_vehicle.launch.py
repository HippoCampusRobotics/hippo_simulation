from ament_index_python.packages import get_package_share_path
import launch
import launch_ros
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    default_model_path = package_path / 'models/hippo3/urdf/hippo3.xacro'
    default_vehicle_name = 'uuv00'
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    vehicle_name = LaunchConfiguration('vehicle_name')

    model_launch_arg = DeclareLaunchArgument(
        name='model_path',
        default_value=str(default_model_path),
        description='Absolute model path')
    vehicle_name_launch_arg = DeclareLaunchArgument(
        name='vehicle_name',
        default_value=default_vehicle_name,
        description='Vehicle name used as namespace.')
    fake_estimator_launch_arg = DeclareLaunchArgument(
        name='fake_state_estimation', default_value='false')
    fake_vision_launch_arg = DeclareLaunchArgument(name='fake_vision',
                                                   default_value='false')

    robot_description = LaunchConfiguration(
        'robot_description',
        default=Command([
            'ros2 run hippo_sim create_robot_description.py ', '--input ',
            LaunchConfiguration('model_path'), ' --mappings vehicle_name=',
            LaunchConfiguration('vehicle_name')
        ]))

    description = {'robot_description': robot_description}

    vehicle_group = GroupAction(actions=[
        PushRosNamespace(vehicle_name),
        Node(package='hippo_sim',
             executable='spawn',
             parameters=[description],
             arguments=[
                 '--param',
                 'robot_description',
                 '--remove_on_exit',
                 'true',
                 '--x',
                 '1.3',
                 '--y',
                 '1.0',
                 '--z',
                 '-0.5',
                 '--Y',
                 '1.5708',
             ],
             output='screen'),
        Node(package='hippo_sim',
             executable='bridge',
             parameters=[{
                 'use_sim_time': use_sim_time
             }],
             output='screen'),
        Node(package='hippo_sim',
             executable='fake_state_estimator',
             name='state_estimator',
             parameters=[
                 {
                     'use_sim_time': use_sim_time,
                 },
             ],
             output='screen',
             condition=IfCondition(LaunchConfiguration(
                 'fake_state_estimation'))),
        Node(package='hippo_sim',
    """
             executable="fake_vision",
             name="vision",
             parameters=[
                 {
                     'use_sim_time': use_sim_time,
                 },
             ],
             condition=IfCondition(LaunchConfiguration('fake_vision'))),
        Node(package='state_estimation',
             executable='estimator',
             name='state_estimator',
             parameters=[
                 {
                     'use_sim_time': use_sim_time,
                 },
             ],
             condition=UnlessCondition(
                 LaunchConfiguration('fake_state_estimation'))),

    launch_path = str(
        get_package_share_path('hippo_common') /
        'launch/tf_publisher_hippo.launch.py')
    launch_source = PythonLaunchDescriptionSource(launch_path)
    tf_publisher_vehicle = IncludeLaunchDescription(
        launch_source,
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return launch.LaunchDescription([
        model_launch_arg,
        vehicle_name_launch_arg,
        fake_estimator_launch_arg,
        fake_vision_launch_arg,
        tf_publisher_vehicle,
        vehicle_group,
    ])
