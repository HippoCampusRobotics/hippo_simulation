import math

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    hippo_sim_path = get_package_share_path('hippo_sim')
    apriltag_path = str(hippo_sim_path / 'models/apriltag/urdf/apriltag.xacro')

    tmp = -math.pi / 2.0
    poses = [
        {
            'xyz': (0.7, 3.8, -0.5),
            'rpy': (tmp, 0.0, 0.0),
            'tag_id': 0,
        },
        {
            'xyz': (1.3, 3.8, -0.5),
            'rpy': (tmp, 0.0, 0.0),
            'tag_id': 1,
        },
        {
            'xyz': (0.7, 3.8, -0.9),
            'rpy': (tmp, 0.0, 0.0),
            'tag_id': 2,
        },
        {
            'xyz': (1.3, 3.8, -0.9),
            'rpy': (tmp, 0.0, 0.0),
            'tag_id': 3,
        },
    ]

    tag_size = 0.075
    nodes = []
    for i, p in enumerate(poses):
        tag_id = p['tag_id']
        mappings = [
            f'size_x={tag_size}',
            f'size_y={tag_size}',
            'size_z=0.01',
            f'tag_id={tag_id}',
        ]
        mappings = ' '.join(mappings)
        description = LaunchConfiguration(
            f'description_{i}',
            default=Command(
                [
                    'ros2 run hippo_sim create_robot_description.py ',
                    '--input ',
                    apriltag_path,
                    ' --mappings ',
                    mappings,
                ]
            ),
        )
        nodes.append(
            Node(
                package='hippo_sim',
                executable='spawn',
                parameters=[
                    {
                        'robot_description': description,
                    }
                ],
                arguments=[
                    '--param',
                    'robot_description',
                    '--name',
                    f'range_tag_{p["tag_id"]:02d}',
                    '--x',
                    f'{p["xyz"][0]}',
                    '--y',
                    f'{p["xyz"][1]}',
                    '--z',
                    f'{p["xyz"][2]}',
                    '--R',
                    f'{p["rpy"][0]}',
                    '--P',
                    f'{p["rpy"][1]}',
                    '--Y',
                    f'{p["rpy"][2]}',
                ],
                output='screen',
            )
        )
    return LaunchDescription(nodes)
