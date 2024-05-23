from simple_launch import GazeboBridge, SimpleLauncher

sl = SimpleLauncher(use_sim_time=True)
sl.declare_gazebo_axes(pitch=0.0)


def launch_setup():
    print('hello')
    sl.gz_launch('empty.sdf')
    sl.spawn_gz_model(
        name='hippocampus',
        model_file=sl.find('hippo_sim', 'gripper.urdf'),
        spawn_args=sl.gazebo_axes_args(),
    )
    print('hello2')

    bridges = [
        GazeboBridge.clock(),
    ]
    print('hello 2.05')
    # gz_topic = GazeboBridge.model_prefix('test')
    gz_topic = '/world/empty/model/hippocampus'
    print('hello 2.1')
    bridges.append(
        GazeboBridge(
            gz_topic,
            'joint_states',
            'sensor_msgs/JointState',
            GazeboBridge.gz2ros,
        )
    )
    print('hello 2.5')
    joints = [
        'joint_5',
        'left_prismatic_revolute_joint',
        'left_revolute_joint',
    ]
    print('hello3')
    for i, joint in enumerate(joints):
        print(i)
        bridges.append(
            GazeboBridge(
                f'/model/hippocampus/joint/{joint}/cmd_force',
                f'joint{i+1}_cmd_effort',
                'std_msgs/Float64',
                GazeboBridge.ros2gz,
            )
        )

    sl.create_gz_bridge(bridges)
    sl.node(
        'slider_publisher', arguments=[sl.find('hippo_sim', 'gripper.yaml')]
    )
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
