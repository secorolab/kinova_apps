from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()

    sl.declare_arg('xacro_file', default_value='pick_place.xacro', description='Path to the xacro file to load')
    sl.declare_arg('robot_ip', '192.168.1.12', description='IP address of the Kinova robot')
    sl.declare_arg('prefix', '', description='Prefix for joint names, useful for multi-robot setup')
    sl.declare_arg('gripper', 'robotiq_2f_85', description='Type of gripper: robotiq_2f_85 or none')


    # Launch the robot state publisher with the xacro file
    sl.robot_state_publisher('kinova_apps', sl.arg('xacro_file'), 'urdf', xacro_args={
        'robot_ip': sl.arg('robot_ip'),
        'prefix': sl.arg('prefix'),
        'gripper': sl.arg('gripper'),
        'gripper_joint_name': 'robotiq_85_left_knuckle_joint'
    })

    return sl.launch_description()
