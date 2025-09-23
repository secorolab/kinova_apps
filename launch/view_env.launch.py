from simple_launch import SimpleLauncher
from simple_launch import get_package_share_directory


def generate_launch_description():

    sl = SimpleLauncher()

    sl.declare_arg('xacro_file', default_value='pick_place.xacro', description='Path to the xacro file to load')

    # Launch the robot state publisher with the xacro file
    sl.robot_state_publisher('kinova_apps', sl.arg('xacro_file'), 'urdf')

    # joint state publisher
    sl.joint_state_publisher(use_gui=True)

    # Launch RViz2 with a predefined configuration file
    rviz_config_file = get_package_share_directory('kinova_apps') + '/config/view_env.rviz'
    sl.node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return sl.launch_description()
