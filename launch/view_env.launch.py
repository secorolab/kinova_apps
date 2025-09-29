from simple_launch import SimpleLauncher
from simple_launch import get_package_share_directory


def generate_launch_description():

    sl = SimpleLauncher()

    sl.declare_arg('xacro_file', 'pick_place.xacro', description='Path to the xacro file to load')
    sl.declare_arg('joint_state_gui', 'true', description='Enable joint state publisher GUI')

    # laod env
    sl.include('kinova_apps', 'load_env.launch.py', launch_arguments={
        'xacro_file': sl.arg('xacro_file'),
    })

    # joint state publisher
    sl.joint_state_publisher(use_gui=sl.arg('joint_state_gui'))

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
