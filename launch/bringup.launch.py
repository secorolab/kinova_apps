from simple_launch import SimpleLauncher, get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


sl = SimpleLauncher()

# Declare arguments
sl.declare_arg('xacro_file', default_value='pick_place.xacro', description='Path to the xacro file to load')

sl.declare_arg('robot_ip', '192.168.1.12', description='IP address of the Kinova robot')
sl.declare_arg('prefix', 'kinova_', description='Prefix for joint names, useful for multi-robot setup')
sl.declare_arg('gripper', 'robotiq_2f_85', description='Type of gripper: robotiq_2f_85 or none')
sl.declare_arg('vision_bringup', True, description='Whether to launch vision bringup')

sl.declare_arg('use_rviz', True, description='Whether to launch RViz')
sl.declare_arg('rviz_config_file', default_value=get_package_share_directory('kinova_apps') + '/config/rviz/default.rviz',
                description='Path to the RViz config file to use')

sl.declare_arg('ros2_controllers_file', default_value=get_package_share_directory('kinova_apps') + '/config/ros2_controllers.yaml',
                description='Path to the ROS2 controllers file')

sl.declare_arg('moveit_configs_pkg', None, description='MoveIt configs package to use')


def launch_setup():

    rviz_config_file = sl.arg('rviz_config_file')

    # robot description
    sl.include('kinova_apps', 'load_env.launch.py', 
        launch_arguments={
            'xacro_file': sl.arg('xacro_file'),
            'prefix': sl.arg('prefix'),
            'gripper': sl.arg('gripper'),
            'robot_ip': sl.arg('robot_ip'),
        }
    )

    # ---------------------- ros2 control --------------------------

    # ros2_control_node
    sl.node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            sl.arg('ros2_controllers_file')
        ],
        remappings=[
            ('~/robot_description', '/robot_description')
        ],
        output='screen'
    )

    # spawner for joint_trajectory_controller
    sl.node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
        output='screen'
    )

    # spawner for gripper controller
    if sl.arg('gripper') == 'robotiq_2f_85':
        sl.node(
            package='controller_manager',
            executable='spawner',
            arguments=['robotiq_gripper_controller', '-c', '/controller_manager'],
            output='screen'
        )

    # joint state_broadcaster
    sl.node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )

    # fault controller - required for robotiq gripper connected via internal interface
    sl.node(
        package='controller_manager',
        executable='spawner',
        arguments=['fault_controller', '-c', '/controller_manager'],
        output='screen'
    )   

    # -----------------------  vision ----------------------------

    # vision bringup
    if sl.arg('vision_bringup'):
        sl.include('kinova_apps', 'vision_bringup.launch.py', 
            launch_arguments={
                'robot_ip': sl.arg('robot_ip'),
                'prefix': sl.arg('prefix'),
            }
        )

    # -----------------------  moveit ----------------------------

    if sl.arg('moveit_configs_pkg'):
        moveit_pkg = sl.arg('moveit_configs_pkg')

        # moveit
        moveit_config = (
            MoveItConfigsBuilder("gen3_2f_85_pick_place", package_name=moveit_pkg)
            .planning_scene_monitor(
                publish_robot_description=True, publish_robot_description_semantic=True
            )
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
            .to_moveit_configs()
        )

        sl.node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()]
        )

        rviz_moveit_params = [
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ]

        rviz_config_file = get_package_share_directory(moveit_pkg) + "/config/moveit.rviz"

    # -----------------------  rviz ----------------------------

    # rviz
    if sl.arg('use_rviz'):
        sl.node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            # the below parameters are needed to show the moveit markers
            parameters=rviz_moveit_params if sl.arg('moveit_configs_pkg') else []
        )

    return sl.launch_description()

generate_launch_description = sl.launch_description(opaque_function=launch_setup)

