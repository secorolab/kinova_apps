from launch.actions import OpaqueFunction
from simple_launch import SimpleLauncher, get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    sl = SimpleLauncher()

    # Declare arguments
    sl.declare_arg('robot_ip', '192.168.1.12', description='IP address of the Kinova robot')
    sl.declare_arg('prefix', 'kinova_', description='Prefix for joint names, useful for multi-robot setup')

    sl.declare_arg('color_resolution',
                    #  '320x240',
                    #  '640x480',
                    '1280x720',
                    #  '1920x1080',
                   description='Resolution for color calibration')
    sl.declare_arg('depth_resolution', 
                    #  '424x240',
                    '480x270',
                   description='Resolution for depth calibration')
    sl.declare_arg('depth_registration', 'true',
                   description='Enable depth registration')

    calibration_dir = get_package_share_directory('kinova_vision') + '/launch/calibration'

    color_calib = calibration_dir + '/default_color_calib_' + sl.arg('color_resolution') + '.ini'
    depth_calib = calibration_dir + '/default_depth_calib_' + sl.arg('depth_resolution') + '.ini'

    kv = sl.include('kinova_vision', 'kinova_vision.launch.py',
        launch_arguments={
            'device': sl.arg('robot_ip'),
            'camera_link_frame_id': sl.arg('prefix') + 'camera_link',
            'color_frame_id': sl.arg('prefix') + 'camera_color_frame',
            'depth_frame_id': sl.arg('prefix') + 'camera_depth_frame',
            'depth_registration': sl.arg('depth_registration'),
            'color_camera_info_url': 'file://' + color_calib,
            'depth_camera_info_url': 'file://' + depth_calib,
        })



    return sl.launch_description()

