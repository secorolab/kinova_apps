from simple_launch import SimpleLauncher, get_package_share_directory

def generate_launch_description():
    sl = SimpleLauncher()

    # Declare arguments
    sl.declare_arg('robot_ip', '192.168.1.12', description='IP address of the Kinova robot')
    sl.declare_arg('color_resolution', '1920x1080',
                   description='Resolution for color calibration')
    sl.declare_arg('depth_resolution', '480x270',
                   description='Resolution for depth calibration')
    sl.declare_arg('depth_registration', 'true',
                   description='Enable depth registration')

    calibration_dir = get_package_share_directory('kinova_vision') + '/launch/calibration'

    color_calib = calibration_dir + '/default_color_calib_' + sl.arg('color_resolution') + '.ini'
    depth_calib = calibration_dir + '/default_depth_calib_' + sl.arg('depth_resolution') + '.ini'

    sl.include('kinova_vision', 'kinova_vision.launch.py',
               launch_arguments={
                   'device': sl.arg('robot_ip'),
                   'depth_registration': sl.arg('depth_registration'),
                   # Prepend “file://” as literal string
                   'color_camera_info_url': 'file://' + color_calib,
                   'depth_camera_info_url': 'file://' + depth_calib,
               })

    return sl.launch_description()

