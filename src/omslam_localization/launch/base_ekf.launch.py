"""Non-actuating default base plus EKF; activation is always explicit."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    backend = get('backend')
    if backend not in ('mock', 'real'):
        raise ValueError('backend must be mock or real')
    if backend == 'real' and get('imu_source') == 'mock':
        raise ValueError('Synthetic IMU source cannot be used with real motor backend')
    if backend == 'real' and get('mounting_verified').lower() != 'true':
        raise ValueError('Real EKF requires verified IMU mounting; specify the measured TF/profile')
    base = Path(get_package_share_directory('omslam_base'))
    local = Path(get_package_share_directory('omslam_localization'))
    local_args = {name: get(name) for name in ('use_sim_time', 'imu_config', 'ekf_config',
                  'base_link_z', 'imu_x', 'imu_y', 'imu_z', 'imu_roll', 'imu_pitch', 'imu_yaw',
                  'imu_port', 'imu_baud', 'mounting_verified', 'mock_yaw_rate')}
    local_args['imu_source'] = get('imu_source')
    local_args['publish_static_tfs'] = get('publish_static_tfs') or ('true' if backend == 'mock' else 'false')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(base/'launch/base.launch.py')),
                                 launch_arguments={'backend': 'serial' if backend == 'real' else 'mock', 'config': get('base_config'), 'use_sim_time': get('use_sim_time')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(local/'launch/localization.launch.py')),
                                 launch_arguments=local_args.items()),
    ]


def generate_launch_description():
    base = Path(get_package_share_directory('omslam_base'))
    local = Path(get_package_share_directory('omslam_localization'))
    defaults = {'backend': 'mock', 'base_config': str(base/'config/base.yaml'), 'use_sim_time': 'false',
                'imu_source': 'external', 'imu_config': str(local/'config/imu.yaml'), 'ekf_config': str(local/'config/ekf.yaml'),
                'publish_static_tfs': '', 'base_link_z': '0.101', 'imu_x': '0.0', 'imu_y': '0.0', 'imu_z': '0.0',
                'imu_roll': '0.0', 'imu_pitch': '0.0', 'imu_yaw': '0.0', 'imu_port': '', 'imu_baud': '921600',
                'mounting_verified': 'false', 'mock_yaw_rate': '0.0'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] + [OpaqueFunction(function=setup)])
