"""Wheel twist + one IMU gyro EKF. No lidar or map-frame feedback input."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    boolean = lambda name: get(name).lower() == 'true'
    use_sim_time = boolean('use_sim_time')
    source = get('imu_source')
    if source not in ('external', 'mock', 'hfi_a9'):
        raise ValueError('imu_source must be external, mock or hfi_a9')
    if source == 'hfi_a9' and (use_sim_time or not boolean('mounting_verified') or not get('imu_port')):
        raise ValueError('Serial IMU requires wall time, explicit port, and verified physical mounting')
    actions = [
        Node(package='omslam_localization', executable='imu_conditioner', name='imu_conditioner',
             parameters=[get('imu_config'), {'use_sim_time': use_sim_time}], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[get('ekf_config'), {'use_sim_time': use_sim_time}],
             remappings=[('odometry/filtered', '/odom')], output='screen'),
    ]
    if boolean('publish_static_tfs'):
        actions += [
            Node(package='tf2_ros', executable='static_transform_publisher', name='base_mount_tf',
                 arguments=['--z', get('base_link_z'), '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='tf2_ros', executable='static_transform_publisher', name='imu_mount_tf',
                 arguments=['--x', get('imu_x'), '--y', get('imu_y'), '--z', get('imu_z'),
                            '--roll', get('imu_roll'), '--pitch', get('imu_pitch'), '--yaw', get('imu_yaw'),
                            '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
                 parameters=[{'use_sim_time': use_sim_time}]),
        ]
    if source == 'mock':
        actions.append(Node(package='omslam_localization', executable='mock_imu', name='mock_imu',
                            parameters=[{'use_sim_time': use_sim_time, 'yaw_rate': float(get('mock_yaw_rate'))}], output='screen'))
    elif source == 'hfi_a9':
        actions.append(Node(package='omslam_localization', executable='hfi_a9_node', name='hfi_a9',
                            parameters=[{'port': get('imu_port'), 'baud': int(get('imu_baud')),
                                         'mounting_verified': True, 'use_sim_time': False}], output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('omslam_localization'))
    defaults = {'use_sim_time': 'false', 'imu_source': 'external',
                'imu_config': str(share/'config/imu.yaml'), 'ekf_config': str(share/'config/ekf.yaml'),
                'publish_static_tfs': 'false', 'base_link_z': '0.101',
                'imu_x': '0.0', 'imu_y': '0.0', 'imu_z': '0.0',
                'imu_roll': '0.0', 'imu_pitch': '0.0', 'imu_yaw': '0.0',
                'imu_port': '', 'imu_baud': '921600', 'mounting_verified': 'false', 'mock_yaw_rate': '0.0'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] + [OpaqueFunction(function=setup)])
