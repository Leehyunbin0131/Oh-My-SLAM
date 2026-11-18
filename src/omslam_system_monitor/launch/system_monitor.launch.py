from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host_role', default_value='host'),
        DeclareLaunchArgument('disk_path', default_value='/'),
        Node(package='omslam_system_monitor', executable='system_monitor', name='system_monitor',
             parameters=[{'host_role': LaunchConfiguration('host_role'),
                          'disk_path': LaunchConfiguration('disk_path'), 'use_sim_time': False}],
             output='screen'),
    ])
