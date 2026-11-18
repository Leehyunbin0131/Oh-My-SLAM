from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default_config=str(Path(get_package_share_directory('omslam_base'))/'config/base.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('backend',default_value='mock'),
        DeclareLaunchArgument('config',default_value=default_config),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        LifecycleNode(package='omslam_base',executable='base_bridge',name='base_bridge',namespace='',output='screen',
                      parameters=[LaunchConfiguration('config'),{'backend':LaunchConfiguration('backend'),
                       'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}])])
