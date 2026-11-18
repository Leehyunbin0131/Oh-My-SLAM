from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    def console(context):
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
        return [Node(package='omslam_operator', executable='console', output='screen',
                     arguments=['--use-sim-time'] if use_sim_time else ['--real-time', '--read-only'])]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=console)])
