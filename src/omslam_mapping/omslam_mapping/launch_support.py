"""Launch exactly one global localization backend; odometry runs separately."""

import atexit
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from omslam_mapping.mode_guard import ModeGuard


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name} must be true or false')
    return value == 'true'


def _start(context, fixed_mode):
    mode = fixed_mode or LaunchConfiguration('mode').perform(context)
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    share = Path(get_package_share_directory('omslam_mapping'))
    supplied_params = LaunchConfiguration('params_file').perform(context)
    params = Path(supplied_params) if supplied_params else share / 'config' / f'{mode}.yaml'
    if not params.is_file():
        raise ValueError(f'Parameter file does not exist: {params}')
    use_sim_time = _boolean(context, 'use_sim_time')
    if mode == 'localization':
        map_name = LaunchConfiguration('map').perform(context)
        if not map_name or not Path(map_name).is_file():
            raise ValueError('localization requires map:=/absolute/path/to/map.yaml')
        set_initial_pose = _boolean(context, 'initial_pose')
        initial = {
            'use_sim_time': use_sim_time,
            'set_initial_pose': set_initial_pose,
            'initial_pose.x': float(LaunchConfiguration('x').perform(context)),
            'initial_pose.y': float(LaunchConfiguration('y').perform(context)),
            'initial_pose.yaw': float(LaunchConfiguration('yaw').perform(context)),
        }
        actions = [
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 output='screen', parameters=[str(params), {
                     'use_sim_time': use_sim_time, 'yaml_filename': str(Path(map_name).resolve())}]),
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                 parameters=[str(params), initial]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_map_localization', output='screen', parameters=[{
                     'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['map_server', 'amcl'], 'bond_timeout': 4.0}]),
        ]
    else:
        actions = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('slam_toolbox')) /
                                               'launch' / 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': str(params), 'use_sim_time': str(use_sim_time).lower(),
                'autostart': 'true', 'use_lifecycle_manager': 'false',
            }.items(),
        )]
    guard = ModeGuard(mode)
    # Keep ownership through child shutdown, until the launch process itself exits.
    atexit.register(guard.close)
    return actions


def make_launch_description(fixed_mode=None):
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Override the selected backend parameter YAML'),
        DeclareLaunchArgument('map', default_value='', description='Saved map YAML for AMCL'),
        DeclareLaunchArgument('initial_pose', default_value='false',
                              description='Seed AMCL from x/y/yaw; otherwise use /initialpose'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]
    if fixed_mode is None:
        arguments.insert(0, DeclareLaunchArgument('mode', default_value='mapping',
                                                 choices=['mapping', 'localization']))
    return LaunchDescription([*arguments, OpaqueFunction(function=_start, args=[fixed_mode])])
