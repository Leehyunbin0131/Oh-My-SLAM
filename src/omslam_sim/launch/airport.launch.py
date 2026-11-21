"""Run the installed airport environment, optionally with walking pedestrians."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _gazebo_exited(event, context):
    if not context.is_shutdown:
        return [EmitEvent(event=Shutdown(reason='Airport simulator exited'))]
    return []


def _launch(context):
    share = Path(get_package_share_directory('omslam_sim'))
    prefix = Path(get_package_prefix('omslam_sim'))
    world_name = LaunchConfiguration('world').perform(context)
    if world_name not in ('airport', 'p4_course'):
        raise ValueError('world must be airport or p4_course')
    world = share / 'worlds' / (('airport.sdf' if _bool(context, 'pedestrians')
                                else 'airport_static.sdf') if world_name == 'airport'
                               else 'p4_course.sdf')
    if not world.is_file():
        raise RuntimeError(f'Airport world was not installed: {world}')
    resource_paths = [str(share / 'models'), str(share)]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resource_paths.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    plugin_paths = [str(prefix / 'lib')]
    if os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH'):
        plugin_paths.append(os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'])
    arguments = [
        str(prefix / 'lib' / 'omslam_sim' / 'gazebo_runner'),
        '--world', world_name,
        'gz', 'sim', '-r', '-v', LaunchConfiguration('verbosity').perform(context),
        '--headless-rendering', '--render-engine-gui',
        LaunchConfiguration('gui_renderer').perform(context), str(world),
    ]
    if not _bool(context, 'gui'):
        arguments.append('-s')
    else:
        renderer = LaunchConfiguration('gui_renderer').perform(context)
        arguments.extend(['--gui-config', str(share / 'config' /
                                              f'airport_gui_{renderer}.config')])
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(resource_paths)),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', ':'.join(plugin_paths)),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('QT_AUTO_SCREEN_SCALE_FACTOR', '0'),
        ExecuteProcess(
            cmd=arguments, output='screen', sigterm_timeout='16', sigkill_timeout='3',
            on_exit=_gazebo_exited,
        ),
    ]
    if _bool(context, 'bridge'):
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='airport_clock_bridge', output='screen',
            arguments=[f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            remappings=[(f'/world/{world_name}/clock', '/clock')],
            parameters=[{'use_sim_time': True}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='airport', choices=['airport', 'p4_course']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('pedestrians', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('bridge', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gui_renderer', default_value='ogre', choices=['ogre', 'ogre2']),
        DeclareLaunchArgument('verbosity', default_value='2', choices=['1', '2', '3', '4']),
        OpaqueFunction(function=_launch),
    ])
