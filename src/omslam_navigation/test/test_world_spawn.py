"""Resolve the installed launch descriptions up to the Gazebo create arguments."""
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

from test_keepout_maps import map_bundle


def compose(package, filename, parameters):
    context = LaunchContext()
    context.launch_configurations.update(parameters)
    path = Path(get_package_share_directory(package)) / 'launch' / filename
    launch = PythonLaunchDescriptionSource(str(path)).get_launch_description(context)
    for action in launch.entities:
        if isinstance(action, OpaqueFunction):
            return action.execute(context), context
        action.execute(context)
    raise AssertionError(f'Missing setup action in {path}')


@pytest.mark.parametrize('world,x,expected', [('airport', None, -108.), ('p4_course', None, 0.),
                                             ('airport', '1.25', 1.25), ('p4_course', '-.5', -.5)])
@pytest.mark.parametrize('entry', ['sim.launch.py', 'slam_navigation.launch.py', 'navigation.launch.py'])
def test_world_selects_spawn_before_process_start(entry, world, x, expected, tmp_path):
    arguments = {'world': world}
    if entry == 'sim.launch.py':
        if x is not None:
            arguments['x'] = x
    else:
        if x is not None:
            arguments['spawn_x'] = x
        if entry == 'navigation.launch.py':
            arguments.update(map=str(map_bundle(tmp_path)), initial_pose='true', initial_x='42')
        navigation, _ = compose('omslam_navigation', entry, arguments)
        sim = next(action for action in navigation[0].get_sub_entities()
                   if isinstance(action, IncludeLaunchDescription))
        arguments = dict(sim.launch_arguments)
    simulator, context = compose('omslam_sim', 'sim.launch.py', arguments)
    for action in simulator:
        if isinstance(action, Node) and perform_substitutions(context, action.cmd[0]).endswith('/create'):
            args = [perform_substitutions(context, part) for part in action.cmd[1:-1]]
            assert args[args.index('-world') + 1] == world
            assert float(args[args.index('-x') + 1]) == expected
            return
    pytest.fail('Simulator did not include ros_gz_sim/create')
