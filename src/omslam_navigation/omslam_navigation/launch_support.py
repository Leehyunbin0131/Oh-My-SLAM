"""Minimal launch composition for Gazebo + SLAM/AMCL + standard Nav2."""

import atexit
import copy
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def merge_parameters(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_parameters(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _read_yaml(path):
    value = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'Expected a YAML mapping: {path}')
    return value


def load_configuration(share, override=None):
    share = Path(share)
    config = merge_parameters(
        _read_yaml(share / 'config/nav2_common.yaml'),
        _read_yaml(share / 'config/sim_profile.yaml'),
    )
    if override:
        config = merge_parameters(config, _read_yaml(override))
    planner = config['planner_server']['ros__parameters']['GridBased']
    lattice = Path(planner['lattice_filepath'])
    if not lattice.is_absolute():
        lattice = (share / lattice).resolve(strict=True)
    planner['lattice_filepath'] = str(lattice)
    bt = config['bt_navigator']['ros__parameters']
    tree = Path(bt['default_nav_to_pose_bt_xml'])
    if not tree.is_absolute():
        tree = (share / tree).resolve(strict=True)
    bt['default_nav_to_pose_bt_xml'] = str(tree)
    return config


def _include(path, arguments):
    return GroupAction(scoped=True, forwarding=False, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())])


def _start(context, mode):
    get = lambda key: LaunchConfiguration(key).perform(context)
    boolean = lambda name: get(name).lower() == 'true'
    share = Path(get_package_share_directory('omslam_navigation'))
    mapping_share = Path(get_package_share_directory('omslam_mapping'))
    config = load_configuration(share, get('nav2_params_file') or None)
    backend_mode = 'mapping' if mode == 'slam' else 'localization'
    backend_params = Path(get('backend_params_file')) if get('backend_params_file') else (
        mapping_share / 'config' / f'{backend_mode}.yaml')
    map_path = ''
    if mode == 'amcl':
        candidate = Path(get('map')).expanduser()
        if not candidate.is_file():
            raise ValueError('AMCL requires map:=/absolute/path/to/map.yaml')
        map_path = str(candidate.resolve())

    actions = []
    if boolean('start_sim'):
        sim = Path(get_package_share_directory('omslam_sim'))
        actions.append(_include(sim / 'launch/sim.launch.py', {
            'world': get('world'),
            'gui': str(boolean('gui')).lower(),
            'pedestrians': str(boolean('pedestrians')).lower(),
            'gui_renderer': get('gui_renderer'),
            'verbosity': get('verbosity'),
            'x': get('spawn_x'),
            'y': get('spawn_y'),
            'yaw': get('spawn_yaw'),
            'autostart_base': 'true',
        }))
    actions.append(_include(mapping_share / 'launch' / f'{backend_mode}.launch.py', {
        'use_sim_time': 'true',
        'params_file': str(backend_params.resolve()),
        'map': map_path,
        'initial_pose': str(boolean('initial_pose')).lower(),
        'x': get('initial_x'),
        'y': get('initial_y'),
        'yaw': get('initial_yaw'),
    }))
    with tempfile.NamedTemporaryFile(mode='w', prefix='omslam-mini-nav-', suffix='.yaml',
                                     encoding='utf-8', delete=False) as output:
        yaml.safe_dump(config, output, sort_keys=False)
        parameter_path = Path(output.name)
    atexit.register(parameter_path.unlink, missing_ok=True)

    for package, executable, name, remappings in [
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         [('cmd_vel', '/cmd_vel/requested'), ('cmd_vel_smoothed', '/cmd_vel')]),
        ('nav2_planner', 'planner_server', 'planner_server', []),
        ('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', '/cmd_vel/requested')]),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', [('cmd_vel', '/cmd_vel/requested')]),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
        ('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation', []),
    ]:
        actions.append(Node(
            package=package, executable=executable, name=name, output='screen',
            parameters=[str(parameter_path)], remappings=remappings, respawn=False,
        ))
    return actions


def make_launch_description(mode):
    if mode not in ('slam', 'amcl'):
        raise ValueError('Navigation mode must be slam or amcl')
    arguments = {
        'start_sim': ('true', 'Start Gazebo and robot; false reuses an existing simulator'),
        'world': ('p4_course', 'Simulator world: p4_course or airport'),
        'gui': ('true', 'Show Gazebo GUI'),
        'pedestrians': ('false', 'Enable airport pedestrian traffic'),
        'gui_renderer': ('ogre', 'Gazebo GUI renderer'),
        'verbosity': ('2', 'Gazebo verbosity'),
        'spawn_x': ('', 'Initial world X, defaults to selected world spawn'),
        'spawn_y': ('0.0', 'Initial world Y'),
        'spawn_yaw': ('0.0', 'Initial world yaw'),
        'map': ('', 'Saved map YAML path for AMCL mode'),
        'initial_pose': ('false', 'Seed AMCL pose from initial_x/y/yaw'),
        'initial_x': ('0.0', 'AMCL initial X'),
        'initial_y': ('0.0', 'AMCL initial Y'),
        'initial_yaw': ('0.0', 'AMCL initial yaw'),
        'nav2_params_file': ('', 'Optional Nav2 parameter override YAML'),
        'backend_params_file': ('', 'Optional SLAM or AMCL parameter YAML'),
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, description) in arguments.items()),
        OpaqueFunction(function=_start, args=[mode]),
    ])
