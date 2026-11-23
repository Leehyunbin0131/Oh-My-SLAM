"""Compose the installed simulation stack and validate its navigation contract."""

import atexit
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml
from omslam_sim.control import SIM_MAX_LINEAR_VELOCITY

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FOOTPRINT = [[0.09, 0.26], [0.09, -0.26], [-0.39, -0.26], [-0.39, 0.26]]
PADDED_FOOTPRINT = [[0.11, 0.28], [0.11, -0.28], [-0.41, -0.28], [-0.41, 0.28]]
NAVIGATION_NODES = [
    'velocity_smoother', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
]
# Parameters that only the removed Smac fork declared. Stock Jazzy ignores an
# undeclared override, so accepting it would silently imply a planner contract.
UNSUPPORTED_PLANNER_PARAMETERS = ('xy_goal_tolerance',)
LATTICE_SHA256 = '105b37351d15e340cc2921fa8878c8ea727a65cd7e89c39c977be0654be3b2a9'


def merge_parameters(base, override):
    """Merge YAML mappings without losing nested costmap parameter namespaces."""
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


def _validate_override(base, override, prefix=''):
    # Additional Nav2 parameters can change topic/clear/timeout semantics. Keep
    # the override surface explicit rather than silently accepting unknown keys.
    for key, value in override.items():
        name = f'{prefix}.{key}' if prefix else key
        if key not in base:
            raise ValueError(f'Unsupported navigation override parameter: {name}')
        if isinstance(base[key], dict):
            if not isinstance(value, dict):
                raise ValueError(f'Expected a parameter mapping: {name}')
            _validate_override(base[key], value, name)
        elif isinstance(value, dict):
            raise ValueError(f'Expected a parameter value: {name}')


def _number(value, name, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    if minimum is not None and value < minimum:
        raise ValueError(f'{name} must be >= {minimum}')
    if maximum is not None and value > maximum:
        raise ValueError(f'{name} must be <= {maximum}')
    return value


def _required_equal(actual, expected, name):
    if actual != expected:
        raise ValueError(f'{name} violates the P4 contract: expected {expected!r}, got {actual!r}')


def map_image_path(map_path):
    path = Path(map_path)
    image = Path(_read_yaml(path).get('image', ''))
    return image if image.is_absolute() else path.parent / image


def keepout_file_for_map(map_path, allow_pending=False):
    """Return a validated optional sidecar bound to this exact map raster.

    Old maps may have no sidecar. A present but invalid sidecar must never be
    silently ignored, which would load a map without its intended restrictions.
    """
    from .keepouts import load_keepout_file
    path = Path(map_path)
    pending = path.with_suffix('.keepout.pending')
    if not allow_pending and (pending.exists() or pending.is_symlink()):
        raise ValueError('Map keepout save is incomplete; save a complete bundle before loading it')
    sidecar = path.with_suffix('.keepout.json')
    metadata_path = path.with_suffix('.metadata.json')
    if metadata_path.is_file():
        files = json.loads(metadata_path.read_text(encoding='utf-8')).get('files', {})
        if 'keepout.json' in files:
            if not sidecar.is_file():
                raise ValueError(f'Required keepout file is missing: {sidecar}')
            if hashlib.sha256(sidecar.read_bytes()).hexdigest() != files['keepout.json'].get('sha256'):
                raise ValueError(f'Keepout file hash does not match map metadata: {sidecar}')
    if not sidecar.exists() and not sidecar.is_symlink():
        return None
    load_keepout_file(sidecar)
    data = json.loads(sidecar.read_text(encoding='utf-8'))
    binding = data.get('map_sha256')
    if not isinstance(binding, dict):
        raise ValueError('Keepout file has no map hash binding')
    for name, actual in [('yaml', path), ('pgm', map_image_path(path))]:
        if not actual.is_file() or binding.get(name) != hashlib.sha256(actual.read_bytes()).hexdigest():
            raise ValueError(f'Keepout {name} hash does not match the selected map')
    return sidecar.resolve()


def validate_map_bundle(path, allow_pending_keepouts=False):
    """Require a simulation map with intact provenance before starting AMCL."""
    path = Path(path).expanduser()
    if not path.is_absolute() or not path.is_file():
        raise ValueError('map must be an existing absolute map YAML path')
    path = path.resolve()
    data = _read_yaml(path)
    _number(data.get('resolution'), 'map.resolution', 0.05, 0.05)
    origin = data.get('origin')
    if not isinstance(origin, list) or len(origin) != 3:
        raise ValueError('map.origin must contain x, y, yaw')
    for index, value in enumerate(origin):
        _number(value, f'map.origin[{index}]')
    image = Path(data.get('image', ''))
    if not image.is_absolute():
        image = path.parent / image
    if not image.is_file():
        raise ValueError(f'Map image does not exist: {image}')
    metadata_path = path.with_suffix('.metadata.json')
    if not metadata_path.is_file():
        raise ValueError(f'Simulation map provenance is required: {metadata_path}')
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    _required_equal(metadata.get('schema_version'), 1, 'map metadata schema')
    _required_equal(metadata.get('backend'), 'simulation', 'map backend')
    _required_equal(metadata.get('frames'), {
        'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_footprint',
    }, 'map frames')
    for name, actual in [('yaml', path), ('pgm', image)]:
        recorded = metadata.get('files', {}).get(name, {}).get('sha256')
        if recorded != hashlib.sha256(actual.read_bytes()).hexdigest():
            raise ValueError(f'Map {name} hash does not match its metadata: {actual}')
    keepout_file_for_map(path, allow_pending=allow_pending_keepouts)
    return path


def validate_configuration(configuration, share):
    """Reject overrides that break the selected plugins, geometry or command chain."""
    share = Path(share)
    c = configuration
    planner = c['planner_server']['ros__parameters']
    controller = c['controller_server']['ros__parameters']
    smoother = c['velocity_smoother']['ros__parameters']
    guard = c['navigation_guard']['ros__parameters']
    bt = c['bt_navigator']['ros__parameters']
    behavior = c['behavior_server']['ros__parameters']
    lifecycle = c['lifecycle_manager_navigation']['ros__parameters']
    for name in ('operator_enabled', 'manual_reverse_enabled', 'rear_observation_verified'):
        if type(guard[name]) is not bool:
            raise ValueError(f'guard.{name} must be a boolean')
    reverse = guard['manual_reverse_enabled']
    if reverse and not (guard['operator_enabled'] and guard['rear_observation_verified']):
        raise ValueError('manual reverse requires operator mode and a verified rear-observation profile')
    for name in NAVIGATION_NODES + ['navigation_guard', 'lifecycle_manager_navigation']:
        _required_equal(c[name]['ros__parameters']['use_sim_time'], True, name + '.use_sim_time')
    _required_equal(planner['planner_plugins'], ['GridBased'], 'planner_plugins')
    _required_equal(planner['GridBased']['plugin'], 'nav2_smac_planner::SmacPlannerLattice', 'planner plugin')
    _required_equal(planner['GridBased']['allow_unknown'], False, 'allow_unknown')
    _required_equal(planner['GridBased']['allow_reverse_expansion'], False, 'allow_reverse_expansion')
    for name in UNSUPPORTED_PLANNER_PARAMETERS:
        if name in planner['GridBased']:
            raise ValueError(f'GridBased.{name} is not a stock Jazzy Smac Lattice parameter')
    _required_equal(controller['controller_plugins'], ['FollowPath'], 'controller_plugins')
    _required_equal(controller['goal_checker_plugins'], ['goal_checker'], 'goal_checker_plugins')
    _required_equal(controller['progress_checker_plugins'], ['progress_checker'], 'progress_checker_plugins')
    dwb = controller['FollowPath']
    _required_equal(dwb['plugin'], 'dwb_core::DWBLocalPlanner', 'controller plugin')
    _required_equal(dwb['trajectory_generator_name'], 'omslam_navigation::SpeedAwareTrajectoryGenerator',
                    'DWB low-speed trajectory generator')
    cell_size = c['local_costmap']['local_costmap']['ros__parameters']['resolution']
    dwb['minimum_projection_distance'] = 2.0 * _number(cell_size, 'local costmap resolution', 0.000001)
    for critic in ('ObstacleFootprint', 'RotateToGoal'):
        if critic not in dwb['critics']:
            raise ValueError(f'DWB requires the {critic} critic')
    _number(dwb['ObstacleFootprint.scale'], 'ObstacleFootprint.scale', 0.000001)
    _number(dwb['RotateToGoal.scale'], 'RotateToGoal.scale', 0.000001)
    for name in ['min_vel_x', 'min_vel_y', 'max_vel_y', 'min_speed_xy', 'min_speed_theta']:
        _required_equal(dwb[name], 0.0, 'DWB.' + name)
    vx = _number(dwb['max_vel_x'], 'DWB.max_vel_x', 0.000001, SIM_MAX_LINEAR_VELOCITY)
    wz = _number(dwb['max_vel_theta'], 'DWB.max_vel_theta', 0.000001, 0.60)
    _number(dwb['max_speed_xy'], 'DWB.max_speed_xy', 0.000001, vx)
    for name in ['acc_lim_x', 'acc_lim_theta']:
        _number(dwb[name], 'DWB.' + name, 0.000001)
    for name in ['decel_lim_x', 'decel_lim_theta']:
        _number(dwb[name], 'DWB.' + name, maximum=-0.000001)
    xy_tolerance = _number(controller['goal_checker']['xy_goal_tolerance'],
                           'goal xy tolerance', 0.000001, 0.10)
    _number(controller['goal_checker']['yaw_goal_tolerance'], 'goal yaw tolerance', 0.000001, 0.15)
    _required_equal(controller['goal_checker']['plugin'],
                    'nav2_controller::StoppedGoalChecker', 'goal checker')
    _required_equal(controller['goal_checker']['stateful'], False, 'current goal XY must be checked')
    stopped_velocity = _number(controller['goal_checker']['trans_stopped_velocity'],
                               'goal stopped translation velocity', 0.000001, 0.01)
    _number(controller['goal_checker']['rot_stopped_velocity'],
            'goal stopped rotation velocity', 0.000001, 0.02)
    # The goal checker is the single source for DWB approach/alignment
    # thresholds. Smac's `tolerance` remains an independent A* search fallback.
    # Revalidation must reject a contradictory resolved configuration as well.
    for params, field, value, label in [
        (dwb, 'xy_goal_tolerance', xy_tolerance, 'DWB terminal XY'),
        (dwb, 'trans_stopped_velocity', stopped_velocity, 'DWB stopped translation'),
    ]:
        if field in params:
            _required_equal(params[field], value, label)
        params[field] = value
    _required_equal(controller['progress_checker']['plugin'],
                    'omslam_navigation::SpeedAwareProgressChecker', 'progress checker')
    _required_equal(behavior['behavior_plugins'], ['wait'], 'behavior_plugins')
    _required_equal(behavior['wait']['plugin'], 'nav2_behaviors::Wait', 'wait behavior')
    _required_equal(bt['navigators'], ['navigate_to_pose'], 'navigators')
    _required_equal(lifecycle['node_names'], NAVIGATION_NODES, 'lifecycle activation order')
    _required_equal(lifecycle['attempt_respawn_reconnection'], False, 'automatic reconnection')
    _required_equal(lifecycle['autostart'], True, 'Nav2 lifecycle autostart')
    _required_equal(guard['lifecycle_nodes'], NAVIGATION_NODES, 'guard lifecycle_nodes')
    for params, name in [(controller, 'controller'), (smoother, 'smoother')]:
        _required_equal(params['enable_stamped_cmd_vel'], True, name + '.enable_stamped_cmd_vel')
    _required_equal(smoother['stamp_smoothed_velocity_with_smoothing_time'], False, 'stamp preservation')
    _required_equal(smoother['feedback'], 'OPEN_LOOP', 'smoother feedback')
    _required_equal(smoother['min_velocity'], [-vx if reverse else 0.0, 0.0, -wz], 'smoother min_velocity')
    _required_equal(smoother['max_velocity'], [vx, 0.0, wz], 'smoother max_velocity')
    _required_equal(smoother['deadband_velocity'], [0.0, 0.0, 0.0], 'smoother deadband')
    _number(smoother['velocity_timeout'], 'velocity_timeout', 0.001, 0.30)
    for key in ['scan_timeout', 'depth_timeout', 'health_timeout']:
        _number(guard[key], 'guard.' + key, 0.001, 0.30)
    _required_equal(guard['max_linear_velocity'], vx, 'guard linear limit')
    _required_equal(guard['max_angular_velocity'], wz, 'guard angular limit')
    for name, expected in [('base_frame', 'base_footprint'), ('odom_frame', 'odom'),
                           ('map_frame', 'map'), ('scan_frame', 'laser_frame')]:
        _required_equal(guard[name], expected, 'guard.' + name)
    for key, upper in [('odom_timeout', 0.20), ('wheel_timeout', 0.40), ('map_tf_timeout', 0.50),
                       ('clock_stall_timeout', 0.30), ('map_jump_translation', 0.10),
                       ('map_jump_rotation', 0.10), ('service_timeout', 2.0)]:
        _number(guard[key], 'guard.' + key, 0.001, upper)
    _number(guard['recovery_healthy_time'], 'guard.recovery_healthy_time', 1.0)
    for costmap, expected_frame, expected_layers in [
        ('global_costmap', 'map', ['static_layer', 'obstacle_layer', 'camera_memory', 'keepout', 'inflation_layer']),
        ('local_costmap', 'odom', ['obstacle_layer', 'camera_memory', 'keepout', 'inflation_layer']),
    ]:
        params = c[costmap][costmap]['ros__parameters']
        _required_equal(params['use_sim_time'], True, costmap + '.use_sim_time')
        _required_equal(params['robot_base_frame'], 'base_footprint', costmap + '.base frame')
        _required_equal(params['global_frame'], expected_frame, costmap + '.global frame')
        _required_equal(params['resolution'], 0.05, costmap + '.resolution')
        _number(params['inflation_layer']['inflation_radius'],
                costmap + '.inflation_radius',
                max(math.hypot(x, y) for x, y in PADDED_FOOTPRINT) + params['resolution'])
        _required_equal(json.loads(params['footprint']), FOOTPRINT, costmap + '.footprint')
        _required_equal(params['footprint_padding'], 0.02, costmap + '.padding')
        _required_equal(params['plugins'], expected_layers, costmap + '.plugins')
        for layer, plugin in [('obstacle_layer', 'nav2_costmap_2d::ObstacleLayer'),
                              ('inflation_layer', 'nav2_costmap_2d::InflationLayer')]:
            _required_equal(params[layer]['plugin'], plugin, costmap + '.' + layer)
        _required_equal(params['camera_memory']['plugin'],
                        'omm_costmap_layers::ObstacleMemoryLayer', costmap + '.memory plugin')
        _required_equal(params['keepout']['plugin'],
                        'omm_costmap_layers::KeepoutLayer', costmap + '.keepout plugin')
        _number(params['camera_memory']['sensor_timeout'], costmap + '.memory timeout', 0.001, 0.30)
        _required_equal(params['obstacle_layer']['scan']['topic'], '/scan', costmap + '.scan')
        for field in ['clearing', 'marking']:
            _required_equal(params['obstacle_layer']['scan'][field], True, costmap + '.scan.' + field)
        _number(params['camera_memory']['padding'], costmap + '.memory padding', 0.0)
    _required_equal(c['global_costmap']['global_costmap']['ros__parameters']['static_layer']['plugin'],
                    'nav2_costmap_2d::StaticLayer', 'global static plugin')
    for params, name in [(bt, 'bt_navigator'), (behavior, 'behavior_server')]:
        _required_equal(params['robot_base_frame'], 'base_footprint', name + '.robot_base_frame')
    lattice = Path(planner['GridBased']['lattice_filepath'])
    if not lattice.is_absolute():
        lattice = share / lattice
    lattice = lattice.resolve(strict=True)
    if hashlib.sha256(lattice.read_bytes()).hexdigest() != LATTICE_SHA256:
        raise ValueError('Lattice does not match the pinned official diff primitives')
    data = json.loads(lattice.read_text(encoding='utf-8'))
    metadata = data['lattice_metadata']
    for key, expected in [('motion_model', 'diff'), ('grid_resolution', 0.05),
                          ('turning_radius', 0.5), ('num_of_headings', 16),
                          ('number_of_trajectories', 112)]:
        _required_equal(metadata[key], expected, 'lattice.' + key)
    if len(metadata['heading_angles']) != 16 or len(data['primitives']) != 112:
        raise ValueError('Lattice arrays do not match metadata')
    planner['GridBased']['lattice_filepath'] = str(lattice)
    tree_path = Path(bt['default_nav_to_pose_bt_xml'])
    if not tree_path.is_absolute():
        tree_path = share / tree_path
    tree_path = tree_path.resolve(strict=True)
    tree = ET.parse(tree_path)
    recovery = tree.findall('.//RecoveryNode')
    if len(recovery) != 1 or recovery[0].get('number_of_retries') != '1':
        raise ValueError('The navigation BT must have exactly one retry budget of one')
    for tag in ['ComputePathToPose', 'FollowPath', 'CancelControl', 'Wait']:
        if len(tree.findall('.//' + tag)) != 1:
            raise ValueError(f'The bounded navigation BT requires one {tag}')
    if any(tree.findall('.//' + tag) for tag in ['Spin', 'BackUp', 'ClearEntireCostmap', 'RetryUntilSuccessful']):
        raise ValueError('Unbounded or excluded recovery nodes are not supported')
    _required_equal(tree.find('.//Wait').get('wait_duration'), '2.0', 'recovery wait')
    bt['default_nav_to_pose_bt_xml'] = str(tree_path)
    return c


def load_configuration(share, override=None, *, operator_enabled=None):
    share = Path(share)
    result = merge_parameters(_read_yaml(share / 'config/nav2_common.yaml'),
                              _read_yaml(share / 'config/sim_profile.yaml'))
    if override:
        overrides = _read_yaml(override)
        _validate_override(result, overrides)
        result = merge_parameters(result, overrides)
    if operator_enabled is not None:
        result['navigation_guard']['ros__parameters']['operator_enabled'] = operator_enabled
    return validate_configuration(result, share)


def _include(path, arguments):
    # Drop unrelated parent LaunchConfigurations; every child input is explicit.
    return GroupAction(scoped=True, forwarding=False, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())])


def _start(context, mode):
    get = lambda key: LaunchConfiguration(key).perform(context)

    def boolean(name):
        value = get(name).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return value == 'true'

    if get('backend') != 'sim' or not boolean('use_sim_time'):
        raise ValueError('P4 navigation requires backend:=sim and use_sim_time:=true')
    if boolean('autostart_base'):
        raise ValueError('Use /navigation/enable after readiness; autostart_base must be false')
    share = Path(get_package_share_directory('omslam_navigation'))
    mapping_share = Path(get_package_share_directory('omslam_mapping'))
    config = load_configuration(share, get('nav2_params_file') or None, operator_enabled=boolean('operator_enabled'))
    config['navigation_guard']['ros__parameters']['localization_mode'] = mode
    validate_configuration(config, share)
    backend_mode = 'mapping' if mode == 'slam' else 'localization'
    backend_params = Path(get('backend_params_file')) if get('backend_params_file') else (
        mapping_share / 'config' / f'{backend_mode}.yaml')
    if not backend_params.is_file():
        raise ValueError(f'Backend parameter file does not exist: {backend_params}')
    backend_config = _read_yaml(backend_params)
    for name in (['slam_toolbox'] if mode == 'slam' else ['map_server', 'amcl']):
        _required_equal(backend_config[name]['ros__parameters'].get('use_sim_time'),
                        True, name + '.use_sim_time')
    for key in ['spawn_x', 'spawn_y', 'spawn_yaw', 'initial_x', 'initial_y', 'initial_yaw']:
        if key != 'spawn_x' or get(key):
            _number(float(get(key)), key)
    initial_pose = boolean('initial_pose')
    if mode == 'amcl':
        map_path = str(validate_map_bundle(get('map')))
    else:
        if get('map'):
            raise ValueError('SLAM mode does not accept map; use navigation.launch.py for AMCL')
        map_path = ''
    # Map coordinates only have meaning within the selected map. Starting a
    # fresh SLAM session must not inherit zones from a previous map or override.
    keepout_file = keepout_file_for_map(map_path) if map_path else None
    config['navigation_guard']['ros__parameters']['keepout_file'] = str(keepout_file or '')
    actions = []
    if boolean('start_sim'):
        sim = Path(get_package_share_directory('omslam_sim'))
        actions.append(_include(sim / 'launch/sim.launch.py', {
            'world': get('world'), 'gui': str(boolean('gui')).lower(),
            'pedestrians': str(boolean('pedestrians')).lower(),
            'fault_relay': str(boolean('fault_relay')).lower(),
            'gui_renderer': get('gui_renderer'), 'verbosity': get('verbosity'),
            'x': get('spawn_x'), 'y': get('spawn_y'), 'yaw': get('spawn_yaw'),
        }))
    actions.append(_include(mapping_share / 'launch' / f'{backend_mode}.launch.py', {
        'use_sim_time': 'true', 'params_file': str(backend_params.resolve()),
        'map': map_path, 'initial_pose': str(initial_pose).lower(),
        'x': get('initial_x'), 'y': get('initial_y'), 'yaw': get('initial_yaw'),
    }))
    # One merged parameter file reaches child costmap nodes created internally
    # by planner/controller; per-node dictionaries would lose those namespaces.
    with tempfile.NamedTemporaryFile(mode='w', prefix='omslam-nav-', suffix='.yaml',
                                     encoding='utf-8', delete=False) as output:
        yaml.safe_dump(config, output, sort_keys=False)
        parameter_path = Path(output.name)
    atexit.register(parameter_path.unlink, missing_ok=True)
    actions.append(Node(package='omm_perception', executable='obstacle_memory',
                        name='obstacle_memory', output='screen',
                        parameters=[str(share / 'config/perception.yaml')]))
    for package, executable, name, remappings in [
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         [('cmd_vel', '/cmd_vel/requested'), ('cmd_vel_smoothed', '/cmd_vel/smoothed')]),
        ('nav2_planner', 'planner_server', 'planner_server', []),
        ('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', '/cmd_vel/nav')]),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', [('cmd_vel', '/cmd_vel/nav')]),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
        ('omslam_navigation', 'navigation_guard', 'navigation_guard', []),
        ('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation', []),
    ]:
        actions.append(Node(package=package, executable=executable, name=name, output='screen',
                            parameters=[str(parameter_path)], remappings=remappings, respawn=False))
    return actions


def make_launch_description(mode):
    if mode not in ('slam', 'amcl'):
        raise ValueError('Navigation mode must be slam or amcl')
    arguments = {
        'backend': ('sim', 'P4 supports only the Gazebo simulation backend'),
        'use_sim_time': ('true', 'Required for simulator sensor and TF timestamps'),
        'start_sim': ('true', 'Start one robot and simulator; false reuses an existing simulator'),
        'world': ('airport', 'Installed simulator world, airport or p4_course'),
        'gui': ('true', 'Show Gazebo GUI'),
        'pedestrians': ('false', 'Enable airport pedestrian traffic'),
        'fault_relay': ('false', 'Enable the explicit verification sensor relay'),
        'gui_renderer': ('ogre', 'Gazebo GUI renderer'),
        'verbosity': ('2', 'Gazebo verbosity'),
        'spawn_x': ('', 'Initial world X, defaults to the selected simulator world spawn'),
        'spawn_y': ('0.0', 'Initial world Y position'),
        'spawn_yaw': ('0.0', 'Initial world yaw'),
        'map': ('', 'AMCL requires an absolute simulation map YAML with bundle metadata'),
        'initial_pose': ('false', 'Seed AMCL; false waits for /initialpose before readiness'),
        'initial_x': ('0.0', 'AMCL initial X in map'),
        'initial_y': ('0.0', 'AMCL initial Y in map'),
        'initial_yaw': ('0.0', 'AMCL initial yaw in map'),
        'nav2_params_file': ('', 'Optional YAML overrides merged over common and simulation defaults'),
        'backend_params_file': ('', 'Optional mapping or localization backend YAML'),
        'autostart_base': ('false', 'Must remain false; /navigation/enable owns base activation'),
        'operator_enabled': ('false', 'Use explicit operator ownership, manual control and guarded goal proxy'),
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, description) in arguments.items()),
        OpaqueFunction(function=_start, args=[mode]),
    ])
