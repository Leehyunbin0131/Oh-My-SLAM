"""Verify minimal standard Nav2 configuration and launch composition."""
import math
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from omslam_navigation.launch_support import load_configuration
from omslam_sim.control import SIM_MAX_LINEAR_VELOCITY, SIM_MAX_WHEEL_RPM


def test_minimal_nav2_configuration_uses_standard_plugins_and_70_rpm():
    share = Path(get_package_share_directory('omslam_navigation'))
    config = load_configuration(share)
    assert SIM_MAX_WHEEL_RPM == 70.0
    expected_vx = 70.0 * math.pi / 30.0 * 0.068
    assert SIM_MAX_LINEAR_VELOCITY == pytest.approx(expected_vx, abs=1e-9)

    planner = config['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_smac_planner::SmacPlannerLattice'
    assert Path(planner['lattice_filepath']).is_file()

    controller = config['controller_server']['ros__parameters']
    assert controller['progress_checker']['plugin'] == 'nav2_controller::SimpleProgressChecker'
    assert controller['goal_checker']['plugin'] == 'nav2_controller::StoppedGoalChecker'
    dwb = controller['FollowPath']
    assert dwb['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert dwb['trajectory_generator_name'] == 'dwb_plugins::StandardTrajectoryGenerator'
    assert dwb['max_vel_x'] == pytest.approx(expected_vx, abs=1e-9)
    assert config['velocity_smoother']['ros__parameters']['max_velocity'][0] == pytest.approx(expected_vx, abs=1e-9)

    assert config['global_costmap']['global_costmap']['ros__parameters']['plugins'] == [
        'static_layer', 'obstacle_layer', 'inflation_layer'
    ]
    assert config['local_costmap']['local_costmap']['ros__parameters']['plugins'] == [
        'obstacle_layer', 'inflation_layer'
    ]
    assert 'collision_monitor' not in config
    assert 'navigation_guard' not in config


def test_slam_and_amcl_launches_compose_standard_nav2_nodes(tmp_path):
    share = Path(get_package_share_directory('omslam_navigation'))
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text(yaml.safe_dump({'image': 'map.pgm', 'resolution': 0.05, 'origin': [0, 0, 0]}))

    for entry, extra in [('slam_navigation.launch.py', {}), ('navigation.launch.py', {'map': str(map_yaml)})]:
        context = LaunchContext()
        context.launch_configurations.update({'world': 'p4_course', 'gui': 'false', **extra})
        launch = PythonLaunchDescriptionSource(str(share / 'launch' / entry)).get_launch_description(context)
        actions = None
        for entity in launch.entities:
            if isinstance(entity, OpaqueFunction):
                actions = entity.execute(context)
                break
            entity.execute(context)
        assert actions is not None
        node_names = [a._Node__node_name for a in actions if isinstance(a, Node)]
        assert node_names == [
            'velocity_smoother',
            'planner_server',
            'controller_server',
            'behavior_server',
            'bt_navigator',
            'lifecycle_manager_navigation',
        ]
