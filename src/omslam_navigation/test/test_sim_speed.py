"""The installed simulation speed cap stays coherent through the command chain."""
import copy
import math
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from omslam_navigation.guard_core import Limits
from omslam_navigation.launch_support import load_configuration, validate_configuration
from omslam_navigation.operator_core import OperatorGate


def test_installed_70_rpm_profile_and_default_operator_fraction():
    share = Path(get_package_share_directory('omslam_navigation'))
    config = load_configuration(share)
    # Computed from the wheel radius and RPM, independently of SIM_MAX_*.
    maximum = 2.0 * math.pi * .068 * 70.0 / 60.0
    dwb = config['controller_server']['ros__parameters']['FollowPath']
    smoother = config['velocity_smoother']['ros__parameters']
    guard = config['navigation_guard']['ros__parameters']
    assert [dwb['max_vel_x'], dwb['max_speed_xy'], smoother['max_velocity'][0], guard['max_linear_velocity']] == pytest.approx([maximum] * 4, abs=1e-9)
    assert dwb['max_vel_theta'] == smoother['max_velocity'][2] == guard['max_angular_velocity'] == .5
    assert smoother['min_velocity'] == [0.0, 0.0, -.5]
    assert [dwb['acc_lim_x'], dwb['acc_lim_theta'], dwb['decel_lim_x'], dwb['decel_lim_theta']] == [.8, .6, -1.5, -1.0]
    assert smoother['max_accel'] == [.8, 0.0, .6] and smoother['max_decel'] == [-1.5, 0.0, -1.0]
    assert smoother['smoothing_frequency'] == 50.0 and smoother['velocity_timeout'] == .3
    assert 'collision_monitor' not in config
    assert config['lifecycle_manager_navigation']['ros__parameters']['node_names'] == guard['lifecycle_nodes'] == [
        'velocity_smoother', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
    gate = OperatorGate(Limits(max_vx=guard['max_linear_velocity'], max_wz=guard['max_angular_velocity']))
    assert gate.speed_percent == 50.
    assert gate.linear_limit == pytest.approx(maximum / 2, abs=1e-9)
    assert gate.angular_limit == .25
    assert gate.wheel_rpm_limit == 35.
    assert OperatorGate().limits.max_vx == pytest.approx(maximum, abs=1e-12)
    assert OperatorGate().limits.max_wz == .5
    for package, filename, expected in [('omslam_sim', 'sim_base.yaml', 70.), ('omslam_base', 'base.yaml', 95.)]:
        data = yaml.safe_load((Path(get_package_share_directory(package)) / 'config' / filename).read_text())
        assert next(iter(data.values()))['ros__parameters']['max_rpm'] == expected
    invalid = copy.deepcopy(config)
    invalid['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] = maximum + .01
    with pytest.raises(ValueError):
        validate_configuration(invalid, share)


def test_inflation_covers_padded_footprint_and_grid_margin():
    share = Path(get_package_share_directory('omslam_navigation'))
    config = load_configuration(share)
    for name in ('global_costmap', 'local_costmap'):
        params = config[name][name]['ros__parameters']
        assert params['inflation_layer']['inflation_radius'] >= math.hypot(.41, .28) + .05
        invalid = copy.deepcopy(config)
        invalid[name][name]['ros__parameters']['inflation_layer']['inflation_radius'] = .45
        with pytest.raises(ValueError, match='inflation_radius'):
            validate_configuration(invalid, share)
