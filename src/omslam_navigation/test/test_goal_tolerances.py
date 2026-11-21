"""One launch setting controls approach, alignment and goal completion."""
import copy
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from omslam_navigation.launch_support import load_configuration, validate_configuration


def test_goal_checker_thresholds_are_connected_to_planner_and_dwb(tmp_path):
    share = Path(get_package_share_directory('omslam_navigation'))
    override = tmp_path / 'goal.yaml'
    override.write_text(yaml.safe_dump({'controller_server': {'ros__parameters': {
        'goal_checker': {'xy_goal_tolerance': .06, 'trans_stopped_velocity': .005},
    }}}))
    for path, xy, stopped in [(None, .08, .01), (override, .06, .005)]:
        config = load_configuration(share, path)
        controller = config['controller_server']['ros__parameters']
        planner = config['planner_server']['ros__parameters']['GridBased']
        assert planner['xy_goal_tolerance'] == controller['goal_checker']['xy_goal_tolerance'] == xy
        assert controller['FollowPath']['xy_goal_tolerance'] == xy
        assert controller['FollowPath']['trans_stopped_velocity'] == stopped
        assert controller['goal_checker']['trans_stopped_velocity'] == stopped
        # A* fallback distance is deliberately separate from the arrival circle.
        assert planner['tolerance'] == .04
        assert planner['tolerance'] < xy
        assert controller['goal_checker']['plugin'] == 'nav2_controller::StoppedGoalChecker'
        assert controller['goal_checker']['stateful'] is False


@pytest.mark.parametrize('section,plugin,field,value', [
    ('planner_server', 'GridBased', 'xy_goal_tolerance', .2),
    ('controller_server', 'FollowPath', 'xy_goal_tolerance', .2),
    ('controller_server', 'FollowPath', 'trans_stopped_velocity', .1),
    ('controller_server', 'FollowPath', 'RotateToGoal.scale', 0.),
    ('controller_server', 'goal_checker', 'stateful', True),
])
def test_resolved_arrival_contract_rejects_divergence(section, plugin, field, value):
    share = Path(get_package_share_directory('omslam_navigation'))
    config = copy.deepcopy(load_configuration(share))
    config[section]['ros__parameters'][plugin][field] = value
    with pytest.raises(ValueError):
        validate_configuration(config, share)


def test_alignment_critic_cannot_be_removed():
    share = Path(get_package_share_directory('omslam_navigation'))
    config = load_configuration(share)
    config['controller_server']['ros__parameters']['FollowPath']['critics'].remove('RotateToGoal')
    with pytest.raises(ValueError, match='RotateToGoal'):
        validate_configuration(config, share)
