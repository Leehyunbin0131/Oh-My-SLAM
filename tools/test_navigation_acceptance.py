import json
import math

import pytest

import build_navigation as build
from verify_sim_navigation import command_limit_evidence


def test_wheel_limits_are_checked_at_the_actuator():
    geometry = {'left_radius': .068, 'right_radius': .068, 'wheel_separation': .48, 'max_rpm': 100.}
    max_v = 100 * math.pi / 30 * geometry['left_radius']
    limits = {'max_linear_velocity': max_v, 'max_angular_velocity': .6}
    topics = ('/cmd_vel/nav', '/cmd_vel/requested', '/cmd_vel/smoothed',
              '/cmd_vel/monitored', '/cmd_vel', '/sim/cmd_vel')
    def event(topic, vx, wz):
        return {'topic': topic, 'velocity': [vx, 0., 0., 0., 0., wz]}
    events = [event(topic, max_v, .6) for topic in topics[:-1]]
    events.append(event(topics[-1], max_v, 0.))
    result = command_limit_evidence(events, geometry, limits)
    assert all(stage['passed'] for stage in result.values())
    assert result[topics[0]]['max_rpm'] > 100
    assert result[topics[-1]]['max_rpm'] == pytest.approx(100)
    events[-1] = event(topics[-1], max_v, .6)
    assert not command_limit_evidence(events, geometry, limits)[topics[-1]]['passed']
    assert not command_limit_evidence(events[:-1], geometry, limits)[topics[-1]]['passed']


@pytest.mark.parametrize('tree', ['src', 'install'])
def test_preflight_rejects_changed_source_or_install(tmp_path, monkeypatch, tree):
    monkeypatch.setattr(build, 'WORKSPACE', tmp_path)
    monkeypatch.setattr(build, 'navigation_packages', lambda: ['fixture'])
    for subtree in ('src', 'install'):
        target = tmp_path/subtree/'fixture'
        target.mkdir(parents=True)
        (target/'config.yaml').write_text('setting: 1')
    record = {'exit_code':0, 'packages':['fixture'],
              'source_sha256':build.tree_hashes('src',['fixture']),
              'installed_sha256':build.tree_hashes('install',['fixture'])}
    path = tmp_path/'reports/build.json'
    build.write_manifest(path, record)
    assert build.verified_build(path) == json.loads(path.read_text())
    (tmp_path/tree/'fixture/config.yaml').write_text('setting: 2')
    with pytest.raises(ValueError, match='differ'):
        build.verified_build(path)
