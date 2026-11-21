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


def test_cancel_checks_original_stamp_before_terminal_and_unstamped_actuator_after():
    from verify_sim_navigation import late_cancel_commands
    def event(topic, stamp, wall):
        return dict(topic=topic, stamp=stamp, wall_time=wall, velocity=[.1,0,0,0,0,0])
    old = event('/cmd_vel', 9.99, 32.)
    new = event('/cmd_vel', 10.01, 30.5)
    actuator = event('/sim/cmd_vel', None, 31.1)
    assert late_cancel_commands([old, new, actuator], 10., 31.) == [new, actuator]


def test_sensor_stop_keeps_actual_deadline_when_invalid_or_late_frames_arrive():
    from verify_sim_navigation import last_valid_sensor_stamp
    topic = '/camera/depth/image_rect_raw'
    rows = [dict(topic=topic, stamp=t, wall_time=w, valid_fraction=q)
            for t,w,q in [(10.1,2.,1.),(10.2,3.,0.),(10.3,4.1,1.)]]
    zero = dict(wall_time=4., sim_time=10.51)
    assert zero['sim_time'] - last_valid_sensor_stamp(rows,topic,10.,zero) > .40
