"""No-entry regions remain geometric constraints, not just map decorations."""
from types import SimpleNamespace as NS
import json
import math
import pytest
from omslam_navigation.keepouts import (
    validate_polygons, load_keepout_file, pose_allowed, motion_allowed, KeepoutManager)


def rect(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def test_rectangles_reject_invalid_and_normalize(tmp_path):
    rectangle = rect(1., 2., 3., 4.)
    assert validate_polygons([rectangle[::-1]]) == [rectangle]
    invalid = [[[0, 0], [1, 0], [1.1, 1], [0, 1]],
               rect(0., 0., .001, 1.), rect(0., 0., math.nan, 1.),
               [[0, 0], [1, 0], [1, 1], [1, 1]]]
    for polygon in invalid:
        with pytest.raises(ValueError): validate_polygons([polygon])
    with pytest.raises(ValueError): validate_polygons([rectangle] * 65)
    path = tmp_path/'zones.json'
    path.write_text(json.dumps({'schema_version': 1, 'frame_id': 'map', 'polygons': [rectangle]}))
    assert load_keepout_file(path) == [rectangle]
    path.write_text(json.dumps({'schema_version': 1, 'frame_id': 'odom', 'polygons': [rectangle]}))
    with pytest.raises(ValueError): load_keepout_file(path)


def test_goal_uses_rotated_full_body_not_center():
    zones = [rect(-.4, .2, -.3, .4)]
    assert not pose_allowed(zones, 0., 0., 0.)  # free center, rear overlaps
    assert pose_allowed(zones, 0., 0., math.pi)
    assert not pose_allowed([rect(.11, -.1, .2, .1)], 0., 0., 0.)  # touching


def test_command_barrier_checks_swept_turn_and_backward_motion():
    zones = [rect(.45, -.2, .55, .2)]
    assert pose_allowed(zones, 0, 0, 0)
    assert not motion_allowed(zones, (0., 0., 0.), .5, 0.)
    assert motion_allowed(zones, (0., 0., math.pi), .2, 0.)
    assert not motion_allowed([rect(-.7, -.2, -.6, .2)], (0., 0., 0.), -.5, 0.)
    assert not motion_allowed([rect(-.3, -.5, -.2, -.4)], (0., 0., 0.), 0., 1.)
    assert not motion_allowed(zones, (math.nan, 0., 0.), .1, 0.)


def test_mutation_requires_owner_stop_and_current_revision(monkeypatch):
    monkeypatch.setattr('omslam_navigation.keepouts.time.monotonic', lambda: 10.)
    manager = KeepoutManager.__new__(KeepoutManager)
    manager.polygons, manager.revision = [], 0
    published = []
    manager.publish = lambda: published.append(manager.revision)
    core = NS(mode='DISABLED', pending_mode=None, active_goals={}, allowed_goals=set(),
              stopped=lambda *_: True)
    operator = NS(authorized=lambda token, epoch: token == 'owner' and epoch == 1, pending=None)
    manager.node = NS(core=core, operator=operator, states={'sim_base': (2, 10.)},
                      operation=None, base_transition=None, now_ns=lambda: 1_000_000_000)
    polygon = NS(points=[NS(x=x, y=y, z=0.) for x, y in rect(1., 1., 2., 2.)])
    request = NS(token='observer', epoch=1, expected_revision=0, polygons=[polygon])
    response = lambda: NS(success=False, message='', revision=0)
    assert not manager.set_zones(request, response()).success
    request.token = 'owner'
    core.mode = 'MANUAL'
    assert not manager.set_zones(request, response()).success
    core.mode = 'DISABLED';core.stopped=lambda *_: False
    assert not manager.set_zones(request, response()).success
    core.stopped=lambda *_: True
    assert manager.set_zones(request, response()).success
    assert manager.revision == 1 and published == [1]
    # A stale editor cannot overwrite a newer snapshot or clear it.
    request.polygons = []
    assert not manager.set_zones(request, response()).success
    assert manager.polygons
    request.expected_revision = 1
    assert manager.set_zones(request, response()).success
    assert not manager.polygons


def test_disabled_transition_readiness_is_observable_without_crashing():
    import rclpy
    from omslam_navigation.navigation_guard import NavigationGuard
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'operator_enabled:=true'], domain_id=222)
    node = NavigationGuard()
    try:
        node.core.pending_mode = 'DISABLED'
        missing = node.readiness()
        assert isinstance(missing, list) and missing  # no sensors; must remain blocked
        assert node.core.pending_mode == 'DISABLED'
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_five_centimetre_zone_survives_point32_roundtrip_at_airport_coordinates():
    import struct
    def wire(x):
        return struct.unpack('f', struct.pack('f', x))[0]
    points = [[wire(x), wire(y)] for x,y in [(-120.,0.),(-119.95,0.),(-119.95,.05),(-120.,.05)]]
    assert validate_polygons([points])
    with pytest.raises(ValueError):
        validate_polygons([[[0.,0.],[.049,0.],[.049,.05],[0.,.05]]])
