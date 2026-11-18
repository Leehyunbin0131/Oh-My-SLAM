"""Save exactly the current map's zones and keep source bundles intact."""
from contextlib import nullcontext
import hashlib
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
import yaml
from geometry_msgs.msg import Point32, Polygon
from omslam_operator_msgs.msg import KeepoutZones
from omslam_operator_msgs.srv import MapCommand
from omslam_navigation.keepouts import load_keepout_file
from omslam_navigation.launch_support import validate_map_bundle
from omslam_operator_support.session import (
    MapSupport, copy_localization_map, map_prefix, save_keepout_sidecar)


ZONES = [[[0., 0.], [2., 0.], [2., 1.], [0., 1.]]]


def bundle(path):
    image = path.with_suffix('.pgm')
    image.parent.mkdir(parents=True, exist_ok=True)
    image.write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
    path.write_text(yaml.safe_dump({'image': image.name, 'resolution': .05,
        'origin': [0., 0., 0.], 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .25}))
    path.with_suffix('.metadata.json').write_text(json.dumps({
        'schema_version': 1, 'backend': 'simulation',
        'frames': {'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_footprint'},
        'files': {name: {'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                  for name, p in [('yaml', path), ('pgm', image)]}}))
    return path


def test_save_sidecar_roundtrip_and_no_overwrite(tmp_path):
    path = bundle(tmp_path / 'map.yaml')
    target = save_keepout_sidecar(path, ZONES)
    assert load_keepout_file(target) == ZONES
    assert validate_map_bundle(path) == path
    original = target.read_bytes()
    with pytest.raises(FileExistsError):
        save_keepout_sidecar(path, [])
    assert target.read_bytes() == original
    with pytest.raises(ValueError, match='기존 지도'):
        map_prefix(tmp_path, str(path.with_suffix('')))
    assert not list(tmp_path.glob('.*.tmp'))


def test_amcl_save_as_copies_raster_and_current_zones_without_old_graph_claim(tmp_path):
    source = bundle(tmp_path / 'source.yaml')
    save_keepout_sidecar(source, [])
    meta = source.with_suffix('.metadata.json')
    data = json.loads(meta.read_text())
    data['files']['posegraph'] = {'path': '/old/graph.posegraph', 'sha256': 'old'}
    meta.write_text(json.dumps(data))
    old = {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    destination = copy_localization_map(source, tmp_path / 'copied')
    save_keepout_sidecar(destination, ZONES)
    assert validate_map_bundle(destination) == destination
    assert load_keepout_file(destination.with_suffix('.keepout.json')) == ZONES
    copied_meta = json.loads(destination.with_suffix('.metadata.json').read_text())
    assert set(copied_meta['files']) == {'yaml', 'pgm'}
    assert copied_meta['operation'] == 'copy_saved_raster'
    assert yaml.safe_load(destination.read_text())['image'] == 'copied.pgm'
    assert destination.with_suffix('.pgm').read_bytes() == source.with_suffix('.pgm').read_bytes()
    assert all((tmp_path / name).read_bytes() == content for name, content in old.items())
    with pytest.raises(ValueError, match='기존 지도'):
        copy_localization_map(source, tmp_path / 'copied')


def test_snapshot_uses_current_guard_session_and_ignores_older_revision(monkeypatch):
    wall = [10.]
    monkeypatch.setattr('omslam_operator_support.session.time.monotonic', lambda: wall[0])
    context = SimpleNamespace(keepout_lock=threading.Lock(), keepout_sessions={},
                             state=SimpleNamespace(session_id='new'), state_at=10.)
    for session, revision, zones in [('old', 99, ZONES), ('new', 2, []), ('new', 1, ZONES)]:
        msg = KeepoutZones(session_id=session, revision=revision)
        msg.header.frame_id = 'map'
        msg.polygons = [Polygon(points=[Point32(x=x, y=y) for x, y in zone]) for zone in zones]
        MapSupport.receive_keepouts(context, msg)
    assert MapSupport.keepout_snapshot(context) == ('new', 2, [])


def test_amcl_save_command_does_not_launch_slam_saver(tmp_path):
    source = bundle(tmp_path / 'source.yaml')
    context = SimpleNamespace(busy=threading.Lock(), root=tmp_path, loaded_map_path=source,
        authorize=lambda _: None, stop_and_confirm=lambda: None,
        keepout_snapshot=lambda: ('session', 3, ZONES))
    req = MapCommand.Request(operation='save', path=str(tmp_path / 'saved'))
    context.map_reservation = lambda _, **kwargs: nullcontext(lambda: None)
    context.save_reserved = lambda *args: MapSupport.save_reserved(context, *args)
    result = MapSupport.command(context, req, MapCommand.Response())
    assert result.success, result.message
    assert len(result.paths) == 2
    assert load_keepout_file(result.paths[1]) == ZONES
    assert not (tmp_path / 'saved.keepout.pending').exists()


def test_invalid_saved_sidecar_rejects_mode_switch_before_stop(tmp_path):
    source = bundle(tmp_path / 'source.yaml')
    source.with_suffix('.keepout.json').write_text('{}')
    calls = []
    context = SimpleNamespace(busy=threading.Lock(), root=tmp_path,
        processes=SimpleNamespace(items={'navigation': object()}),
        authorize=lambda _: None, stop_and_confirm=lambda: calls.append('stop'))
    result = MapSupport.command(context,
        MapCommand.Request(operation='localize', path=str(source)), MapCommand.Response())
    assert not result.success and not calls


def test_changed_zones_during_save_leave_same_prefix_available(tmp_path):
    source = bundle(tmp_path / 'source.yaml')
    snapshots = iter([('session', 3, ZONES), ('session', 4, [])])
    context = SimpleNamespace(busy=threading.Lock(), root=tmp_path, loaded_map_path=source,
        authorize=lambda _: None, stop_and_confirm=lambda: None,
        keepout_snapshot=lambda: next(snapshots))
    context.map_reservation = lambda _, **kwargs: nullcontext(lambda: None)
    context.save_reserved = lambda *args: MapSupport.save_reserved(context, *args)
    request = MapCommand.Request(operation='save', path=str(tmp_path / 'saved'))
    result = MapSupport.command(context, request, MapCommand.Response())
    assert not result.success and not list(tmp_path.glob('saved.*'))
    context.keepout_snapshot = lambda: ('session', 4, [])
    assert MapSupport.command(context, request, MapCommand.Response()).success
    assert load_keepout_file(tmp_path / 'saved.keepout.json') == []


def test_guard_reserves_save_across_two_consoles_and_emergency_stop(tmp_path, monkeypatch):
    import time
    from lifecycle_msgs.msg import State
    from std_srvs.srv import Trigger
    from omslam_navigation.operator_control import OperatorControl
    from omslam_navigation.operator_core import OperatorGate
    from omslam_operator_msgs.msg import OperatorState
    from omslam_operator_msgs.srv import Control

    now = 100_000_000_000
    guard = OperatorControl.__new__(OperatorControl)
    guard.node = SimpleNamespace(now_ns=lambda: now, publish_zero=lambda: None,
        states={'sim_base': [State.PRIMARY_STATE_INACTIVE]}, base_transition=None)
    guard.core, guard.completed, guard.inflight = OperatorGate(), {}, {}
    guard.waiters, guard.pending, guard.map_reservation = [], None, None
    guard.report = lambda *args: None
    def stationary():
        wall = time.monotonic()
        samples = [(now-350_000_000, wall-.35)] if guard.core.clock_ns is None else []
        for stamp, received in [*samples, (now, wall)]:
            guard.core.clock(stamp, received)
            for source in ('wheel', 'odom'):
                guard.core.sample_stop(source, stamp, 0., 0., True, received)
            guard.core.stopped(stamp, received)
    def complete(coroutine):
        with pytest.raises(StopIteration) as ended:
            coroutine.send(None)
        return ended.value.value
    def acquire(name, key):
        return complete(guard.control(Control.Request(command='acquire', owner_id=name,
                                        request_id=key), Control.Response()))
    first = acquire('A', 'first')
    assert first.success and guard.authorized(first.token, first.epoch)
    context = MapSupport.__new__(MapSupport)
    context.busy, context.root = threading.Lock(), tmp_path
    context.loaded_map_path = bundle(tmp_path / 'source.yaml')
    context.args, context.control, context.stop_client = SimpleNamespace(real=False), 'control', 'stop'
    context.state = OperatorState(session_id='guard', stopped=True, base_active=False)
    context.keepout_snapshot = lambda: ('guard', 2, ZONES)
    def call(endpoint, request, timeout=12.):
        if endpoint == 'control':
            response = complete(guard.control(request, Control.Response()))
        else:
            operation = guard.stop(request, Trigger.Response())
            operation.send(None)
            assert not acquire('B', 'during-stop').success
            stationary()
            guard.tick(now, time.monotonic())
            response = complete(operation)
            context.state_at = time.monotonic()
        if not response.success:
            raise RuntimeError(response.message)
        return response
    context.call = call
    call('stop', Trigger.Request())  # A stop without any map reservation also rejects acquire.
    assert not acquire('A', 'first').success
    first = acquire('A', 'new-owner')
    assert first.success and guard.authorized(first.token, first.epoch)
    called = []
    def copy_during_save(source, destination, reserved=False):
        assert context.busy.locked() and not acquire('B', 'during-save').success
        call('stop', Trigger.Request())  # A global stop still works while saving.
        assert not acquire('B', 'after-emergency-stop').success
        called.append(True)
        return copy_localization_map(source, destination, reserved)
    monkeypatch.setattr('omslam_operator_support.session.copy_localization_map', copy_during_save)
    request = MapCommand.Request(operation='save', path=str(tmp_path/'saved'), session_id='A',
                                 token=first.token, epoch=first.epoch)
    saved = context.command(request, MapCommand.Response())
    assert saved.success and called == [True], saved.message
    assert not acquire('A', 'first').success  # Do not replay an expired successful acquire.
    second = acquire('B', 'after-save')
    assert second.success and guard.authorized(second.token, second.epoch)
    assert load_keepout_file(saved.paths[1]) == ZONES
