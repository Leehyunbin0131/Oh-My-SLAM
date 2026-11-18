"""Prevent map operations from mutating a running or unowned session."""
from pathlib import Path
import threading
from types import SimpleNamespace
import pytest
from omslam_operator_msgs.srv import MapCommand
from nav_msgs.msg import OccupancyGrid
from omslam_operator_support.session import MapSupport, Processes, map_prefix


def reject(message):
    def operation(*_):
        raise RuntimeError(message)
    return operation


def test_invalid_owner_never_stops_or_writes(tmp_path):
    called = []
    context = SimpleNamespace(busy=threading.Lock(), root=tmp_path,
        authorize=reject('invalid owner'), stop_and_confirm=lambda: called.append('stop'))
    req = MapCommand.Request(operation='save', path=str(tmp_path/'new-map'))
    result = MapSupport.command(context, req, MapCommand.Response())
    assert not result.success and 'invalid owner' in result.message
    assert called == [] and list(tmp_path.iterdir()) == []
    assert not context.busy.locked()


def test_unconfirmed_stop_never_restarts_navigation(tmp_path):
    called = []
    context = SimpleNamespace(busy=threading.Lock(), root=tmp_path,
        processes=SimpleNamespace(items={'navigation': object()}, stop=lambda _: called.append('kill')),
        authorize=lambda _: None, stop_and_confirm=reject('stop unknown'),
        start_navigation=lambda *_: called.append('start'))
    result = MapSupport.command(context, MapCommand.Request(operation='slam'), MapCommand.Response())
    assert not result.success and 'stop unknown' in result.message
    assert called == []


def test_map_bundle_members_are_not_overwritten(tmp_path):
    prefix = tmp_path / 'map'
    Path(str(prefix) + '.posegraph').write_bytes(b'user map')
    with pytest.raises(ValueError, match='기존 지도'):
        map_prefix(tmp_path, str(prefix))
    assert Path(str(prefix)+'.posegraph').read_bytes() == b'user map'


def test_new_static_map_accepts_zero_load_stamp_but_not_old_receipt(monkeypatch):
    context = SimpleNamespace(map_stamp=0, map_received_at=9.,
        state=SimpleNamespace(base_active=False), count_subscribers=lambda _: 1)
    assert not MapSupport.localization_map_ready(context, 10.)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 1
    grid.info.resolution = .05
    grid.data = [0]
    monkeypatch.setattr('omslam_operator_support.session.time.monotonic', lambda: 11.)
    MapSupport.receive_map(context, grid)
    assert context.map_stamp == 0
    assert MapSupport.localization_map_ready(context, 10.)
    context.state.base_active = True
    assert not MapSupport.localization_map_ready(context, 10.)


def test_monitor_does_not_report_intentional_stop_or_old_replacement(tmp_path, monkeypatch):
    import io
    processes = Processes(tmp_path)
    old = SimpleNamespace(pid=123, returncode=None)
    old.poll = lambda: old.returncode
    old.wait = lambda timeout: old.returncode
    processes.items['navigation'] = (old, io.StringIO())
    def kill(pid, sig):
        assert pid == 123
        old.returncode = -2
        # This runs after ownership removal, while the OS exit becomes visible.
        assert processes.exited() == []
        raise ProcessLookupError
    monkeypatch.setattr('omslam_operator_support.session.os.killpg', kill)
    processes.stop('navigation')
    replacement = SimpleNamespace(returncode=None, poll=lambda: None)
    processes.items['navigation'] = (replacement, io.StringIO())
    assert processes.exited() == []
    replacement.returncode = 7
    replacement.poll = lambda: 7
    assert processes.exited() == [('navigation', 7)]


def test_monitor_serializes_poll_with_process_replacement(tmp_path):
    import io
    processes = Processes(tmp_path)
    entered, release, replaced = threading.Event(), threading.Event(), threading.Event()
    def poll():
        entered.set()
        assert release.wait(2.)
        return None
    processes.items['navigation'] = (SimpleNamespace(poll=poll), io.StringIO())
    result = []
    monitor = threading.Thread(target=lambda: result.extend(processes.exited()))
    def replace():
        with processes.lock:
            processes.items.pop('navigation')
            replaced.set()
    monitor.start()
    assert entered.wait(2.)
    changer = threading.Thread(target=replace)
    changer.start()
    try:
        assert not replaced.wait(.05)
    finally:
        release.set()
        monitor.join(2.)
        changer.join(2.)
    assert replaced.is_set() and result == []
