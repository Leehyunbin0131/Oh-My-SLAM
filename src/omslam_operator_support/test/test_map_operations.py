"""Prevent map operations from mutating a running or unowned session."""
from contextlib import nullcontext
from pathlib import Path
import threading
import time
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
        authorize=lambda _: None, map_reservation=lambda *a, **k: nullcontext(lambda: None),
        stop_and_confirm=reject('stop unknown'),
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


def test_amcl_initialization_rejects_an_earlier_map_and_waits_for_local_clock():
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from omslam_navigation.operator_control import OperatorControl
    guard = OperatorControl.__new__(OperatorControl)
    clock = [100_000_000_000]
    guard.node = SimpleNamespace(now_ns=lambda: clock[0])
    guard.session_start_ns = 99_900_000_000
    guard.pending_pose, guard.localization_initialized = None, False
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.orientation.w = 1.
    msg.header.stamp.sec, msg.header.stamp.nanosec = 99, 890_000_000
    guard.amcl_pose(msg)
    assert not guard.localization_initialized and guard.pending_pose is None
    msg.header.stamp.sec, msg.header.stamp.nanosec = 100, 40_000_000
    guard.amcl_pose(msg)
    assert not guard.localization_initialized
    clock[0] += 40_000_000
    guard.confirm_localization(clock[0])
    assert guard.localization_initialized


def test_initialpose_holds_map_lease_until_current_estimate_and_teardown_retires_it():
    from contextlib import contextmanager
    import time
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from omslam_operator_msgs.msg import OperatorState
    context = MapSupport.__new__(MapSupport)
    context.busy = threading.Lock()
    events = []
    @contextmanager
    def reservation(request, ending_guard=False):
        events.append(('begin', ending_guard))
        try:
            yield lambda: events.append('confirm')
        finally:
            events.append('retired' if ending_guard and not context.processes.items else 'released')
    context.authorize = lambda request: None
    context.stop_and_confirm = lambda: events.append('stopped')
    context.map_reservation = reservation
    context.count_subscribers = lambda topic: 1
    context.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=100_000_000_000, to_msg=lambda: Time(sec=100)))
    context.state = OperatorState(session_id='old', localization_initialized=True, stopped=True)
    context.state_at = time.monotonic()
    def publish(message):
        assert events == [('begin', False), 'stopped', 'confirm']
        estimate = PoseWithCovarianceStamped()
        estimate.header = message.header
        context.receive_amcl(estimate)
        events.append('amcl')
    context.pose_pub = SimpleNamespace(publish=publish)
    result = context.command(MapCommand.Request(operation='initialpose'), MapCommand.Response())
    assert result.success, result.message
    assert events == [('begin',False),'stopped','confirm','amcl','confirm','released']
    events.clear()
    context.processes = SimpleNamespace(items={'navigation':True})
    def stop_previous(key):
        events.append('old guard stopped')
        context.processes.items.pop(key)
    context.processes.stop = stop_previous
    context.keepout_lock, context.keepout_sessions = threading.Lock(), {}
    context.map_sub, context.map_group = object(), None
    context.destroy_subscription = lambda sub: None
    context.create_subscription = lambda *args, **kwargs: object()
    def start(*args):
        assert events == [('begin',True),'stopped','confirm','old guard stopped','retired']
        context.processes.items['navigation'] = (SimpleNamespace(poll=lambda:None), None)
        context.state = OperatorState(session_id='new', auto_ready=True)
        context.state.header.stamp = Time(sec=101)
        context.state_at = time.monotonic()
    context.start_navigation = start
    result = context.command(MapCommand.Request(operation='slam'), MapCommand.Response())
    assert result.success, result.message

def test_stationary_amcl_requires_measurement_after_requested_pose(monkeypatch):
    now = [20.]
    monkeypatch.setattr(time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(time, 'sleep', lambda amount: now.__setitem__(0, now[0] + amount))
    state = SimpleNamespace(session_id='new', localization_initialized=True,
                            stopped=True, base_active=False)
    support = SimpleNamespace(state=state, state_at=20., amcl_estimate=(99, 20.1))
    updates = []

    def request_update(request):
        updates.append(request)
        support.amcl_estimate = (102, now[0])
        support.state_at = now[0]
        return SimpleNamespace(done=lambda: True)

    support.nomotion_client = SimpleNamespace(service_is_ready=lambda: True, call_async=request_update)
    MapSupport.wait_initial_pose(support, 'new', 100, 20.)
    assert len(updates) == 1


@pytest.mark.parametrize('reply', ['unanswered', 'complete', 'answered', 'rejected'])
def test_service_call_drops_only_unanswered_requests_from_long_lived_client(reply):
    from rclpy.task import Future
    from std_srvs.srv import Trigger

    class Client:
        def __init__(self):
            self.calls, self.pending, self.removed = [], [], []

        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, request):
            future = Future()
            self.calls.append(future)
            self.pending.append(future)
            response = Trigger.Response(success=reply != 'rejected', message='guard rejected')
            if reply == 'complete':
                future.set_result(response)
            elif reply != 'unanswered':
                threading.Timer(.02, future.set_result, (response,)).start()
            return future

        def remove_pending_request(self, future):
            self.removed.append(future)
            self.pending.remove(future)

    client = Client()
    if reply in ('unanswered', 'rejected'):
        with pytest.raises(RuntimeError, match='응답 시간 초과' if reply == 'unanswered' else 'guard rejected'):
            MapSupport.call(SimpleNamespace(), client, Trigger.Request(), timeout=.2)
    else:
        assert MapSupport.call(SimpleNamespace(), client, Trigger.Request(), timeout=.2).success
    if reply == 'unanswered':
        assert client.removed == client.calls and client.pending == []
    else:
        assert client.removed == [] and client.pending == client.calls and client.calls[0].done()
