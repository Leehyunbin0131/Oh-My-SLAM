"""Bound GUI work independently of the robot's dense observation/history buffers."""
from collections import deque
from types import SimpleNamespace
import threading

from builtin_interfaces.msg import Time
from omslam_operator.model import ConsoleModel
from omslam_operator.presentation import presentation_snapshot


def test_only_visible_bounded_plot_data_crosses_qml_boundary():
    raw = {'operator': {'mode': 'AUTO'}, 'topics': {}, 'cameras': {}, 'velocity': {},
           'history': [{'t': i*.1, 'gyro': [1., 2., 3.], 'gyro_base': [3., 2., 1.],
                        'rpm': [7., 8.], 'velocity': {'/odom': {'v': .1}}}
                       for i in range(600)],
           'path': [[1., 2.]]*10000, 'memory_points': [[3., 4.]]*10000,
           'diagnostics': {'motor': {'level': 0}}, 'events': [{'message': 'event'}]*200}
    driving = presentation_snapshot(raw, 0, 0, 0)
    assert 'history' not in driving and 'path' not in driving and 'memory_points' not in driving
    assert driving['gyroPlot'] == driving['rpmPlot'] == driving['velocityPlot'] == []
    assert len(driving['events']) == 1 and driving['diagnostics'] == {}

    imu = presentation_snapshot(raw, 1, 1, 1)
    assert len(imu['gyroPlot']) == 300
    assert imu['gyroPlot'][0]['v'] == [3., 2., 1.]
    assert imu['rpmPlot'] == imu['velocityPlot'] == []
    drive = presentation_snapshot(raw, 2, 0, 0)
    assert len(drive['rpmPlot']) == len(drive['velocityPlot']) == 300
    assert drive['velocityPlot'][-1]['v'] == [0., .1, 0.]
    assert drive['gyroPlot'] == []
    assert len(raw['history']) == 600  # Display reduction never drops source observations.


def test_ui_liveness_tick_does_not_refresh_or_emit_render_data():
    shared = SimpleNamespace(lock=threading.RLock(), data={'operator': {'mode': 'AUTO'}},
                             latest_heartbeat=None, controls=deque(), goal_active=False)
    calls = []
    model = SimpleNamespace(
        s=shared, token='session', epoch=4, sequence=10, queue_running=False, queue_waiting=False,
        now=lambda: Time(sec=12, nanosec=345),
        _sync_control=lambda *_: None,
        refresh=lambda: calls.append('render'),
    )
    ConsoleModel.tick(model)
    assert calls == []
    assert shared.latest_heartbeat.token == 'session'
    assert shared.latest_heartbeat.epoch == 4
    assert shared.latest_heartbeat.sequence == 11
    assert shared.latest_heartbeat.header.stamp == Time(sec=12, nanosec=345)
