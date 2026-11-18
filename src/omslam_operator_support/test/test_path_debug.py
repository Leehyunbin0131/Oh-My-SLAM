"""Path diagnostics preserve raw geometry, goal identity and incomplete-write evidence."""
import json
import math
import time
import zlib

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from omslam_operator_support.path_debug import GoalCorrelation, RecordWriter, path_metrics, plain


def path(points, angle=0.):
    msg = Path()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = 10
    for x, y in points:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
        msg.poses.append(pose)
    return plain(msg)


def test_raw_geometry_keeps_loop_and_heading_distinct_from_chord():
    raw = path([(0, 0), (1, 0), (1, 1), (0, 1), (.01, 0)], angle=.7)
    stats = path_metrics(raw, {'frame_id': 'map', 'x': .01, 'y': 0., 'yaw': .2})
    assert stats['length_m'] > 4.
    assert stats['chord_m'] == pytest.approx(.01)
    assert stats['length_over_chord'] > 400
    assert stats['endpoint_goal_distance_m'] == pytest.approx(0.)
    assert stats['endpoint_goal_yaw_error_rad'] == pytest.approx(.5)
    assert raw['header']['stamp']['sec'] == 10 and len(raw['poses']) == 5
    assert 'endpoint_goal_distance_m' not in path_metrics(raw, {'frame_id': 'odom'})


def test_goal_correlation_rejects_other_sessions_and_deduplicates_history(tmp_path):
    goals = GoalCorrelation(tmp_path)
    event = {'event': 'goal_requested', 'request_id': 'one', 'event_id': 'request-one',
             'directory': str(tmp_path), 'x': 1., 'y': 2., 'yaw': .3, 'frame_id': 'map'}
    assert not goals.event(dict(event, directory=str(tmp_path/'other')))
    assert goals.event(event)
    assert not goals.event(event)
    assert goals.active == 'one' and goals.target['yaw'] == .3
    assert goals.event({'event': 'goal_response', 'request_id': 'one', 'accepted': True,
                        'action_goal_id': 'server-one'})
    assert goals.event({'event': 'goal_result', 'request_id': 'one', 'result': {'status': 4}})
    assert goals.active == 'one'  # Retain identity for last-path-after-arrival analysis.
    assert goals.goals['one']['events'][-1]['result']['status'] == 4
    assert goals.event(dict(event, request_id='two', event_id='request-two', yaw=1.2))
    assert goals.active == 'two' and goals.target['yaw'] == 1.2


def test_writer_drains_incrementally_and_never_overwrites_run(tmp_path):
    writer = RecordWriter(tmp_path/'run')
    record = {'kind': 'path', 'received_wall_ns': 13, 'ros_time_ns': 10,
              'payload': path([(0, 0), (1, 0)])}
    assert writer.submit(record)
    raw = bytes([0, 255, 100])
    assert writer.submit({'kind': 'grid', 'payload': {'file': 'grid.bin.zlib'}},
                         ('grid.bin.zlib', zlib.compress(raw)))
    # Visibility while running, not only on graceful exit.
    deadline = time.monotonic() + 2.
    while time.monotonic() < deadline and not (writer.directory/'records.jsonl').read_text():
        time.sleep(.02)
    assert (writer.directory/'records.jsonl').read_text()
    status = writer.close()
    assert status['status'] == 'stopped' and status['dropped'] == 0
    assert status['received'] == status['written'] == 2 and status['queued_bytes'] == 0
    lines = [json.loads(line) for line in (writer.directory/'records.jsonl').read_text().splitlines()]
    assert lines[0] == record
    assert zlib.decompress((writer.directory/'grid.bin.zlib').read_bytes()) == raw
    with pytest.raises(FileExistsError):
        RecordWriter(writer.directory)


@pytest.mark.parametrize('limit_kind', ['queue', 'disk'])
def test_capacity_failures_are_explicit_and_bounded(tmp_path, limit_kind):
    writer = RecordWriter(tmp_path/limit_kind,
                          queue_bytes=1 if limit_kind == 'queue' else 1024,
                          max_bytes=1 if limit_kind == 'disk' else 1024)
    accepted = writer.submit({'value': 'test'})
    status = writer.close()
    assert accepted == (limit_kind == 'disk')
    assert status['status'] == 'error' and status['error']
    assert status['dropped'] == 1 and status['written'] == 0
    assert status['queued_bytes'] == 0


def test_disk_error_stays_observable_without_blocking_submit(tmp_path, monkeypatch):
    writer = RecordWriter(tmp_path/'run')
    original = type(writer.directory).open
    def fail_snapshot(self, *args, **kwargs):
        if self.name == 'grid.bin.zlib':
            raise OSError('simulated disk full')
        return original(self, *args, **kwargs)
    monkeypatch.setattr(type(writer.directory), 'open', fail_snapshot)
    assert writer.submit({'kind': 'grid'}, ('grid.bin.zlib', b'data'))
    status = writer.close()
    assert status['status'] == 'error' and 'disk full' in status['error']
    assert status['dropped'] == 1 and status['written'] == 0
    assert not writer.submit({'kind': 'late'})
    assert writer.status()['dropped'] == 2
