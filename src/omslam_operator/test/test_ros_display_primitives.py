"""Normalize actual Jazzy scalar storage before JSON and QML presentation."""
import json
import math
from collections import OrderedDict
from types import SimpleNamespace
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from omslam_operator_msgs.action import OperatorNavigate
from omslam_operator_msgs.msg import MotorTelemetry
from omslam_operator_msgs.srv import Control
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import CameraInfo
from omslam_operator import worker
from omslam_operator.worker import ConsoleNode, plain


def test_plain_matches_json_round_trip_for_ros_messages():
    large_int = 2**53 + 1
    telemetry = MotorTelemetry(
        counts=[large_int, -large_int], ab=[0, 255], armed=True, settled=[True, False],
        measured_rpm=[1.25, float('nan'), float('inf')],
        counters=[KeyValue(key='encoder', value='left')])
    diagnostics = DiagnosticArray(status=[DiagnosticStatus(
        level=DiagnosticStatus.WARN, values=[KeyValue(key='active', value='true')])])
    response = Control.Response(success=True, epoch=large_int)
    goal = OperatorNavigate.Goal(epoch=large_int)
    goal.pose.header.frame_id = 'map'

    for message in (telemetry, diagnostics, response, goal):
        previous = json.loads(json.dumps(message_to_ordereddict(message)))
        current = plain(message)
        assert type(current) is dict
        assert json.dumps(current) == json.dumps(previous)

    data = plain(telemetry)
    assert type(data['counts']) is list and data['counts'] == [large_int, -large_int]
    assert all(type(value) is int for value in data['counts'])
    assert type(data['ab']) is list and data['ab'] == [0, 255]
    assert all(type(value) is int for value in data['ab'])
    assert data['armed'] is True
    assert type(data['settled']) is list and all(type(value) is bool for value in data['settled'])
    rpm = data['measured_rpm']
    assert type(rpm) is list and all(type(value) is float for value in rpm)
    assert rpm[0] == 1.25 and math.isnan(rpm[1]) and rpm[2] == math.inf
    assert type(data['counters']) is list and type(data['counters'][0]) is dict
    data['counters'][0]['value'] = 'right'
    assert telemetry.counters[0].value == 'left'
    diagnostic = plain(diagnostics)['status'][0]
    assert type(diagnostic['level']) is str and diagnostic['level'] == chr(1)
    assert type(diagnostic['values'][0]) is dict
    assert plain(response)['success'] is True
    converted_goal = plain(goal)
    assert type(converted_goal['epoch']) is int and converted_goal['epoch'] == large_int
    assert type(converted_goal['pose']['header']) is dict


def test_plain_converts_nested_tuples_and_preserves_mapping_order(monkeypatch):
    converted = OrderedDict([('later', (OrderedDict([('value', False)]),)), ('earlier', 1)])
    monkeypatch.setattr(worker, 'message_to_ordereddict', lambda _: converted)
    result = plain(object())
    assert list(result) == ['later', 'earlier']
    assert type(result['later']) is list and type(result['later'][0]) is dict
    assert result['later'][0]['value'] is False


def test_diagnostic_byte_level_and_camera_arrays_are_native_primitives():
    shared = SimpleNamespace(lock=threading.RLock(), data={'diagnostics': {}})
    receiver = SimpleNamespace(s=shared)
    message = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = 'base'
    status.level = bytes([DiagnosticStatus.OK[0]]) if isinstance(DiagnosticStatus.OK, bytes) else DiagnosticStatus.OK
    status.message = 'ready'
    status.values = [KeyValue(key='active', value='true')]
    message.status = [status]
    ConsoleNode.diagnostics(receiver, message)
    level = shared.data['diagnostics']['base']['level']
    assert type(level) is int and level == 0
    assert json.loads(json.dumps(shared.data))['diagnostics']['base']['level'] == 0

    camera = CameraInfo()
    camera.width = 320
    camera.height = 240
    camera.k = [200., 0., 160., 0., 200., 120., 0., 0., 1.]
    camera.d = [.1, .2, .3, .4, .5]
    ConsoleNode.info(receiver, 'rgb', camera)
    intrinsics = shared.data['intrinsics']['rgb']
    assert all(type(value) is float for value in intrinsics['k'] + intrinsics['d'])
    decoded = json.loads(json.dumps(shared.data))
    assert decoded['intrinsics']['rgb']['k'][0] == 200.
    assert decoded['intrinsics']['rgb']['d'] == [.1, .2, .3, .4, .5]
