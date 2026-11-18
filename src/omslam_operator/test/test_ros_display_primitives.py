"""Normalize actual Jazzy scalar storage before JSON and QML presentation."""
import json
from types import SimpleNamespace
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CameraInfo
from omslam_operator.worker import ConsoleNode


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
