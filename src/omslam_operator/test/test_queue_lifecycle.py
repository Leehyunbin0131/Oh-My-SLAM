"""Queue result tracking survives cancel, stop and a change of control mode."""
import time

import pytest
from builtin_interfaces.msg import Time
from PySide6.QtCore import QCoreApplication
from omslam_operator.model import ConsoleModel
from omslam_operator.worker import Shared


@pytest.fixture
def model():
    app = QCoreApplication.instance() or QCoreApplication([])
    shared = Shared()
    value = ConsoleModel(shared)
    value.timer.stop()
    value.display_timer.stop()
    value.token = 'owner-token'
    value.epoch = 1
    value.now = lambda: Time(sec=100)
    shared.data['ros_time'] = 100.
    shared.data['operator'] = {
        'mode': 'AUTO', 'phase': 'ACTIVE', 'owner_id': value.owner,
        'session_id': 'guard-session', 'stopped': False,
    }
    value._control_state = shared.data['operator']
    value.queue = [{'x': 1., 'y': 0., 'yaw': 0., 'status': '대기'},
                   {'x': 2., 'y': 0., 'yaw': 0., 'status': '대기'}]
    # A previous goal's terminal result must be discarded at enqueue, not later.
    shared.data['goal_result'] = {'success': True, 'status': 4}
    value.runQueue()
    assert shared.data['goal_result'] == {}
    assert value.queue_index == 0 and value.queue_waiting
    shared.controls.clear()  # The worker has dispatched the first goal.
    shared.goal_active = True
    yield value
    value.timer.stop()
    value.display_timer.stop()
    value.deleteLater()
    assert app is not None


def fresh_stop(model, age=0.):
    shared = model.s
    shared.data['operator']['stopped'] = True
    for topic in ('/operator/state', '/odom', '/wheel/odometry'):
        shared.data['topics'][topic] = {'received': time.monotonic()-age, 'stamp': 100.}
    for topic in ('/odom', '/wheel/odometry'):
        shared.data['velocity'][topic] = {'v': 0., 'w': 0.}


@pytest.mark.parametrize('operation', ['cancel', 'stop', 'mode'])
def test_halt_keeps_current_row_until_terminal_and_fresh_stop(model, operation):
    if operation == 'mode':
        model.control('MANUAL')
    else:
        getattr(model, operation)()
    assert model.queue_running is False
    assert model.queue_waiting is True
    assert '정지 확인 중' in model.queue[0]['status']
    # Neither a mode/stop response nor a measured stop replaces an action result.
    fresh_stop(model)
    model.tick()
    assert model.queue_waiting
    model.s.goal_active = False
    model.s.data['goal_result'] = {'success': False, 'status': 5}
    fresh_stop(model, age=1.)
    model.tick()
    assert model.queue_waiting and model.queue[0]['status'] != '취소됨'
    fresh_stop(model)
    model.tick()
    assert model.queue[0]['status'] == '취소됨'
    assert model.queue[1]['status'] == '대기'
    assert not model.queue_waiting and not model.queue_running
    assert not any(kind == 'navigate' for kind, _ in model.s.controls)


@pytest.mark.parametrize('success', [True, False])
def test_queue_advances_only_on_success_and_fresh_stop(model, success):
    model.s.goal_active = False
    model.s.data['goal_result'] = {'success': success, 'status': 4 if success else 6}
    model.tick()
    assert model.queue_waiting
    assert model.queue[1]['status'] == '대기'
    fresh_stop(model)
    model.tick()
    if success:
        assert model.queue[0]['status'] == '완료'
        assert model.queue[1]['status'] == '이동 중'
        assert model.queue_index == 1 and model.queue_waiting and model.queue_running
        assert model.s.data['goal_result'] == {}
        model.tick()
        assert model.queue[1]['status'] == '이동 중'  # Prior success is not reused.
    else:
        assert model.queue[0]['status'] == '실패'
        assert model.queue[1]['status'] == '대기'
        assert not model.queue_running and not model.queue_waiting
