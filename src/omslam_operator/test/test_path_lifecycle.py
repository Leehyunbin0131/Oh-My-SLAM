"""An ended navigation path cannot reappear from delayed ROS deliveries."""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from omslam_operator_msgs.action import OperatorNavigate
from omslam_operator_msgs.msg import OperatorState
from omslam_operator.worker import ConsoleNode, Shared


def fixture():
    shared, clock, requests = Shared(), [100_000_000_000], []
    def send(goal, feedback_callback):
        accepted, result = Future(), Future()
        requests.append((accepted, result))
        return accepted
    receiver = SimpleNamespace(s=shared, path_debug_pub=None, map_rejoins=[],
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock[0])),
        pending_samples={}, navigate=SimpleNamespace(server_is_ready=lambda: True, send_goal_async=send),
        project=lambda points, frame, stamp: points)
    receiver.subscribe_map_inputs = lambda: receiver.map_rejoins.append(True)
    receiver.assign=lambda *args: ConsoleNode.assign(receiver, *args)
    receiver.debug_event=lambda *args, **kwargs: None
    data=dict(token='owner', epoch=1, x=1., y=0., yaw=0., stamp=Time(sec=100))
    return receiver, clock, requests, data


def plan(seconds, x=1.):
    message=Path()
    message.header.stamp=Time(sec=seconds)
    message.header.frame_id='map'
    pose=PoseStamped()
    pose.pose.position.x=x
    message.poses=[pose]
    return message


def accept(request):
    accepted, result=request
    accepted.set_result(SimpleNamespace(accepted=True,
        goal_id=SimpleNamespace(uuid=[0]*16), get_result_async=lambda: result))
    return result


@pytest.mark.parametrize('status', [4, 5, 6])
def test_terminal_clears_path_and_rejects_late_delivery_until_new_goal(status):
    receiver, clock, requests, data=fixture()
    ConsoleNode.start_goal(receiver, data)
    result=accept(requests[-1])
    ConsoleNode.path(receiver, plan(101))
    assert receiver.s.data['path']==[(1., 0.)]
    revision=receiver.s.map_revisions['path']
    result.set_result(SimpleNamespace(result=OperatorNavigate.Result(
        success=status==4, status=status, message='terminal')))
    assert receiver.s.data['path']==[]
    assert receiver.s.map_revisions['path']>revision
    assert receiver.s.data['goal_result']['status']==status
    assert not receiver.s.goal_active and receiver.s.goal_handle is None

    ConsoleNode.path(receiver, plan(102))
    assert receiver.s.data['path']==[]
    clock[0]=110_000_000_000
    ConsoleNode.start_goal(receiver, data)
    accept(requests[-1])
    assert receiver.s.data['goal_result']=={} and receiver.s.goal_active
    ConsoleNode.path(receiver, plan(102))
    assert receiver.s.data['path']==[]  # Previous goal arriving during the next.
    ConsoleNode.path(receiver, plan(110, x=2.))
    assert receiver.s.data['path']==[(2., 0.)]


def test_observer_without_local_goal_still_displays_paths():
    receiver, _, _, _=fixture()
    ConsoleNode.path(receiver, plan(90))
    assert receiver.s.data['path']==[(1., 0.)]


def test_observer_uses_authoritative_goal_across_windows_and_late_samples():
    receiver, _, requests, data = fixture()
    receiver.last_event_signature = ''
    def report(at, uid='', start=0, session='guard'):
        msg = OperatorState(session_id=session, goal_id=uid, goal_stamp_ns=start*1_000_000_000)
        msg.header.stamp = Time(sec=at)
        ConsoleNode.operator(receiver, msg)
    ConsoleNode.start_goal(receiver, data)
    result = accept(requests[0])
    ConsoleNode.path(receiver, plan(101, 1.5))
    report(101, 'first', 100)
    assert receiver.s.data['path'] == [(1.5, 0.)]
    result.set_result(SimpleNamespace(result=OperatorNavigate.Result(success=True, status=4,
                                                                       internal_goal_id='first')))
    ConsoleNode.path(receiver, plan(102))
    report(103)
    ConsoleNode.path(receiver, plan(109))  # Late sample of the old goal.
    assert receiver.s.data['path'] == []
    ConsoleNode.path(receiver, plan(111, 2.))  # Remote plan can arrive before its goal state.
    report(112, 'second', 110)
    assert receiver.s.data['path'] == [(2., 0.)]
    ConsoleNode.path(receiver, plan(109))
    assert receiver.s.data['path'] == [(2., 0.)]
    report(114)
    ConsoleNode.path(receiver, plan(115))
    report(112, 'second', 110)  # Old /operator/state must not resurrect it.
    assert receiver.s.data['path'] == []
    report(120, session='replacement')
    assert receiver.map_rejoins == [True]
    report(121, 'from-previous-map', 118, session='guard')
    assert receiver.s.data['operator']['session_id'] == 'replacement'


def test_rejected_goal_clears_path_and_does_not_reopen_on_late_plan():
    receiver, _, requests, data=fixture()
    ConsoleNode.path(receiver, plan(99))
    ConsoleNode.start_goal(receiver, data)
    assert receiver.s.data['path']==[]
    accepted, _=requests[-1]
    accepted.set_result(SimpleNamespace(accepted=False, goal_id=SimpleNamespace(uuid=[0]*16)))
    ConsoleNode.path(receiver, plan(101))
    assert receiver.s.data['path']==[] and not receiver.s.goal_active
    assert receiver.s.data['goal_result']['status']==6
