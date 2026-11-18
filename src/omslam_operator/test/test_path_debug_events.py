"""Diagnostic events correlate targets/results without exposing drive capabilities."""
from concurrent.futures import Future
import json
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from omslam_operator_msgs.action import OperatorNavigate
from omslam_operator.worker import ConsoleNode, Shared


def fixture(ready=True, enabled=True):
    sent, events = [], []
    shared = Shared('/tmp/path-debug-test' if enabled else '')
    accepted, result = Future(), Future()
    receiver = SimpleNamespace(s=shared,
        path_debug_pub=SimpleNamespace(publish=lambda m: events.append(json.loads(m.data))) if enabled else None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)),
        navigate=SimpleNamespace(server_is_ready=lambda: ready,
            send_goal_async=lambda goal, feedback_callback: (sent.append(goal), accepted)[1]))
    receiver.debug_event=lambda *a,**k: ConsoleNode.debug_event(receiver,*a,**k)
    receiver.assign=lambda key,value: ConsoleNode.assign(receiver,key,value)
    data={'token':'do-not-log-this-capability','epoch':987654321,'x':.9,'y':.2,'yaw':.4,
          'stamp':Time(sec=100,nanosec=123)}
    return receiver, data, sent, events, accepted, result


def test_debug_goal_events_preserve_goal_and_correlate_accepted_result_without_token():
    receiver, data, sent, events, accepted, result = fixture()
    ConsoleNode.start_goal(receiver,data)
    handle=SimpleNamespace(accepted=True,goal_id=SimpleNamespace(uuid=list(range(16))),
                           get_result_async=lambda: result)
    accepted.set_result(handle)
    result.set_result(SimpleNamespace(result=OperatorNavigate.Result(
        success=True,message='arrived',status=4,internal_goal_id='nav2-goal')))
    assert len(sent)==1 and sent[0].token==data['token'] and sent[0].epoch==data['epoch']
    assert [event['event'] for event in events]==['goal_requested','goal_response','goal_result']
    assert len({event['request_id'] for event in events})==1
    assert events[0]['yaw']==.4 and events[0]['stamp']=={'sec':100,'nanosec':123}
    assert events[1]['action_goal_id']==bytes(range(16)).hex()
    assert events[2]['result']['internal_goal_id']=='nav2-goal'
    assert all(event['directory']==receiver.s.path_debug_directory for event in events)
    assert data['token'] not in json.dumps(events) and 'epoch' not in json.dumps(events)
    assert receiver.s.data['goal_result']['success'] and not receiver.s.goal_active


def test_debug_unavailable_server_records_failure_and_disabled_mode_sends_no_events():
    receiver,data,sent,events,_,_=fixture(ready=False)
    ConsoleNode.start_goal(receiver,data)
    assert not sent and events[-1]['event']=='goal_result'
    assert not events[-1]['result']['success']
    receiver,data,sent,events,_,_=fixture(ready=False,enabled=False)
    ConsoleNode.start_goal(receiver,data)
    assert not sent and not events and not receiver.s.data['goal_result']['success']


def test_debug_status_ignores_other_recording_directory():
    receiver,_,_,_,_,_=fixture()
    message=SimpleNamespace(data=json.dumps({'directory':'/tmp/other','status':'recording'}))
    ConsoleNode.path_debug_status(receiver,message)
    assert 'path_debug' not in receiver.s.data
    message.data=json.dumps({'directory':receiver.s.path_debug_directory,'status':'error','error':'disk full'})
    ConsoleNode.path_debug_status(receiver,message)
    assert receiver.s.data['path_debug']['error']=='disk full'
