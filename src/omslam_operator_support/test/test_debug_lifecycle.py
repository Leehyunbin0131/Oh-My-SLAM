"""Isolated ROS graph checks for staged lifecycle startup and both command topics."""
import json
import os
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters
from omslam_operator_msgs.msg import OperatorState
from omslam_operator_support.path_debug import PathDebugNode, RecordWriter, REQUIRED_PARAMETERS

# Parameters a configured stock Jazzy Smac Lattice planner exposes. The removed
# fork additionally declared GridBased.xy_goal_tolerance; stock Jazzy does not.
STOCK_PLANNER_PARAMETERS = [
    'planner_plugins', 'expected_planner_frequency', 'GridBased.plugin', 'GridBased.tolerance',
    'GridBased.smooth_path', 'GridBased.lattice_filepath', 'GridBased.allow_unknown',
    'GridBased.allow_reverse_expansion', 'GridBased.analytic_expansion_max_length',
    'GridBased.max_planning_time', 'GridBased.cache_obstacle_heuristic',
]


def test_recorder_waits_for_configured_plugins_records_nav_and_requeries_next_session(tmp_path):
    rclpy.init(args=[], domain_id=205)
    namespace = '/debug_test_' + str(os.getpid())
    server = Node('test_servers', namespace=namespace)
    recorder = PathDebugNode(RecordWriter(tmp_path/'raw'), str(tmp_path), use_sim_time=False, namespace=namespace)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(recorder)
    configured, plugins = [False], [False]
    def state(request, response):
        response.current_state.id = State.PRIMARY_STATE_ACTIVE if configured[0] else State.PRIMARY_STATE_UNCONFIGURED
        return response
    def listing(request, response):
        response.result.names = sorted(REQUIRED_PARAMETERS['controller_server']) if plugins[0] else ['controller_plugins']
        return response
    def planner_listing(request, response):
        response.result.names = list(STOCK_PLANNER_PARAMETERS)
        return response
    def parameters(request, response):
        response.values = [ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=.1) for _ in request.names]
        return response
    assert 'GridBased.xy_goal_tolerance' not in STOCK_PLANNER_PARAMETERS
    for node, names in (('controller_server', listing), ('planner_server', planner_listing)):
        prefix = namespace + '/' + node
        server.create_service(GetState, prefix+'/get_state', state)
        server.create_service(ListParameters, prefix+'/list_parameters', names)
        server.create_service(GetParameters, prefix+'/get_parameters', parameters)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    operator = server.create_publisher(OperatorState, namespace+'/operator/state', latched)
    speeds = {name: server.create_publisher(TwistStamped, namespace+name, 10)
              for name in ('/cmd_vel/nav', '/cmd_vel/requested', '/cmd_vel')}
    def until(predicate):
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            recorder.housekeeping()
            if predicate():
                return
        raise AssertionError((recorder.parameter_errors, recorder.parameters_completed, recorder.parameter_jobs))
    try:
        operator.publish(OperatorState(session_id='map-one'))
        until(lambda: recorder.parameter_session == 'map-one' and 'not configured' in recorder.parameter_errors.get('controller_server',''))
        assert not recorder.parameters_completed
        configured[0] = True
        until(lambda: 'waiting for configured parameters' in recorder.parameter_errors.get('controller_server',''))
        assert 'controller_server' not in recorder.parameters_completed
        # The configured stock planner completes without the fork-only key.
        until(lambda: 'planner_server' in recorder.parameters_completed)
        plugins[0] = True
        until(lambda: 'controller_server' in recorder.parameters_completed)
        until(lambda: all(pub.get_subscription_count() > 0 for pub in speeds.values()))
        for (name, pub), vx in zip(speeds.items(), (.5, .15, .12)):
            message = TwistStamped()
            message.header.stamp.sec = 30
            message.header.frame_id = 'base_footprint'
            message.twist.linear.x = vx
            pub.publish(message)
        until(lambda: all(recorder.counts[name] for name in speeds))
        plugins[0] = False
        operator.publish(OperatorState(session_id='map-two'))
        until(lambda: recorder.parameter_session == 'map-two' and 'waiting for configured parameters' in recorder.parameter_errors.get('controller_server',''))
        assert 'controller_server' not in recorder.parameters_completed
        plugins[0] = True
        until(lambda: {'controller_server', 'planner_server'} <= recorder.parameters_completed)
        recorder.parameter_deadline = time.monotonic()-1.
        recorder.housekeeping()
        summary = recorder.finish()
        assert not summary['error']
        saved = json.loads((tmp_path/'raw/summary.json').read_text())
        assert saved['parameters_completed'] == ['controller_server', 'planner_server']
        assert 'bt_navigator' in saved['parameters_failed'] and not saved['parameters_pending']
        assert 'planner_server' not in saved['parameters_failed']
        with (tmp_path/'raw/records.jsonl').open() as stream:
            rows = [json.loads(line) for line in stream]
        params = [r['payload'] for r in rows if r['kind']=='parameters']
        controller = [r for r in params if r['node'] == namespace + '/controller_server']
        planner = [r for r in params if r['node'] == namespace + '/planner_server']
        assert [r['session_id'] for r in controller] == ['map-one','map-two']
        assert [r['session_id'] for r in planner] == ['map-one','map-two']
        assert all('goal_checker.xy_goal_tolerance' in r['values'] for r in controller)
        assert all('GridBased.tolerance' in r['values'] and 'GridBased.xy_goal_tolerance' not in r['values']
                   for r in planner)
        observed = {r['topic'][len(namespace):]:r['payload'] for r in rows if r['topic'][len(namespace):] in speeds}
        assert [observed[k]['twist']['linear']['x'] for k in speeds] == [.5,.15,.12]
        assert all(message['header']=={'stamp':{'sec':30,'nanosec':0},'frame_id':'base_footprint'}
                   for message in observed.values())
    finally:
        recorder.writer.close()
        executor.shutdown()
        recorder.destroy_node()
        server.destroy_node()
        rclpy.shutdown()
