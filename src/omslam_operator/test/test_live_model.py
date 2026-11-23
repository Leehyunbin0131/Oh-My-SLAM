"""Fresh ownership and revocable user intent for the simplified ROS GUI."""
import time
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from PySide6.QtCore import QLocale, QObject, QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlComponent, QQmlEngine, qmlRegisterType

from omslam_operator.live_model import LiveConsoleModel
from omslam_operator.visuals import RobotMap
from omslam_operator.worker import Shared


@pytest.fixture
def model():
    app = QGuiApplication.instance() or QGuiApplication([])
    shared = Shared()
    value = LiveConsoleModel(shared)
    value.timer.stop()
    value.display_timer.stop()
    value.now = lambda: Time(sec=100)
    shared.data['ros_time'] = 100.
    shared.data['operator'] = {'mode': 'DISABLED', 'phase': 'READY', 'owner_id': '',
        'epoch': 0, 'session_id': 'guard-1', 'stopped': True, 'base_active': False,
        'operator_enabled': True, 'manual_ready': True, 'auto_ready': True,
        'reverse_allowed': False, 'linear_limit': .1, 'angular_limit': .3,
        'max_linear_velocity': .2, 'max_angular_velocity': .6, 'reason': 'ready'}
    shared.data['operator'].update(min_wheel_rpm=3., min_rotation_velocity=17 * np.pi / 600,
                                   max_wheel_rpm=100., wheel_rpm_limit=50.)
    shared.data['operator'].update(localization_mode='slam', localization_initialized=True)
    shared.data['keepouts'] = {'session_id':'guard-1','revision':0,'polygons':[]}
    shared.map_layers['map'] = {'width': 20, 'height': 20, 'resolution': .1,
                               'origin': [0., 0., 0.], 'cells': np.zeros((20, 20))}
    fresh(value)
    value.tick()
    shared.controls.clear()
    yield value
    value.timer.stop()
    value.display_timer.stop()
    value.deleteLater()
    assert app is not None


def fresh(model):
    for topic in ('/operator/state', '/odom', '/wheel/odometry'):
        model.s.data['topics'][topic] = {'received': time.monotonic(), 'stamp': 100.}
    for topic in ('/odom', '/wheel/odometry'):
        model.s.data['velocity'][topic] = {'v': 0., 'w': 0.}


def commands(model, kind):
    return [data for name, data in model.s.controls if name == kind]


def test_live_ui_qml_binding_updates(model):
    engine = QQmlEngine()
    engine.rootContext().setContextProperty('operatorModel', model)
    component = QQmlComponent(engine)
    component.setData(b'import QtQml\nQtObject { property string mode: operatorModel.ui.mode }', QUrl())
    view = component.create()
    assert view is not None, [error.toString() for error in component.errors()]
    model.refresh()
    assert view.property('mode') == 'DISABLED'
    model.s.data['operator']['mode'] = 'AUTO'
    fresh(model)
    model.refresh()
    assert view.property('mode') == 'AUTO'
    view.deleteLater()


def test_installed_driving_page_and_direct_pad_use_server_caps(model):
    from omslam_navigation.operator_core import OperatorGate

    core = OperatorGate()
    linear, angular = core.axis_limits(3.)
    model.s.data['operator'].update(max_wheel_rpm=70., wheel_rpm_limit=35.,
                                   max_linear_velocity=core.limits.max_vx, max_angular_velocity=.5)
    own(model, 'MANUAL')
    model.refresh()
    qmlRegisterType(RobotMap, 'OhMyRobot', 1, 0, 'NativeMap')
    engine = QQmlEngine()
    engine.rootContext().setContextProperty('operatorModel', model)
    page = Path(get_package_share_directory('omslam_operator')) / 'qml/design/DrivingPage.qml'
    component = QQmlComponent(engine, QUrl.fromLocalFile(str(page)))
    view = component.create()
    assert view is not None, [error.toString() for error in component.errors()]
    slider = view.findChild(QObject, 'speedSlider')
    field = view.findChild(QObject, 'rpmLimitInput')
    pad = view.findChild(QObject, 'directPad')
    assert (slider.property('from'), slider.property('to'), slider.property('value')) == (3., 70., 35.)
    model.s.data['operator'].update(wheel_rpm_limit=3., linear_limit=linear, angular_limit=angular)
    model.refresh()
    assert slider.property('value') == 3.
    assert pad.property('linearLimit') == linear and pad.property('angularLimit') == angular
    decimal = QLocale().decimalPoint()
    field.setProperty('text', '2' + decimal + '999')
    assert not field.property('acceptableInput')
    field.setProperty('text', '17' + decimal + '3456789')
    assert field.property('acceptableInput')
    view.deleteLater()


def test_path_debug_badge_requires_fresh_successful_recorder(model):
    model.s.path_debug_directory='/tmp/current-path-recording'
    model.s.data['path_debug']={'status':'recording','directory':model.s.path_debug_directory,
                              '_received':time.monotonic(),'dropped':0,'error':''}
    model.refresh()
    assert model.controlUi['pathDebugText']=='경로 기록 중'
    assert model.controlUi['pathDebugEnabled']
    model.s.data['path_debug']['dropped']=1
    model.refresh()
    assert model.controlUi['pathDebugState']=='error'
    model.s.data['path_debug']['_received']=time.monotonic()-4.
    model.refresh()
    assert model.controlUi['pathDebugText']=='경로 기록 대기'
    model.s.path_debug_directory=''
    model.refresh()
    assert not model.controlUi['pathDebugEnabled']


def reply(model, mode='DISABLED', fresh_state=True):
    request = model._request
    assert request is not None
    model.s.controls.clear()
    token = 'token-' + request['id']
    epoch = model.epoch + 1
    model.s.data['control_response'] = {'_request_id': request['id'], '_command': request['command'],
        '_received': time.monotonic(), 'success': True, 'token': token, 'epoch': epoch}
    if fresh_state:
        model.s.data['operator'].update(owner_id=model.owner, epoch=epoch, mode=mode,
            phase='ACTIVE' if mode != 'DISABLED' else 'READY', base_active=mode != 'DISABLED')
        fresh(model)
    model.tick()
    return token, epoch


def own(model, mode='AUTO'):
    model.token, model.epoch = 'existing-token', 4
    model.s.data['operator'].update(owner_id=model.owner, epoch=4, mode=mode,
        phase='ACTIVE' if mode != 'DISABLED' else 'READY', base_active=mode != 'DISABLED')
    fresh(model)
    model.tick()
    model.s.controls.clear()


def test_live_route_waits_for_matching_fresh_active_owner(model):
    model.addWaypoint(.5, .5, .3)
    model.startRoute()
    assert len(commands(model, 'control')) == 1
    assert commands(model, 'control')[0]['command'] == 'acquire'
    model.tick()
    assert len(commands(model, 'control')) == 1  # No acquisition retry storm.
    reply(model)
    assert commands(model, 'control')[0]['mode'] == 'AUTO'
    token, epoch = reply(model, 'AUTO', fresh_state=False)
    assert not commands(model, 'navigate')
    model.s.data['operator'].update(owner_id=model.owner, epoch=epoch, mode='AUTO', phase='ACTIVE')
    fresh(model)
    model.tick()
    assert len(commands(model, 'navigate')) == 1
    assert commands(model, 'navigate')[0]['token'] == token
    assert model.queue_running and model.queue_waiting


@pytest.mark.parametrize('release', ['release', 'focus', 'page'])
def test_live_manual_release_cancels_pending_intent_without_late_motion(model, release):
    model.manualIntent(.1, .2, True)
    reply(model)
    assert model._request['mode'] == 'MANUAL'
    if release == 'focus':
        model.setActive(False)
    elif release == 'page':
        model.setPage(1)
    else:
        model.manualIntent(0., 0., False)
    reply(model, 'MANUAL')
    model._neutral_at = time.monotonic() - 1.
    model.tick()
    assert model._pending is None and not model._held
    assert model.s.latest_manual is not None
    assert not model.s.latest_manual.deadman
    assert model.s.latest_manual.command.twist.linear.x == 0.


def test_live_manual_takeover_sends_physical_units_after_neutral_sample(model):
    own(model, 'AUTO')
    model.queue = [{'uid': 'a', 'x': .5, 'y': .5, 'yaw': 0., 'status': '이동 중'}]
    model.queue_running, model.queue_waiting, model.queue_index = True, True, 0
    model.manualIntent(.07, .21, True)
    assert not model.queue_running and model.queue_waiting
    assert len(commands(model, 'control')) == 1
    assert commands(model, 'control')[0]['mode'] == 'MANUAL'
    reply(model, 'MANUAL')
    assert not model.s.latest_manual.deadman
    model._neutral_at = time.monotonic() - 1.
    model.tick()
    assert model.s.latest_manual.deadman
    assert model.s.latest_manual.command.twist.linear.x == pytest.approx(.07)
    assert model.s.latest_manual.command.twist.angular.z == pytest.approx(.21)
    assert not commands(model, 'navigate')
    model.manualIntent(-.1, 0., True)
    assert not model.s.latest_manual.deadman  # Existing rear-observation restriction.


@pytest.mark.parametrize('loss', ['stale', 'owner', 'restart'])
def test_live_loss_never_reacquires_or_replays_held_input(model, loss):
    own(model, 'MANUAL')
    model.manualIntent(.1, 0., True)
    if loss == 'stale':
        model.s.data['topics']['/operator/state']['received'] -= 1.
    elif loss == 'owner':
        model.s.data['operator'].update(owner_id='', epoch=9)
        fresh(model)
    else:
        model.s.data['operator']['session_id'] = 'guard-2'
        fresh(model)
    model.tick()
    assert not model.token and not model._held and model._pending is None
    model.s.controls.clear()
    model.s.data['operator'].update(owner_id='', mode='DISABLED', phase='READY')
    fresh(model)
    model.tick()
    assert not commands(model, 'control') and not commands(model, 'navigate')
    assert model.s.latest_manual is None


def test_live_foreign_owner_is_never_replaced(model):
    model.s.data['operator']['owner_id'] = 'another-operator'
    model.tick()
    model.manualIntent(.1, 0., True)
    assert not commands(model, 'control') and not model._held
    assert '다른 조작자' in model._notice


def test_live_map_load_requires_disabled_owner_and_explicit_initial_pose(model):
    own(model, 'AUTO')
    model.loadMap('/maps/room.yaml')
    assert not commands(model, 'maps')
    assert commands(model, 'control')[0]['mode'] == 'DISABLED'
    reply(model, 'DISABLED')
    assert len(commands(model, 'maps')) == 1
    payload = commands(model, 'maps')[0]
    assert payload['operation'] == 'localize' and payload['token'] == model.token
    model.s.controls.clear()
    model.s.data['map_response'] = {'_request_id': payload['_request_id'], 'success': True, 'message': 'loaded'}
    model.s.data['operator'].update(localization_mode='amcl', localization_initialized=False)
    model.tick()
    assert model._needs_initial_pose
    model.addWaypoint(.5, .5, 0.)
    model.startRoute()
    assert not commands(model, 'navigate') and not commands(model, 'control')
    model.setInitialPose(.1, .2, .3)
    payload = commands(model, 'maps')[0]
    assert payload['operation'] == 'initialpose' and payload['yaw'] == .3
    model.s.data['operator']['auto_ready'] = False
    model.s.data['map_response'] = {'_request_id': payload['_request_id'], 'success': True}
    model.tick()
    assert model._needs_initial_pose
    model.s.data['operator'].update(auto_ready=True, localization_initialized=True)
    fresh(model)
    model.tick()
    assert not model._needs_initial_pose


def test_live_invalid_map_path_and_service_rejection_are_visible(model):
    model.loadMap('')
    assert '경로' in model._notice and not commands(model, 'maps')
    own(model, 'DISABLED')
    model.loadMap('/missing/map.yaml')
    payload = commands(model, 'maps')[0]
    model.s.data['map_response'] = {'_request_id': payload['_request_id'], 'success': False,
                                    'message': 'map YAML does not exist'}
    model.tick()
    assert model._notice == 'map YAML does not exist'
    assert model._map_inflight is None and not model._needs_initial_pose


def test_live_waypoint_edit_reorder_remove_and_unknown_rejection(model):
    model.addWaypoint(.65, .65, .5)
    model.addWaypoint(.5, .6, .7)
    first = model.queue[0]['uid']
    model.editWaypoint(0, .6, .7, 1.2)
    assert model.queue[0]['uid'] == first and model.queue[0]['yaw'] == 1.2
    model.moveWaypoint(0, 1)
    assert model.queue[1]['uid'] == first
    model.removeGoal(0)
    assert len(model.queue) == 1 and model.queue[0]['uid'] == first
    model.s.map_layers['map']['cells'][10, 10] = -1
    model.addWaypoint(1., 1., 0.)
    assert len(model.queue) == 1 and '관측되지' in model._notice


def test_live_shared_manual_auto_speed_accepts_fine_values(model):
    own(model, 'DISABLED')
    model.setSpeedLimit(.025)
    request = commands(model, 'speed')[0]
    assert request['linear_limit'] == .025 and request['speed_percent'] == 12.5
    assert request['wheel_rpm_limit'] == 12.5
    model.refresh()
    assert model.ui['speedOptions'] == [.05, .1, .2]


def test_live_missing_and_stale_telemetry_never_becomes_sample_values(model):
    model.refresh()
    for field in ('leftRpm', 'rightRpm', 'roll', 'pitch', 'heading', 'hostCpu', 'robotCpu'):
        assert model.ui[field] == '—'
    model.s.data['joints'] = {'left_wheel_joint': {'rpm': 12.}, 'right_wheel_joint': {'rpm': 13.}}
    model.s.data['topics']['/joint_states'] = {'received': time.monotonic(), 'stamp': 100.}
    model.s.data['diagnostics']['cpu'] = {'name': 'system_monitor/pc/cpu', 'received': time.monotonic(),
        'values': [{'key': 'host_role', 'value': 'operator_pc'}, {'key': 'valid', 'value': 'true'},
                   {'key': 'busy_percent', 'value': '23'}]}
    model.refresh()
    assert model.ui['leftRpm'] == '12.0' and model.ui['hostCpu'] == '23%'
    assert model.ui['robotCpu'] == '—'
    model.s.data['topics']['/joint_states']['received'] -= 1.
    model.s.data['diagnostics']['cpu']['received'] -= 4.
    model.refresh()
    assert model.ui['leftRpm'] == '—' and model.ui['hostCpu'] == '—'


def test_live_read_only_blocks_every_robot_mutation(model):
    own(model, 'AUTO')
    model.read_only = True
    model.addWaypoint(.5, .5, 0.)
    model.startRoute()
    model.manualIntent(.1, 0., True)
    model.setSpeedLimit(.1)
    model.saveMap('/maps/new')
    model.loadMap('/maps/existing.yaml')
    model.newMap()
    model.setInitialPose(.5, .5, 0.)
    model.control('MANUAL')
    model.acquireControl()
    model.speed(.1, .3, 50.)
    model.mapOperation('slam')
    model.cancel()
    model.navigate()
    model.stop()
    model.shutdown()
    assert not model.s.controls
    assert not model._held and model._pending is None
    assert model.s.latest_manual is None or not model.s.latest_manual.deadman


def test_live_no_path_requires_explicit_terminal_evidence_and_clears_on_new_goal(model):
    model.s.data['goal_result'] = {'success': False, 'status': 6,
        'message': 'Failed to create plan: no valid path found', '_received': time.monotonic()}
    model.refresh()
    assert model.ui['movementState'] == 'no_path'
    model.addWaypoint(.5, .5, 0.)
    model.refresh()
    assert model.ui['movementState'] != 'no_path'
    model.s.data['goal_result'] = {'success': False, 'status': 6,
        'message': 'operator ownership ended', '_received': time.monotonic()}
    model.refresh()
    assert model.ui['movementState'] != 'no_path'


def test_live_amcl_reconnect_requires_map_and_missing_pose_evidence(model):
    model.s.data['operator'].update(localization_mode='amcl', localization_initialized=False, auto_ready=False)
    model.refresh()
    assert model.ui['movementState'] == 'pose_required' and model.ui['needsInitialPose']
    model.s.data['operator']['localization_initialized'] = True
    model.s.data['operator']['phase'] = 'BLOCKED'
    model.refresh()
    assert model.ui['movementState'] == 'blocked'
    assert not model.ui['needsInitialPose']  # Ordinary TF/health loss is not an initial-pose claim.


def test_remote_map_replacement_and_pose_are_shared_by_all_windows(model):
    model.addWaypoint(.5, .5, 1.)
    model.s.map_layers['map']['revision'] = 7
    fresh(model)
    model.tick()
    assert len(model.queue) == 1  # In-session online SLAM revisions preserve the route.
    model.s.data['operator'].update(session_id='another-map', localization_mode='amcl',
                                    localization_initialized=False, auto_ready=False)
    fresh(model)
    model.tick()
    model.refresh()
    assert not model.queue and model.ui['needsInitialPose']
    model.startRoute()
    assert not model.s.controls
    model._needs_initial_pose = True  # The opening window is also waiting for initialization.
    model.s.data['operator'].update(localization_initialized=True, auto_ready=True)
    fresh(model)
    model.tick()
    model.refresh()
    assert not model.ui['needsInitialPose']
    model.addWaypoint(.7, .5, 0.)
    model.startRoute()
    reply(model)
    reply(model, 'AUTO')
    assert [data['x'] for data in commands(model, 'navigate')] == [.7]


def test_refused_rpm_never_replays_after_owner_release(model):
    model.s.data['operator'].update(owner_id='other-window', wheel_rpm_limit=10.)
    fresh(model)
    model.tick()
    model.setRpmLimit(95.)
    assert not model._speed_dirty and not model.s.controls
    model.s.data['operator']['owner_id'] = ''
    fresh(model)
    model.tick()
    model.acquireControl()
    reply(model)
    assert not commands(model, 'speed')
    model.setRpmLimit(37.5)
    assert commands(model, 'speed')[0]['wheel_rpm_limit'] == pytest.approx(37.5)


def test_stop_before_worker_claim_finishes_only_after_measured_stop(model):
    own(model, 'AUTO')
    model.addWaypoint(.5, .5, 0.)
    model.startRoute()
    request = commands(model, 'navigate')[0]['request_id']
    assert model.s.goal_phase == 'queued'
    model.s.data['operator']['stopped'] = False
    model.stop()
    model.tick()
    assert model.queue_waiting and model.s.data['goal_result']['request_id'] == request
    assert [kind for kind, _ in model.s.controls] == ['stop']
    model.s.data['operator']['stopped'] = True
    fresh(model)
    model.tick()
    assert not model.queue_waiting and model.queue[0]['status'] == '취소됨'


@pytest.mark.parametrize('server_rpm,percent,expected_rpm', [
    (70., 25., 17.5), (70., 50., 35.), (70., 100., 70.),
    (100., 25., 25.), (100., 50., 50.), (100., 100., 100.),
])
def test_gui_speed_steps_follow_server_rpm_limit(model, server_rpm, percent, expected_rpm):
    maximum = 2 * np.pi * .068 * server_rpm / 60
    model.s.data['operator'].update(max_linear_velocity=maximum, linear_limit=maximum / 2,
                                    max_angular_velocity=.5, max_wheel_rpm=server_rpm,
                                    wheel_rpm_limit=server_rpm / 2)
    own(model, 'DISABLED')
    selected = maximum * percent / 100
    model.setSpeedLimit(selected)
    request = commands(model, 'speed')[0]
    assert request['linear_limit'] == pytest.approx(selected)
    assert request['speed_percent'] == pytest.approx(percent)
    assert request['wheel_rpm_limit'] == pytest.approx(expected_rpm)
    assert request['angular_limit'] == pytest.approx(.5 * percent / 100)
    model.s.data['operator']['linear_limit'] = selected
    model.refresh()
    assert model.ui['speedOptions'] == pytest.approx([maximum / 4, maximum / 2, maximum])
    assert model.ui['speedIndex'] == [25., 50., 100.].index(percent)
    assert selected * 60 / (2 * np.pi * .068) == pytest.approx(expected_rpm)


def test_numeric_rpm_maps_to_authorized_speed_request(model):
    own(model, 'DISABLED')
    model.setRpmLimit(37.5)
    req=commands(model, 'speed')[0]
    assert req['wheel_rpm_limit']==pytest.approx(37.5)
    assert req['linear_limit']==pytest.approx(.075)
    assert req['angular_limit']==pytest.approx(.225)
    for value in [0., 1., 2.9999995, -1., 100.1, float('nan'), float('inf')]:
        model.setRpmLimit(value)
    assert len(commands(model,'speed'))==1


# Simulation profile: 70 RPM on 0.068 m wheels. max * 70 / 70 rounds above max.
SIM_MAX_LINEAR = 0.49846603436958054
SIM_MAX_RPM = 70.0


@pytest.mark.parametrize('rpm', [3., 17.34567890123456, 35., 70.])
def test_numeric_rpm_reaches_service_and_both_drive_modes_without_round_trip(model, rpm):
    from types import SimpleNamespace
    from omslam_navigation.operator_control import OperatorControl
    from omslam_navigation.operator_core import OperatorGate
    from omslam_operator_msgs.srv import SetSpeedLimit

    model.s.data['operator'].update(max_linear_velocity=SIM_MAX_LINEAR, linear_limit=SIM_MAX_LINEAR / 2,
                                    max_angular_velocity=.5, max_wheel_rpm=SIM_MAX_RPM,
                                    wheel_rpm_limit=SIM_MAX_RPM / 2)
    own(model, 'DISABLED')
    model.setRpmLimit(rpm)
    requests = commands(model, 'speed')
    assert len(requests) == 1
    request = requests[0]
    # The same bounds operator_control.speed() enforces on the service request.
    assert 0 < request['linear_limit'] <= SIM_MAX_LINEAR
    assert 0 < request['angular_limit'] <= .5
    assert request['speed_percent'] == rpm / SIM_MAX_RPM * 100.
    assert request['wheel_rpm_limit'] == rpm
    assert request['linear_limit'] == pytest.approx(SIM_MAX_LINEAR * rpm / SIM_MAX_RPM)
    assert request['angular_limit'] == pytest.approx(max(.5 * rpm / SIM_MAX_RPM, 17 * np.pi / 600))
    core = OperatorGate()
    published = []
    service = SimpleNamespace(core=core, authorized=lambda token, epoch: (token, epoch) == (model.token, model.epoch),
                              publish_speed=lambda: published.append(core.wheel_rpm_limit))
    response = OperatorControl.speed(service, SetSpeedLimit.Request(**request), SetSpeedLimit.Response())
    assert response.success and response.wheel_rpm_limit == rpm and published == [rpm]
    for mode in ('MANUAL', 'AUTO'):
        core.mode = mode
        vx, *_, wz = core.limit_values((request['linear_limit'], 0., 0., 0., 0., 0.))
        assert core.geometry.rpm(vx, wz) == pytest.approx((rpm, rpm))
        for direction in (-1., 1.):
            vx, *_, wz = core.limit_values((0., 0., 0., 0., 0., direction * request['angular_limit']))
            assert wz == pytest.approx(direction * request['angular_limit'])
            if rpm == 3.:
                assert core.geometry.rpm(vx, wz) == (-direction * 3., direction * 3.)
    model.refresh()
    assert model.ui['minRpm'] == 3. and model.ui['maxRpm'] == SIM_MAX_RPM


def test_keepout_edit_waits_for_disabled_owner_and_server_snapshot(model):
    model.addKeepout(.2,.2,.8,.9)
    assert commands(model,'control')[0]['command']=='acquire'
    assert not commands(model,'keepouts')
    reply(model)
    req=commands(model,'keepouts')[0]
    assert req['expected_revision']==0 and req['polygons'][0][0]==[.2,.2]
    assert model.keepoutZones==[]  # never paint an unacknowledged optimistic zone
    model.s.data['keepouts']={'session_id':'guard-1','revision':1,'polygons':[
        {'points':[{'x':x,'y':y} for x,y in req['polygons'][0]]}]}
    model.s.data['keepout_response']={'_request_id':req['_request_id'],'success':True,'message':'saved','revision':1}
    fresh(model);model.tick()
    assert len(model.keepoutZones)==1 and model.keepoutZones[0]['x_min']==.2
    model.s.controls.clear()
    model.removeKeepout(0)
    assert commands(model,'keepouts')[0]['polygons']==[]
    assert commands(model,'keepouts')[0]['expected_revision']==1


def test_keepout_stale_session_and_concurrent_delete_are_not_applied(model):
    own(model, 'DISABLED')
    model.s.data['keepouts']={'session_id':'old-guard','revision':99,'polygons':[]}
    model.addKeepout(.1,.1,.3,.3)
    assert not commands(model,'keepouts')
    model.s.data['keepouts']={'session_id':'guard-1','revision':2,'polygons':[]}
    model.removeKeepout(0)
    assert not commands(model,'keepouts')
    assert '다시 선택' in model._notice
