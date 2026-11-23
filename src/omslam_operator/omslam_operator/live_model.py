"""ROS-backed map/logs interface with one fresh, revocable operator intent.

Only the Qt timer renews owner/manual leases. Service replies never resurrect
released input, and actions wait for the matching fresh owner state.
"""
import json
import math
import time
import uuid
from pathlib import Path
from urllib.parse import unquote, urlparse

from PySide6.QtCore import Property, Signal, Slot
from omslam_base.core import same_float_boundary
from omslam_navigation.operator_core import speed_axis_limits
from omslam_operator_msgs.msg import ManualCommand

from .model import ConsoleModel, number
from .presentation import presentation_snapshot


class LiveConsoleModel(ConsoleModel):
    notice = Signal(str)
    uiChanged = Signal()
    controlUiChanged = Signal()
    telemetryUiChanged = Signal()
    resourcesUiChanged = Signal()
    rgbUiChanged = Signal()
    depthUiChanged = Signal()
    logsChanged = Signal()
    goalsChanged = Signal()
    dataChanged = Signal()
    mapChanged = Signal(str)
    keepoutsChanged = Signal()

    TELEMETRY_FIELDS = frozenset(('leftRpm', 'rightRpm', 'roll', 'pitch', 'heading',
        'linearVelocity', 'angularVelocity', 'position', 'poseX', 'poseY', 'poseYaw'))
    RESOURCE_FIELDS = frozenset(('hostCpu', 'hostMemory', 'hostTemperature',
                                'robotCpu', 'robotMemory', 'robotTemperature'))
    CAMERA_FIELDS = frozenset(prefix + suffix for prefix in ('rgb', 'depth')
                             for suffix in ('Revision', 'Available', 'Fresh', 'Status'))

    def __init__(self, shared, initial_page=0, read_only=False):
        with shared.lock:
            shared.history_enabled = False
            shared.data['history'].clear()
        self.read_only = read_only
        self._ui = {}
        self._control_ui = {}
        self._telemetry_ui = {}
        self._resources_ui = {}
        self._rgb_ui = {}
        self._depth_ui = {}
        self._log_entries = []
        self._published_goals = []
        self._map_versions = {}
        self._map_pose = None
        self._pending = None
        self._request = None
        self._response_at = 0.
        self._held = False
        self._neutral_epoch = None
        self._map_inflight = None
        self._map_response_seen = None
        self._needs_initial_pose = False
        self._map_name = ''
        self._notice = ''
        self._notice_at = 0.
        self._last_debug = 0.
        self._connection_seen = None
        self._desired_rpm = None
        self._speed_dirty = False
        self._speed_request = None
        self._speed_response_seen = None
        self._result_notice_after = time.monotonic()
        self._goal_notice_seen = None
        self._keepout_zones = []
        self._keepout_polygons = []
        self._keepout_revision = 0
        self._keepout_available = False
        self._keepout_request = None
        super().__init__(shared, initial_page)
        # Legacy editing slots emit changed. Only actual goal changes propagate
        # to their list and the map; telemetry refreshes never emit this signal.
        self.changed.connect(self._refresh_goals)

    @Property('QVariantMap', notify=uiChanged)
    def ui(self):
        return self._ui

    @Property('QVariantMap', notify=controlUiChanged)
    def controlUi(self):
        return self._control_ui

    @Property('QVariantMap', notify=telemetryUiChanged)
    def telemetryUi(self):
        return self._telemetry_ui

    @Property('QVariantMap', notify=resourcesUiChanged)
    def resourcesUi(self):
        return self._resources_ui

    @Property('QVariantMap', notify=rgbUiChanged)
    def rgbUi(self):
        return self._rgb_ui

    @Property('QVariantMap', notify=depthUiChanged)
    def depthUi(self):
        return self._depth_ui

    @Property('QVariantList', notify=logsChanged)
    def logEntries(self):
        return self._log_entries

    @Property('QVariantList', notify=goalsChanged)
    def goals(self):
        return self.queue

    @Property('QVariantList', notify=keepoutsChanged)
    def keepoutZones(self):
        return self._keepout_zones

    def _sync_keepouts(self):
        with self.s.lock:
            snapshot = dict(self.s.data.get('keepouts', {}))
        available = bool(self._control_state.get('session_id') and
                         snapshot.get('session_id') == self._control_state.get('session_id'))
        polygons, rows = [], []
        if available:
            for index, polygon in enumerate(snapshot.get('polygons', [])):
                points = [[float(p['x']), float(p['y'])] for p in polygon['points']]
                if len(points) != 4 or not all(math.isfinite(v) for p in points for v in p):
                    available = False
                    break
                xs, ys = zip(*points)
                polygons.append(points)
                rows.append({'id':str(index), 'x_min':min(xs), 'y_min':min(ys),
                             'x_max':max(xs), 'y_max':max(ys)})
        if not available:
            polygons, rows = [], []
        self._keepout_available = available
        self._keepout_revision = snapshot.get('revision', 0) if available else 0
        self._keepout_polygons = polygons
        if self._keepout_zones != rows:
            self._keepout_zones = rows
            self.keepoutsChanged.emit()

    def _edit_keepouts(self, polygons):
        if self.queueBusy or self._pending or self._map_inflight or self._keepout_request:
            self._event('진행 중인 작업이 끝난 뒤 금지 구역을 편집하세요')
            return
        if not self._keepout_available:
            self._event('현재 지도의 금지 구역 상태를 기다리는 중입니다')
            return
        self.clearInput()
        payload = {'polygons':polygons, 'expected_revision':self._keepout_revision,
                   'session':self._control_state.get('session_id')}
        if self._begin('keepout', 'DISABLED', payload):
            self._advance_pending()

    @Slot(float, float, float, float)
    def addKeepout(self, x1, y1, x2, y2):
        if not all(math.isfinite(v) for v in (x1,y1,x2,y2)):
            return
        lo_x, hi_x = sorted((x1,x2)); lo_y, hi_y = sorted((y1,y2))
        if hi_x-lo_x < .05 or hi_y-lo_y < .05:
            self._event('금지 구역은 가로·세로 5 cm 이상으로 지정하세요')
            return
        self._sync_keepouts()
        self._edit_keepouts(self._keepout_polygons + [[[lo_x,lo_y],[hi_x,lo_y],[hi_x,hi_y],[lo_x,hi_y]]])

    @Slot(int)
    def removeKeepout(self, index):
        before = (self._keepout_revision, list(self._keepout_zones))
        self._sync_keepouts()
        if before != (self._keepout_revision, self._keepout_zones):
            self._event('금지 구역이 변경되었습니다. 목록에서 다시 선택하세요')
            return
        if 0 <= index < len(self._keepout_polygons):
            self._edit_keepouts(self._keepout_polygons[:index] + self._keepout_polygons[index+1:])

    @Property('QVariantMap', notify=dataChanged)
    def data(self):
        return self._data

    def _refresh_goals(self):
        if self.queue != self._published_goals:
            self._published_goals = [dict(goal) for goal in self.queue]
            self.goalsChanged.emit()

    def _publish_display(self, data, ui, map_versions, pose):
        """Publish independent presentation changes without marshalling dense data."""
        excluded = self.TELEMETRY_FIELDS | self.RESOURCE_FIELDS | self.CAMERA_FIELDS | {'logs'}
        groups = (
            ('_control_ui', {k: v for k, v in ui.items() if k not in excluded}, self.controlUiChanged),
            ('_telemetry_ui', {k: ui[k] for k in self.TELEMETRY_FIELDS}, self.telemetryUiChanged),
            ('_resources_ui', {k: ui[k] for k in self.RESOURCE_FIELDS}, self.resourcesUiChanged),
            ('_rgb_ui', {k: ui[k] for k in self.CAMERA_FIELDS if k.startswith('rgb')}, self.rgbUiChanged),
            ('_depth_ui', {k: ui[k] for k in self.CAMERA_FIELDS if k.startswith('depth')}, self.depthUiChanged),
            ('_log_entries', ui['logs'], self.logsChanged),
            ('_ui', ui, self.uiChanged),
            ('_data', data, self.dataChanged),
        )
        signals = []
        for attribute, value, signal in groups:
            if value != getattr(self, attribute):
                setattr(self, attribute, value)
                signals.append(signal)
        for signal in signals:
            signal.emit()
        self._refresh_goals()
        for key in self._map_versions.keys() | map_versions.keys():
            if self._map_versions.get(key) != map_versions.get(key):
                self.mapChanged.emit(key)
        self._map_versions = map_versions
        pose = tuple(pose) if pose else None
        if pose != self._map_pose:
            self._map_pose = pose
            self.mapChanged.emit('pose')

    def _event(self, message):
        self._notice, self._notice_at = str(message), time.monotonic()
        self.s.event(message)
        self.notice.emit(str(message))

    def _permitted(self):
        if self.read_only:
            self._event('관측 연결입니다. 이 연결에서는 구동·지도 변경을 지원하지 않습니다')
            return False
        return True

    def _fresh(self, topic='/operator/state', timeout=.3):
        with self.s.lock:
            item = self.s.data['topics'].get(topic, {})
        stamp = item.get('stamp')
        return (0. <= time.monotonic() - item.get('received', 0.) < timeout
                and stamp is not None and 0. <= self._ros_time() - stamp < timeout)

    def _ros_time(self):
        with self.s.lock:
            ros = self.s.data.get('ros_time', 0.)
            node = self.s.node
        if node is not None:
            try:
                ros = node.get_clock().now().nanoseconds * 1e-9
            except Exception:
                pass
        return ros

    def _owned(self, op=None):
        op = op if op is not None else self._control_state
        return (self._fresh() and bool(self.token) and op.get('owner_id') == self.owner
                and op.get('epoch') == self.epoch)

    def _ready(self, mode):
        op = self._control_state
        return (self._owned(op) and op.get('mode') == mode
                and op.get('phase') == ('ACTIVE' if mode != 'DISABLED' else 'READY'))

    @Slot(int)
    def setPage(self, value):
        self.clearInput()
        self._page = 0 if value == 0 else 1
        self.command('view', 'driving' if self._page == 0 else 'status')
        self.refresh()

    @Slot(bool)
    def setActive(self, value):
        self.active = value
        if not value:
            self.clearInput()

    @Slot()
    def clearInput(self):
        self._held = self.deadman = False
        self.manual_v = self.manual_w = 0.
        if self._pending and self._pending['kind'] == 'manual':
            self._pending = None
        self.sample_input()

    @Slot(float, float, bool)
    def manualIntent(self, v, w, pressed):
        if not pressed:
            self.clearInput()
            return
        if not self.active or self._page != 0 or not self._permitted():
            self.clearInput()
            return
        if not all(math.isfinite(value) for value in (v, w)):
            self.clearInput()
            self._event('유효하지 않은 직접 조종 입력입니다')
            return
        if self._map_inflight or (self._pending and self._pending['kind'] == 'map'):
            self.clearInput()
            self._event('지도 작업이 끝난 뒤 직접 조종하세요')
            return
        if v < 0 and not self._control_state.get('reverse_allowed', False):
            self.clearInput()
            self._event('후방 관측이 검증되지 않아 후진이 차단되어 있습니다')
            return
        self._held, self.manual_v, self.manual_w = True, float(v), float(w)
        if self._ready('MANUAL'):
            self.deadman = True
            self.sample_input()
            return
        if not self._pending or self._pending['kind'] != 'manual':
            self._halt_queue('직접 조종 전환')
            self._begin('manual', 'MANUAL')
        self._advance_pending()

    # Keep the inherited name useful for callers, with the live physical units.
    @Slot(float, float, bool)
    def manual(self, v, w, pressed):
        self.manualIntent(v, w, pressed)

    def sample_input(self):
        if self.read_only:
            with self.s.lock:
                self.s.latest_manual = None
            return
        if not self.token:
            return
        stamp = self.now()
        if stamp is None:
            return
        self.sequence += 1
        msg = ManualCommand()
        msg.token, msg.epoch, msg.sequence = self.token, self.epoch, self.sequence
        msg.command.header.stamp, msg.command.header.frame_id = stamp, 'base_footprint'
        active = (not self.read_only and self._held and self.active and self._page == 0
                  and self._ready('MANUAL'))
        # The guard requires one neutral sample after token rotation. It is
        # published on its own worker cycle before any still-held movement.
        if self._neutral_epoch != (self.token, self.epoch):
            self._neutral_epoch = (self.token, self.epoch)
            self._neutral_at = time.monotonic()
            active = False
        elif time.monotonic() - getattr(self, '_neutral_at', 0.) < .05:
            active = False
        msg.deadman = self.deadman = bool(active)
        if active:
            op = self._control_state
            linear, angular = float(op.get('linear_limit', 0.)), float(op.get('angular_limit', 0.))
            minimum = -linear if op.get('reverse_allowed', False) else 0.
            msg.command.twist.linear.x = max(minimum, min(linear, self.manual_v))
            msg.command.twist.angular.z = max(-angular, min(angular, self.manual_w))
        with self.s.lock:
            self.s.latest_manual = msg

    def _begin(self, kind, mode, payload=None):
        if not self._permitted():
            return False
        if self._keepout_request:
            self._event('금지 구역 편집 응답을 기다리는 중입니다')
            return False
        if not self._fresh():
            self._event('최신 제어 상태를 기다리는 중입니다. 연결 후 다시 요청하세요')
            self._held = False
            return False
        op = self._control_state
        if op.get('owner_id') not in ('', self.owner):
            self._event('다른 조작자가 제어 중입니다')
            self._held = False
            return False
        if not op.get('operator_enabled', False):
            self._event('이 ROS 연결은 관측만 지원합니다')
            self._held = False
            return False
        self._pending = {'kind': kind, 'mode': mode, 'payload': payload,
                         'started': time.monotonic()}
        self._result_notice_after = time.monotonic()
        return True

    def _send_control(self, command, mode):
        request_id = uuid.uuid4().hex
        self._request = {'id': request_id, 'command': command, 'mode': mode,
                         'sent': time.monotonic(), 'session': self._control_state.get('session_id')}
        self.command('control', {'command': command, 'owner_id': self.owner,
            'token': self.token, 'epoch': self.epoch, 'request_id': request_id, 'mode': mode})

    def _drop_control(self, reason):
        had_intent = bool(self.token or self._pending or self._held or self._request)
        self.token = ''
        self._held = self.deadman = False
        self.manual_v = self.manual_w = 0.
        self._pending = self._request = None
        self._speed_request = None
        self._speed_dirty = False
        self._neutral_epoch = None
        self.pending_mode = None
        self._halt_queue('제어 연결 종료')
        with self.s.lock:
            self.s.latest_manual = self.s.latest_heartbeat = None
            self.s.drop_queued_requests()
            self.s.cancel_unsent_goal()
            self.s.controls = type(self.s.controls)(
                ((kind, data) for kind, data in self.s.controls if kind in ('view', 'stop')),
                maxlen=32)
        if had_intent:
            self._event(reason)

    def _sync_control(self, response, op, state_received):
        if self.read_only:
            self.token = ''
            self._held = self.deadman = False
            self._pending = self._request = None
            return
        session = op.get('session_id')
        if self._fresh() and session:
            if self.guard_session and session != self.guard_session:
                self._drop_control('제어 서버가 다시 시작되어 관측 모드로 연결합니다')
                # A guard session owns one map. SLAM grid revisions within it
                # retain the same map frame; a replacement backend does not.
                self.queue_running = self.queue_waiting = False
                self.queue_index = None
                self.queue.clear()
                self._map_name = ''
                self._needs_initial_pose = False
                self._refresh_goals()
            self.guard_session = session
        if not self._fresh():
            if self.token or self._pending or self._request:
                self._drop_control('제어 상태 수신이 끊겨 입력을 해제했습니다. 다시 조작해야 움직입니다')
            return
        request = self._request
        if request and response.get('_request_id') == request['id']:
            self._request = None
            if response.get('success') and request['session'] == self.guard_session:
                self.token, self.epoch = response.get('token', ''), response.get('epoch', 0)
                self._response_at = response.get('_received', time.monotonic())
                self._neutral_epoch = None
            else:
                self._event(response.get('message', '제어 요청이 거부되었습니다'))
                self._pending = None
                self._speed_dirty = False
                self._held = False
                self._halt_queue('제어 요청 실패')
        elif request and time.monotonic() - request['sent'] > 8.:
            self._drop_control('제어 요청 응답 시간 초과. 다시 요청하세요')
            return
        if self.token and state_received > self._response_at:
            if (op.get('owner_id') != self.owner or op.get('epoch') != self.epoch):
                # A transition rotates the token before its service reply arrives.
                # Retain the old token only while our exact request is outstanding.
                if not self._request:
                    self._drop_control('제어권이 종료되어 관측 모드로 전환했습니다')

    def _advance_pending(self):
        pending = self._pending
        if not pending or self._request or self._map_inflight or self._keepout_request:
            return
        if pending['kind'] == 'manual' and not (self._held and self.active and self._page == 0):
            self._pending = None
            return
        if time.monotonic() - pending['started'] > 15.:
            self._pending = None
            self._speed_dirty = False
            self._held = False
            self._event('요청 준비 시간 초과. 상태를 확인한 뒤 다시 요청하세요')
            return
        if not self._fresh():
            return
        op = self._control_state
        if op.get('owner_id') not in ('', self.owner):
            self._drop_control('다른 조작자가 제어 중입니다')
            return
        if not self.token:
            if op.get('phase') == 'TRANSITION' or op.get('owner_id'):
                return
            self._send_control('acquire', 'DISABLED')
            return
        if not self._owned(op):
            return
        mode = pending['mode']
        if op.get('mode') != mode:
            if op.get('phase') != 'TRANSITION':
                self._send_control('set_mode', mode)
            return
        # DISABLED can be BLOCKED when AMCL is awaiting initial pose. Ownership,
        # fresh measured stop and inactive base still authorize map operations.
        if mode == 'DISABLED':
            if op.get('phase') == 'TRANSITION' or op.get('base_active') or not self._queue_stopped(op):
                return
        elif not self._ready(mode):
            return
        if pending['kind'] == 'manual':
            self._pending = None
            self.sample_input()
        elif pending['kind'] == 'route':
            if self._speed_dirty or self._speed_request:
                return
            self._pending = None
            super().runQueue()
            self._event('경로 이동을 시작했습니다')
        elif pending['kind'] == 'map':
            self._pending = None
            payload = pending['payload']
            request_id = uuid.uuid4().hex
            self._map_inflight = dict(payload, id=request_id, sent=time.monotonic())
            self.command('maps', dict(payload, session_id=self.owner, token=self.token,
                                      epoch=self.epoch, _request_id=request_id))
        elif pending['kind'] == 'keepout':
            self._pending = None
            payload = pending['payload']
            if payload['session'] != op.get('session_id'):
                self._event('지도가 변경되어 금지 구역 편집을 취소했습니다')
                return
            request_id = uuid.uuid4().hex
            self._keepout_request = {'id':request_id,'sent':time.monotonic()}
            self.command('keepouts',dict(payload,token=self.token,epoch=self.epoch,_request_id=request_id))
        else:
            self._pending = None

    @Slot(float, float, float)
    def addWaypoint(self, x, y, yaw):
        if self.queueBusy or self._pending or self._map_inflight:
            self._event('진행 중인 작업이 끝난 뒤 목적지를 편집하세요')
            return
        self.setGoal(x, y, yaw)
        if not self._goal['valid']:
            self._event(self._goal['reason'])
            return
        self.queue.append(dict(self._goal, uid=uuid.uuid4().hex, status='대기'))
        self._result_notice_after = time.monotonic()
        self._event(f'목적지 {len(self.queue)} 추가')
        self.changed.emit()

    @Slot(int, float, float, float)
    def editWaypoint(self, index, x, y, yaw):
        if self.queueBusy or self._pending or not 0 <= index < len(self.queue):
            return
        self.setGoal(x, y, yaw)
        if not self._goal['valid']:
            self._event(self._goal['reason'])
            return
        self.queue[index] = dict(self._goal, uid=self.queue[index]['uid'], status='대기')
        self._result_notice_after = time.monotonic()
        self._event(f'목적지 {index + 1} 수정')
        self.changed.emit()

    @Slot(int, int)
    def moveWaypoint(self, source, destination):
        if (not self.queueBusy and not self._pending
                and 0 <= source < len(self.queue) and 0 <= destination < len(self.queue)):
            self.queue.insert(destination, self.queue.pop(source))
            self._result_notice_after = time.monotonic()
            self._event('목적지 순서 변경')
            self.changed.emit()

    @Slot(int)
    def removeGoal(self, index):
        if not self.queueBusy and not self._pending and 0 <= index < len(self.queue):
            super().removeGoal(index)
            self._result_notice_after = time.monotonic()
            self._event(f'목적지 {index + 1} 삭제')

    @Slot()
    def startRoute(self):
        if not self._permitted() or self.queueBusy or self._pending or self._map_inflight:
            return
        if not self.queue:
            self._event('지도에 목적지를 추가하세요')
            return
        if (self._needs_initial_pose or (self._control_state.get('localization_mode') == 'amcl'
                                        and not self._control_state.get('localization_initialized'))):
            self._event('불러온 지도에 현재 로봇 위치·방향을 지정하세요')
            return
        for goal in self.queue:
            self.setGoal(goal['x'], goal['y'], goal['yaw'])
            if not self._goal['valid']:
                self._event(self._goal['reason'])
                return
        self.clearInput()
        if self._begin('route', 'AUTO'):
            self._advance_pending()

    @Slot()
    def runQueue(self):
        self.startRoute()

    @Slot()
    def acquireControl(self):
        if not self.token and not self._pending and self._begin('owner', 'DISABLED'):
            self._advance_pending()

    @Slot(str)
    def control(self, mode):
        if mode not in ('DISABLED', 'MANUAL', 'AUTO') or not self._permitted():
            return
        self.clearInput()
        self._halt_queue('모드 전환')
        if self._begin('mode', mode):
            self._advance_pending()

    @Slot()
    def cancel(self):
        if self._permitted():
            self._pending = None
            self._held = False
            super().cancel()

    @Slot(float, float, float)
    def speed(self, linear, angular, percent):
        self.setSpeedLimit(linear)

    @Slot(str)
    def mapOperation(self, operation):
        if operation in ('save', 'localize', 'slam', 'initialpose'):
            self._map(operation, self._map_path, self._goal['x'], self._goal['y'], self._goal['yaw'])

    def sendGoal(self, goal):
        if self.read_only or not self._ready('AUTO'):
            self._event('최신 자율 주행 준비 상태가 없어 목표를 전송하지 않았습니다')
            return False
        return super().sendGoal(goal)

    @Slot(float)
    def setSpeedLimit(self, linear):
        maximum = float(self._control_state.get('max_linear_velocity', 0.))
        max_rpm = float(self._control_state.get('max_wheel_rpm', 0.))
        min_rpm = float(self._control_state.get('min_wheel_rpm', 0.))
        rpm = max_rpm * (linear / maximum) if maximum > 0. else 0.
        self.setRpmLimit(min_rpm if same_float_boundary(rpm, min_rpm) else rpm)

    @Slot(float)
    def setRpmLimit(self, rpm):
        if not self._permitted():
            return
        minimum = float(self._control_state.get('min_wheel_rpm', 0.))
        maximum = float(self._control_state.get('max_wheel_rpm', 0.))
        if minimum <= 0. or maximum < minimum:
            self._event('서버에서 RPM 범위를 확인하는 중입니다')
            return
        if not minimum <= rpm <= maximum:
            self._event(f'RPM을 {minimum:g}–{maximum:g} 범위로 입력하세요')
            return
        if not self._owned():
            pending_speed = self._pending and self._pending['kind'] == 'speed'
            if not pending_speed and (self._pending or self._request or not self._begin('speed', 'DISABLED')):
                return
            if not self._fresh() or self._control_state.get('owner_id') not in ('', self.owner):
                self._drop_control('제어 상태가 변경되어 속도 요청을 취소했습니다')
                return
        self._desired_rpm, self._speed_dirty = float(rpm), True
        self._advance_pending()
        self._apply_speed()

    def _apply_speed(self):
        if not self._speed_dirty or self._speed_request or self._request or not self._owned():
            return
        op = self._control_state
        if op.get('phase') == 'TRANSITION':
            return
        rpm = self._desired_rpm
        min_rpm = float(op.get('min_wheel_rpm', 0.))
        max_rpm = float(op.get('max_wheel_rpm', 0.))
        if not 0. < min_rpm <= rpm <= max_rpm:
            self._speed_dirty = False
            self._pending = None
            self._event('서버의 속도 범위가 변경되었습니다. 속도를 다시 선택하세요')
            return
        fraction = rpm / max_rpm
        linear, angular = speed_axis_limits(float(op['max_linear_velocity']),
                                           float(op['max_angular_velocity']),
                                           float(op['min_rotation_velocity']), fraction)
        request_id = uuid.uuid4().hex
        self._speed_request = {'id': request_id, 'sent': time.monotonic()}
        self._speed_dirty = False
        self.command('speed', {'token': self.token, 'epoch': self.epoch,
            'request_id': request_id, 'linear_limit': linear,
            'angular_limit': angular, 'speed_percent': fraction * 100.,
            'wheel_rpm_limit': rpm})

    @staticmethod
    def _path(path):
        if path.startswith('file:'):
            path = unquote(urlparse(path).path)
        return str(Path(path).expanduser()) if path.strip() else ''

    def _map(self, operation, path='', x=0., y=0., yaw=0.):
        if not self._permitted() or self._map_inflight or self._pending:
            return
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self._event('위치·방향은 유한한 수여야 합니다')
            return
        path = self._path(path)
        if operation in ('save', 'localize') and not path:
            self._event('지도 파일 경로를 입력하세요')
            return
        self.clearInput()
        self._halt_queue('지도 작업')
        if self._begin('map', 'DISABLED', {'operation': operation, 'path': path,
                                         'x': float(x), 'y': float(y), 'yaw': float(yaw)}):
            self._event({'save': '지도 저장 준비', 'localize': '지도 불러오기 준비',
                'slam': '새 지도 준비', 'initialpose': '초기 위치 적용 준비'}[operation])
            self._advance_pending()

    @Slot(str)
    def saveMap(self, path):
        self._map('save', path)

    @Slot(str)
    def loadMap(self, path):
        self._map('localize', path)

    @Slot()
    def newMap(self):
        self._map('slam')

    @Slot(float, float, float)
    def setInitialPose(self, x, y, yaw):
        self._map('initialpose', x=x, y=y, yaw=yaw)

    @Slot()
    def stop(self):
        self.clearInput()
        self._pending = self._request = None
        self._speed_dirty = False
        self._speed_request = None
        if self.read_only:
            self._event('관측 연결입니다. 로봇 정지 요청은 제어 연결에서 실행하세요')
            return
        super().stop()

    @Slot()
    def shutdown(self):
        self.clearInput()
        self._pending = self._request = None
        self._halt_queue('앱 종료')
        if self.token and not self.read_only:
            super().stop()

    def tick(self):
        super().tick()
        self._sync_keepouts()
        with self.s.lock:
            map_response = dict(self.s.data.get('map_response', {}))
            speed_response = dict(self.s.data.get('speed_response', {}))
            goal_result = dict(self.s.data.get('goal_result', {}))
            keepout_response = dict(self.s.data.get('keepout_response', {}))
        if self._keepout_request and keepout_response.get('_request_id') == self._keepout_request['id']:
            self._event(keepout_response.get('message', '금지 구역 편집 응답'))
            self._keepout_request = None
        if self._keepout_request and time.monotonic()-self._keepout_request['sent'] > 5.:
            self._keepout_request = None
            self._event('금지 구역 응답 시간 초과. 지도에 표시된 서버 상태를 확인하세요')
        result_serial = repr(goal_result)
        if goal_result and self._goal_notice_seen != result_serial:
            self._goal_notice_seen = result_serial
            self._event(goal_result.get('message', '목표 완료' if goal_result.get('success') else '목표 종료'))
        if self._map_inflight and map_response.get('_request_id') == self._map_inflight['id']:
            operation = self._map_inflight['operation']
            if map_response.get('success'):
                if operation in ('localize', 'slam'):
                    self.queue.clear()
                    self.queue_running = self.queue_waiting = False
                    self.queue_index = None
                    self._needs_initial_pose = operation == 'localize'
                    self._map_name = (Path(self._map_inflight['path']).name if operation == 'localize' else '새 지도 · SLAM')
                elif operation == 'save':
                    self._map_name = Path(self._map_inflight['path']).name
            self._event(map_response.get('message', '지도 작업 응답'))
            self._map_inflight = None
        if self._map_inflight and time.monotonic() - self._map_inflight['sent'] > 90.:
            self._map_inflight = None
            self._event('지도 작업 응답 시간 초과. 시스템 로그를 확인하세요')
        if self._speed_request and speed_response.get('_request_id') == self._speed_request['id']:
            if not speed_response.get('success'):
                self._event(speed_response.get('message', '속도 제한 요청 실패'))
                self._pending = None
            self._speed_request = None
        if self._speed_request and time.monotonic() - self._speed_request['sent'] > 4.:
            self._speed_request = None
            self._pending = None
            self._event('속도 제한 응답 시간 초과')
        if (self._needs_initial_pose and self._fresh() and self._control_state.get('auto_ready')
                and self._control_state.get('localization_mode') == 'amcl'
                and self._control_state.get('localization_initialized')):
            self._needs_initial_pose = False
            self._event('초기 위치와 주행 준비가 확인되었습니다')
        self._apply_speed()
        self._advance_pending()

    def _resources(self, diagnostics, now):
        values = {name: '—' for name in ('hostCpu', 'hostMemory', 'hostTemperature',
                                        'robotCpu', 'robotMemory', 'robotTemperature')}
        for row in diagnostics.values():
            if not row.get('name', '').startswith('system_monitor/') or now - row.get('received', 0.) > 3.:
                continue
            fields = {item['key']: item['value'] for item in row.get('values', [])}
            if fields.get('valid') != 'true':
                continue
            role = fields.get('host_role')
            prefix = 'robot' if role == 'robot' else 'host' if role in ('host', 'operator_pc', 'simulation_host') else None
            if prefix is None:
                continue
            category = row['name'].rsplit('/', 1)[-1]
            try:
                if category == 'cpu':
                    values[prefix + 'Cpu'] = number(float(fields['busy_percent']), 0) + '%'
                elif category == 'memory':
                    values[prefix + 'Memory'] = number(float(fields['used_percent']), 0) + '%'
                elif category == 'temperature':
                    temperatures = [float(v) for k, v in fields.items() if k.endswith('.celsius') and math.isfinite(float(v))]
                    if temperatures:
                        values[prefix + 'Temperature'] = number(max(temperatures), 0) + '°C'
            except (ValueError, KeyError):
                continue
        return values

    def refresh(self):
        now = time.monotonic()
        with self.s.lock:
            d = presentation_snapshot(self.s.data, 3 if self._page == 1 else 0, 0, 1)
            diagnostics = dict(self.s.data['diagnostics'])
            map_ready = bool(self.s.map_layers.get('map'))
            map_versions = dict(self.s.map_revisions)
            # Include presence, so removed grids invalidate the rendered layer.
            for key in ('map', 'global', 'local'):
                grid = self.s.map_layers.get(key)
                map_versions[key] = grid.get('revision', id(grid)) if grid is not None else None
        op = d.get('operator', {})
        fresh = self._fresh()
        connected = fresh or any(self._fresh(topic, .8) for topic in ('/odom', '/imu/data', '/scan'))
        pose = d.get('pose') if now - d.get('pose_received', 0.) < .5 else None
        if self._connection_seen is not None and self._connection_seen != fresh:
            self.s.event('제어 상태 연결됨' if fresh else '제어 상태 연결 끊김 · 마지막 관측은 지연 상태입니다')
        self._connection_seen = fresh
        imu = d.get('imu', {})
        rpy = imu.get('rpy', []) if imu.get('valid') and self._fresh('/imu/data', .5) else []
        motor, rpm = d.get('motor', {}), []
        if self._fresh('/base/motor_state', .5):
            valid = set(motor.get('valid_fields', []))
            for field in ('measured_rpm', 'encoder_rpm'):
                if field in valid and len(motor.get(field, [])) >= 2:
                    rpm = motor[field]
                    break
        if not rpm and self._fresh('/joint_states', .5):
            rpm = [d.get('joints', {}).get(name, {}).get('rpm') for name in ('left_wheel_joint', 'right_wheel_joint')]
        velocity = d.get('velocity', {}).get('/odom', {}) if self._fresh('/odom', .3) else {}
        operation = self._map_inflight or self._pending
        mode = op.get('mode', 'DISABLED') if fresh else 'DISABLED'
        phase = op.get('phase', '') if fresh else ''
        owner_busy = fresh and op.get('owner_id') not in ('', self.owner)
        reconnect_pose_needed = (fresh and map_ready and op.get('localization_mode') == 'amcl'
                                 and not op.get('localization_initialized'))
        needs_pose = self._needs_initial_pose or reconnect_pose_needed
        result = d.get('goal_result', {})
        error = str(result.get('message', '')).lower()
        failed_goal = (fresh and not operation and not self.queue_running
            and result.get('_received', 0.) >= self._result_notice_after
            and not result.get('success', False) and result.get('status') == 6)
        no_path = (failed_goal and (result.get('error_code') == 208 or any(text in error for text in ('no valid path', 'no path', 'failed to create a plan', 'failed to create plan'))))
        if not connected:
            movement = 'disconnected' if d.get('topics') else 'not_connected'
        elif not fresh or operation or phase == 'TRANSITION':
            movement = 'checking'
        elif needs_pose:
            movement = 'pose_required'
        elif not map_ready:
            movement = 'map_required'
        elif no_path:
            movement = 'no_path'
        elif failed_goal or phase == 'BLOCKED' or owner_busy or self.read_only:
            movement = 'blocked'
        else:
            movement = 'ready'
        reason = ('관측 연결 · 실물 구동 미허용' if self.read_only else
                  op.get('reason', '제어 서버를 기다리는 중입니다') if fresh else
                  '최신 제어 상태를 기다리는 중입니다')
        if failed_goal:
            reason = result.get('message', reason)
        elif needs_pose:
            reason = '현재 로봇의 위치·방향을 지도에서 지정하세요. AMCL 위치 관측을 기다리는 중입니다'
        status = self._notice if now - self._notice_at < 8. else reason
        ui = {'connected': connected, 'controlFresh': fresh, 'ownerSelf': self._owned(op),
              'ownerBusy': bool(owner_busy), 'readOnly': self.read_only,
              'mode': mode, 'phase': phase, 'reason': reason, 'statusText': status,
              'movementState': movement, 'movementLabel': status,
              'operationPending': bool(operation or self._request or self._keepout_request),
              'keepoutBusy': bool(self._keepout_request or (self._pending and self._pending['kind']=='keepout')),
              'keepoutAvailable': bool(self._keepout_available and fresh and map_ready and not self.read_only and not owner_busy),
              'maxRpm': float(op.get('max_wheel_rpm', 0.)),
              'rpmLimit': float(op.get('wheel_rpm_limit', 0.)),
              'minRpm': float(op.get('min_wheel_rpm', 0.)), 'rpmStep': .1,
              'canSetInitialPose': fresh and map_ready and op.get('localization_mode') == 'amcl',
              'mapReady': map_ready, 'mapName': self._map_name or ('ROS 지도' if map_ready else '지도 대기'),
              'needsInitialPose': bool(needs_pose), 'routeBusy': bool(self.queueBusy or (self._pending and self._pending['kind'] == 'route')),
              'routeRunning': self.queue_running, 'manualActive': bool(self.deadman),
              'linearLimit': float(op.get('linear_limit', .1)), 'angularLimit': float(op.get('angular_limit', .3)),
              'maxLinearVelocity': float(op.get('max_linear_velocity', .2)),
              'maxAngularVelocity': float(op.get('max_angular_velocity', .6)),
              'reverseAllowed': fresh and bool(op.get('reverse_allowed', False)) and not self.read_only,
              'leftRpm': number(rpm[0], 1) if len(rpm) > 0 else '—',
              'rightRpm': number(rpm[1], 1) if len(rpm) > 1 else '—',
              'roll': number(math.degrees(rpy[0]), 1) if len(rpy) == 3 else '—',
              'pitch': number(math.degrees(rpy[1]), 1) if len(rpy) == 3 else '—',
              'heading': number(math.degrees(pose[2]), 1) if pose else '—',
              'linearVelocity': number(velocity.get('v')), 'angularVelocity': number(velocity.get('w')),
              'position': f'{number(pose[0])}, {number(pose[1])} m' if pose else '—',
              'poseX': pose[0] if pose else None, 'poseY': pose[1] if pose else None,
              'poseYaw': pose[2] if pose else None}
        ui.update(self._resources(diagnostics, now))
        debug_directory = self.s.path_debug_directory
        debug = d.get('path_debug', {})
        debug_fresh = now - debug.get('_received', 0.) < 3.
        debug_state = debug.get('status', 'waiting') if debug_fresh else 'waiting'
        if debug_fresh and debug.get('dropped', 0):
            debug_state = 'error'
        debug_text = {'recording': '경로 기록 중', 'error': '경로 기록 오류',
                      'stopped': '경로 기록 종료'}.get(debug_state, '경로 기록 대기')
        ui.update(pathDebugEnabled=bool(debug_directory), pathDebugState=debug_state,
                  pathDebugText=debug_text, pathDebugDirectory=debug_directory,
                  pathDebugDetail=debug_directory + ('\n' + str(debug['error']) if debug.get('error') else '')
                      + ('\n기록 연결 확인 중' if debug_directory and not debug_fresh else ''))
        ui['speedOptions'] = [ui['maxLinearVelocity'] * ratio for ratio in (.25, .5, 1.)]
        ui['speedIndex'] = min(range(3), key=lambda index: abs(ui['speedOptions'][index] - ui['linearLimit']))
        ros = self._ros_time()
        for key in ('rgb', 'depth'):
            camera = d.get('cameras', {}).get(key, {})
            camera_fresh = (0. <= now - camera.get('received', 0.) < .8
                            and 0. <= ros - camera.get('stamp', -999.) < .8)
            ui.update({key + 'Revision': camera.get('revision', 0),
                key + 'Available': bool(camera), key + 'Fresh': camera_fresh,
                key + 'Status': '수신 중' if camera_fresh else '수신 지연' if camera else '영상 대기'})
        if self._page == 1 and now - self._last_debug > 2.:
            self._last_debug = now
            # No owner tokens or request payloads belong in operator logs.
            details = []
            groups = [('구동', ('motor', 'joints')), ('IMU', ('imu', 'imu_base')),
                ('속도', ('velocity',)), ('주행', ('navigation', 'goal_feedback')),
                ('인지', ('memory', 'health')), ('라이다', ('scan_info',)),
                ('카메라', ('cameras', 'intrinsics', 'depth_info', 'cloud_info'))]
            for title, keys in groups:
                snapshot = {key: d.get(key) for key in keys if d.get(key)}
                if snapshot:
                    details.append('[상세][' + title + '] ' + json.dumps(snapshot, ensure_ascii=False, separators=(',', ':')))
            snapshot = {'host': d.get('host'), 'domain': d.get('domain'), 'diagnostics': diagnostics,
                'freshTopics': {topic: self._fresh(topic, 3. if topic == '/diagnostics' else .8) for topic in d['topics']}}
            details.append('[상세][시스템] ' + json.dumps(snapshot, ensure_ascii=False, separators=(',', ':')))
            self._detail_logs = [dict(time=time.strftime('%H:%M:%S'), count=1, level='상세', message=line) for line in details]
        with self.s.lock:
            ui['logs'] = ([dict(row, level='상세' if row['message'].startswith('[상세]') else '정보')
                for row in self.s.data['events'][:200]] + getattr(self, '_detail_logs', []) if self._page == 1 else [])
        d.update(operatorConnected=fresh, connected=connected, ownerSelf=self.owner,
                 hasToken=bool(self.token), goalActive=self.s.goal_active,
                 actualV=ui['linearVelocity'], actualW=ui['angularVelocity'], operatorReason=reason)
        self._publish_display(d, ui, map_versions, pose)
