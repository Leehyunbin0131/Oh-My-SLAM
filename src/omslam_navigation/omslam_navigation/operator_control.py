"""Operator ROS endpoints hosted by the existing navigation_guard node."""

from collections import OrderedDict
from copy import deepcopy
import math
import time
import uuid

from action_msgs.msg import GoalStatus
from lifecycle_msgs.msg import State, Transition
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from omslam_operator_msgs.action import OperatorNavigate
from omslam_operator_msgs.msg import ManualCommand, OperatorState, OwnerHeartbeat
from omslam_operator_msgs.srv import Control, SetSpeedLimit
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def values(message):
    t = message.twist
    return (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)


class OperatorControl:
    MAP_LEASE_SECONDS = 15.

    def __init__(self, node):
        self.node, self.core = node, node.core
        self.session_id = str(uuid.uuid4())
        self.pending = None
        self.map_reservation = None
        self.session_start_ns = node.now_ns()
        self.localization_initialized = False
        self.pending_pose = None
        self.activating = False
        self.last_report = 0.0
        self.completed = OrderedDict()
        self.inflight = {}
        self.waiters = []
        self.proxy_reserved = False
        self.proxy_handle = None
        self.group = ReentrantCallbackGroup()
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = node.create_publisher(OperatorState, '/operator/state', transient)
        if node.p['localization_mode'] == 'amcl':
            self.pose_sub = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                                      self.amcl_pose, 1)
        self.speed_pub = node.create_publisher(SpeedLimit, '/speed_limit', 1)
        self.control_service = node.create_service(Control, '/operator/control', self.control,
                                                    callback_group=self.group)
        self.stop_service = node.create_service(Trigger, '/operator/stop', self.stop,
                                                 callback_group=self.group)
        self.speed_service = node.create_service(SetSpeedLimit, '/operator/speed_limit', self.speed,
                                                  callback_group=self.group)
        self.manual_sub = node.create_subscription(ManualCommand, '/operator/manual', self.manual, 1)
        self.heartbeat_sub = node.create_subscription(OwnerHeartbeat, '/operator/heartbeat', self.heartbeat, 1)
        self.nav_client = ActionClient(node, NavigateToPose, '/navigate_to_pose', callback_group=self.group)
        self.action = ActionServer(node, OperatorNavigate, '/operator/navigate',
                                   execute_callback=self.execute_goal, goal_callback=self.accept_goal,
                                   cancel_callback=self.cancel_goal, callback_group=self.group)

    def authorized(self, token, epoch):
        return self.core.authorized(token, epoch, self.node.now_ns(), time.monotonic())

    def amcl_pose(self, message):
        now = self.node.now_ns()
        stamp = ns(message.header.stamp)
        pose = message.pose.pose
        q = pose.orientation
        if (message.header.frame_id == 'map' and stamp > 0 and stamp >= self.session_start_ns
                and -300_000_000 <= now - stamp < 1_000_000_000
                and all(math.isfinite(x) for x in (pose.position.x, pose.position.y))
                and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) < .02):
            self.pending_pose = (stamp, time.monotonic())
            self.confirm_localization(now)

    def confirm_localization(self, now):
        if self.pending_pose is None:
            return
        stamp, received = self.pending_pose
        if time.monotonic() - received >= 1.:
            self.pending_pose = None
        elif 0 <= now - stamp < 1_000_000_000:
            self.localization_initialized = True
            self.pending_pose = None

    def map_reserved(self):
        if self.map_reservation and time.monotonic() >= self.map_reservation[1]:
            self.map_reservation = None
        return self.map_reservation is not None

    def map_command(self, request, response):
        key = (request.owner_id, request.token, request.epoch, request.request_id)
        reserved = self.map_reserved()
        matches = reserved and key == self.map_reservation[0]
        if request.command == 'begin_map':
            if not matches and (reserved or self.pending or not self.authorized(request.token, request.epoch)
                                or request.owner_id != self.core.owner_id):
                response.message = '지도 작업을 예약할 수 없습니다. 구동 상태와 제어권을 확인하세요.'
                return response
            if not matches:
                self.core.disable()
                self.node.publish_zero()
        elif not matches:
            response.message = '지도 작업 예약이 만료되었거나 세션이 변경됐습니다.'
            return response
        self.map_reservation = (None if request.command == 'end_map' else
                                (key, time.monotonic() + self.MAP_LEASE_SECONDS))
        response.success = True
        response.message = '지도 작업 예약 해제' if request.command == 'end_map' else '지도 작업 예약 확인'
        return response

    def heartbeat(self, message):
        self.core.heartbeat(message.token, message.epoch, message.sequence, ns(message.header.stamp),
                            self.node.now_ns(), time.monotonic())

    def manual(self, message):
        stamp, wall, ros_ns = ns(message.command.header.stamp), time.monotonic(), self.node.now_ns()
        authorized = self.core.authorized(message.token, message.epoch, ros_ns, wall)
        accepted = self.core.manual(message.token, message.epoch, message.sequence, message.deadman,
                                    stamp, message.command.header.frame_id, values(message.command), ros_ns, wall)
        if accepted:
            output = deepcopy(message.command)
            limited = self.core.limit_values(values(output))
            output.twist.linear.x, output.twist.angular.z = limited[0], limited[5]
            self.node.last_original_stamp = stamp
            self.node.requested_pub.publish(output)
            if output.twist.linear.x == 0 and output.twist.angular.z == 0:
                self.node.publish_zero()
        elif authorized:
            self.node.publish_zero()

    def response_values(self, success, message):
        return self.core.control_result(success, message)

    @staticmethod
    def fill_response(response, result):
        response.success, response.message, response.token, response.epoch, response.mode = result
        return response

    async def control(self, request, response):
        if request.command == 'validate':
            good = self.authorized(request.token, request.epoch)
            return self.fill_response(response, self.response_values(good, 'owner valid' if good else 'owner expired or invalid'))
        if not request.request_id or len(request.request_id) > 128:
            return self.fill_response(response, self.response_values(False, 'a bounded request_id is required'))
        if request.command in ('begin_map', 'hold_map', 'end_map'):
            return self.map_command(request, response)
        if self.map_reserved() or (request.command == 'acquire' and self.pending):
            return self.fill_response(response, self.response_values(False, 'a stop, mode change or map operation is pending'))
        key = (request.command, request.owner_id, request.token, request.epoch, request.request_id)
        if key in self.completed:
            previous = self.completed[key]
            if (request.command in ('acquire', 'set_mode') and previous[0]
                    and not self.authorized(previous[2], previous[3])):
                return self.fill_response(response, self.response_values(False, 'previous ownership has ended'))
            return self.fill_response(response, previous)
        if key in self.inflight:
            return self.fill_response(response, await self.inflight[key])
        future = Future()
        self.inflight[key] = future
        result = None
        try:
            if request.command == 'acquire':
                success, message = self.core.acquire(request.owner_id, self.node.now_ns(), time.monotonic())
                result = self.response_values(success, message)
            elif request.command not in ('release', 'set_mode'):
                result = self.response_values(False, 'unsupported control command')
            elif not self.authorized(request.token, request.epoch):
                result = self.response_values(False, 'owner expired or invalid')
            elif self.pending is not None:
                result = self.response_values(False, 'a stop or mode transition is pending')
            else:
                target = 'DISABLED' if request.command == 'release' else request.mode
                success, message = self.core.begin_transition(target)
                if not success:
                    result = self.response_values(False, message)
                else:
                    self.start_pending(request.command, target, future)
                    result = await future
            if not future.done():
                future.set_result(result)
            self.completed[key] = result
            while len(self.completed) > 64:
                self.completed.popitem(last=False)
            return self.fill_response(response, result)
        finally:
            self.inflight.pop(key, None)

    def start_pending(self, kind, mode, future):
        self.activating = False
        self.node.publish_zero()
        self.node.cancel_attempted = None
        self.pending = {'kind': kind, 'mode': mode, 'future': future,
                        'deadline': time.monotonic() + 6.0, 'stage': 'stop'}

    def finish_pending(self, success, message):
        pending, self.pending = self.pending, None
        self.activating = False
        if pending and not pending['future'].done():
            pending['future'].set_result(self.response_values(success, message))

    async def stop(self, request, response):
        self.finish_pending(False, 'superseded by stop request')
        self.core.disable()
        self.core.forget_owner()
        self.node.publish_zero()
        future = Future()
        self.start_pending('stop', 'DISABLED', future)
        result = await future
        response.success, response.message = result[:2]
        return response

    def speed(self, request, response):
        valid = self.authorized(request.token, request.epoch)
        rpm = request.wheel_rpm_limit or self.core.geometry.max_rpm * request.speed_percent / 100.
        numeric = (request.linear_limit, request.angular_limit, request.speed_percent)
        valid = valid and all(math.isfinite(x) for x in numeric) and (
            0 < request.linear_limit <= self.core.limits.max_vx and
            0 < request.angular_limit <= self.core.limits.max_wz and
            1.0 <= request.speed_percent <= 100.0 and
            math.isfinite(rpm) and 1.0 <= rpm <= self.core.geometry.max_rpm and
            abs(rpm / self.core.geometry.max_rpm * 100. - request.speed_percent) < 1e-6)
        if valid:
            self.core.linear_limit, self.core.angular_limit, self.core.speed_percent = numeric
            self.core.wheel_rpm_limit = rpm
            self.publish_speed()
            # Already buffered downstream samples remain stamped, and their
            # final output is independently capped by the same current limits.
        response.success = bool(valid)
        response.message = 'limits accepted; Nav2 topic has no application ACK' if valid else 'invalid owner or limits'
        response.linear_limit = self.core.linear_limit
        response.angular_limit = self.core.angular_limit
        response.speed_percent = self.core.speed_percent
        response.wheel_rpm_limit = self.core.wheel_rpm_limit
        return response

    def publish_speed(self):
        message = SpeedLimit()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.percentage, message.speed_limit = True, self.core.speed_percent
        self.speed_pub.publish(message)

    def accept_goal(self, request):
        pose = request.pose.pose
        q = pose.orientation
        finite = all(math.isfinite(x) for x in (pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w))
        valid_pose = (finite and request.pose.header.frame_id == 'map' and abs(pose.position.z) < 1e-6
                      and abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and abs(q.z*q.z + q.w*q.w - 1) < 0.02)
        if (not valid_pose or not self.authorized(request.token, request.epoch) or
                self.core.mode != 'AUTO' or not self.core.armed or self.pending or
                (self.node.p['localization_mode'] == 'amcl' and not self.localization_initialized) or
                self.proxy_reserved or self.core.active_goals or self.core.allowed_goals or
                self.node.readiness(mode='AUTO') or not self.nav_client.server_is_ready() or
                not self.core.stopped(self.node.now_ns(), time.monotonic())):
            return GoalResponse.REJECT
        if not self.node.keepouts.goal_allowed(pose):
            return GoalResponse.REJECT
        self.proxy_reserved = True
        return GoalResponse.ACCEPT

    def cancel_goal(self, handle):
        self.core.cancel_registered_goal()
        self.node.cancel_attempted = None
        self.node.publish_zero()
        return CancelResponse.ACCEPT

    async def wait_for(self, source=None, predicate=None, timeout=6.0, owner=None):
        future = Future()
        self.waiters.append({'future': future, 'source': source, 'predicate': predicate,
                             'deadline': time.monotonic() + timeout, 'owner': owner})
        return await future

    async def execute_goal(self, handle):
        self.proxy_handle = handle
        request = handle.request
        uid = uuid.uuid4()
        result = OperatorNavigate.Result()
        result.internal_goal_id = uid.hex
        result.status = GoalStatus.STATUS_ABORTED
        try:
            if (not self.authorized(request.token, request.epoch) or
                    not self.core.register_goal(uid.hex, self.node.now_ns(), time.monotonic())):
                raise RuntimeError('owner or goal boundary changed before dispatch')
            goal = NavigateToPose.Goal(pose=request.pose)
            goal.behavior_tree = ''
            self.publish_speed()

            def feedback(message):
                if not handle.is_active:
                    return
                output = OperatorNavigate.Feedback()
                for name in ('current_pose', 'navigation_time', 'estimated_time_remaining',
                             'number_of_recoveries', 'distance_remaining'):
                    setattr(output, name, getattr(message.feedback, name))
                output.internal_goal_id = uid.hex
                handle.publish_feedback(output)

            nav = await self.wait_for(self.nav_client.send_goal_async(
                goal, feedback_callback=feedback, goal_uuid=UUID(uuid=list(uid.bytes))), timeout=2.0,
                owner=(request.token, request.epoch))
            if not nav.accepted:
                raise RuntimeError('Nav2 rejected the requested goal')
            if handle.is_cancel_requested:
                nav.cancel_goal_async()
            wrapped = await self.wait_for(nav.get_result_async(), timeout=3600.0,
                                           owner=(request.token, request.epoch))
            result.status = wrapped.status
            result.message = wrapped.result.error_msg
            await self.wait_for(predicate=lambda: not self.core.active_goals and self.core.stopped(
                self.node.now_ns(), time.monotonic()), timeout=6.0)
            result.success = wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.error_code == 0
            if handle.is_cancel_requested:
                result.success, result.status = False, GoalStatus.STATUS_CANCELED
                result.message = 'goal canceled; measured stop confirmed'
                handle.canceled()
            elif result.success:
                result.message = 'goal succeeded; measured stop confirmed'
                handle.succeed()
            else:
                result.message = result.message or 'navigation failed; measured stop confirmed'
                handle.abort()
        except Exception as error:
            result.success, result.message = False, str(error)
            result.status = GoalStatus.STATUS_ABORTED
            self.core._discard_commands()
            self.core.cancel_required = bool(self.core.active_goals or self.core.allowed_goals)
            self.node.publish_zero()
            if handle.is_active:
                handle.abort()
        finally:
            self.core.allowed_goals.discard(uid.hex)
            self.proxy_reserved, self.proxy_handle = False, None
        return result

    def tick(self, ros_ns, wall):
        for waiter in list(self.waiters):
            future, source = waiter['future'], waiter['source']
            if future.done():
                self.waiters.remove(waiter)
                continue
            if source is not None and source.done():
                try:
                    future.set_result(source.result())
                except Exception as error:
                    future.set_exception(error)
            elif waiter['predicate'] is not None and waiter['predicate']():
                future.set_result(True)
            elif waiter['owner'] and not self.authorized(*waiter['owner']):
                future.set_exception(RuntimeError('operator ownership ended; output stopped'))
            elif wall >= waiter['deadline']:
                future.set_exception(TimeoutError('action or measured-stop confirmation timed out'))
            if future.done():
                self.waiters.remove(waiter)
        pending = self.pending
        if pending:
            if wall >= pending['deadline']:
                self.core.fault('operator mode/stop timeout; stop not confirmed')
                self.node.publish_zero()
                self.finish_pending(False, self.core.reason)
            elif pending['kind'] != 'stop' and not self.core.owner_live(ros_ns, wall):
                self.finish_pending(False, 'operator ownership expired during transition')
            else:
                base_state = self.node.states['sim_base'][0]
                if pending['stage'] == 'stop':
                    if (not self.core.active_goals and self.core.stopped(ros_ns, wall) and
                            base_state == State.PRIMARY_STATE_INACTIVE and self.node.base_transition is None):
                        if pending['mode'] == 'DISABLED':
                            if pending['kind'] in ('stop', 'release'):
                                self.core.disable()
                                self.core.forget_owner()
                                self.finish_pending(True, 'goals terminal, base inactive and measured stop confirmed')
                            else:
                                good, message = self.core.finish_transition(ros_ns, wall)
                                self.finish_pending(good, message)
                        elif not self.node.missing and not self.core.arm_error(wall):
                            pending['stage'], self.activating = 'activate', True
                            self.node.request_base_transition(Transition.TRANSITION_ACTIVATE,
                                                              State.PRIMARY_STATE_ACTIVE, wall)
                elif base_state == State.PRIMARY_STATE_ACTIVE and self.node.base_transition is None:
                    good, message = self.core.finish_transition(ros_ns, wall)
                    self.publish_speed()
                    self.finish_pending(good, message)
        self.report(ros_ns, wall)

    def report(self, ros_ns, wall):
        if wall - self.last_report < 0.10:
            return
        self.last_report = wall
        self.confirm_localization(ros_ns)
        manual_missing = self.node.readiness(mode='MANUAL')
        auto_missing = self.node.readiness(mode='AUTO')
        message = OperatorState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = self.node.p['base_frame']
        message.backend, message.session_id, message.operator_enabled = 'simulation', self.session_id, True
        message.localization_mode = self.node.p['localization_mode']
        message.localization_initialized = message.localization_mode == 'slam' or self.localization_initialized
        message.capabilities = ['observe', 'manual', 'navigate', 'speed_limit', 'maps']
        message.mode = self.core.mode
        message.phase = ('TRANSITION' if self.pending else 'BLOCKED' if self.core.state == 'FAULT_LATCHED' or self.node.missing
                         else 'READY' if self.core.mode == 'DISABLED' else 'ACTIVE')
        message.owner_id, message.epoch = self.core.owner_id, self.core.epoch_ns
        message.ready, message.manual_ready, message.auto_ready = not self.node.missing, not manual_missing, not auto_missing
        message.auto_ready = message.auto_ready and message.localization_initialized
        message.missing, message.reason = list(self.node.missing), self.core.reason
        message.stopped = self.core.stopped(ros_ns, wall)
        message.base_active = self.node.states['sim_base'][0] == State.PRIMARY_STATE_ACTIVE
        message.reverse_allowed = self.core.reverse_allowed
        message.reverse_reason = '' if self.core.reverse_allowed else 'rear low-obstacle observation is not certified for this profile'
        message.max_linear_velocity, message.max_angular_velocity = self.core.limits.max_vx, self.core.limits.max_wz
        message.linear_limit, message.angular_limit, message.speed_percent = self.core.linear_limit, self.core.angular_limit, self.core.speed_percent
        message.max_wheel_rpm = self.core.geometry.max_rpm
        message.wheel_rpm_limit = self.core.wheel_rpm_limit
        message.goal_id, message.navigation_state = self.core.goal_id or '', self.core.state
        message.goal_stamp_ns = self.core.goal_stamp
        message.heartbeat_age = wall - self.core.owner_at if self.core.owner_at is not None else -1.0
        message.command_age = wall - self.core.last_source_at if self.core.last_source_at is not None else -1.0
        self.state_pub.publish(message)
