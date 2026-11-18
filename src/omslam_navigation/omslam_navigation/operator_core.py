"""Single operator ownership, measured-stop boundaries and manual source leases.

No ROS transport belongs here. NavigationGate still owns the original stamped
command history and the final collision-monitor gate for both sources.
"""

import math
import secrets

from .guard_core import NavigationGate
from omslam_base.core import Geometry
from omslam_sim.control import SIM_MAX_WHEEL_RPM


class OperatorGate(NavigationGate):
    OWNER_TIMEOUT = 0.30
    STOP_HOLD = 0.30
    STOP_FRESH = 0.20

    def __init__(self, limits=None, reverse_allowed=False):
        super().__init__(limits)
        self.mode = 'DISABLED'
        self.pending_mode = None
        self.owner_id = self.token = ''
        self.owner_at = None
        self.owner_stamp = self.heartbeat_sequence = self.manual_sequence = 0
        self.manual_at = None
        self.release_required = True
        self.last_direction = 0
        self.allowed_goals = set()
        self.canceled_goals = set()
        self.reverse_allowed = reverse_allowed
        self.speed_percent = 50.0
        self.geometry = Geometry(max_rpm=SIM_MAX_WHEEL_RPM)
        self.wheel_rpm_limit = self.geometry.max_rpm * self.speed_percent / 100.
        self.linear_limit = self.limits.max_vx * self.speed_percent / 100.
        self.angular_limit = self.limits.max_wz * self.speed_percent / 100.
        self.stop_samples = {}
        self.stopped_since = None

    def clock(self, stamp, wall):
        rewound = super().clock(stamp, wall)
        if rewound:
            self.stop_samples.clear()
            self.stopped_since = None
        return rewound

    def desired_mode(self):
        return self.pending_mode or (self.mode if self.mode != 'DISABLED' else 'MANUAL')

    def owner_live(self, ros_ns, wall):
        return (bool(self.token) and self.owner_at is not None and
                0 <= wall - self.owner_at < self.OWNER_TIMEOUT and
                0 <= (ros_ns - self.owner_stamp) * 1e-9 < self.OWNER_TIMEOUT)

    def authorized(self, token, epoch, ros_ns, wall):
        return (bool(token) and secrets.compare_digest(token, self.token) and
                epoch == self.epoch_ns and self.owner_live(ros_ns, wall))

    def control_result(self, success, message):
        # Failed acquire/validate requests belong to an untrusted observer.
        # Never return the current owner's capability in a failure response.
        return (bool(success), message, self.token if success else '',
                self.epoch_ns if success else 0, self.mode)

    def acquire(self, owner_id, ros_ns, wall):
        if self.token or self.armed or self.pending_mode or self.active_goals:
            return False, 'another owner or a previous stop is still active'
        if not owner_id.strip() or len(owner_id) > 128 or ros_ns <= 0:
            return False, 'a nonempty owner ID and progressing ROS clock are required'
        self.owner_id = owner_id
        self.rotate_token(ros_ns, wall)
        self.mode = 'DISABLED'
        return True, 'ownership granted; choose a mode explicitly'

    def rotate_token(self, ros_ns, wall):
        self.token = secrets.token_hex(24)
        self.epoch_ns = ros_ns
        self.owner_at, self.owner_stamp = wall, ros_ns
        self.heartbeat_sequence = self.manual_sequence = 0
        self.manual_at = None
        self.release_required = True
        self.last_direction = 0
        self.allowed_goals.clear()
        self.canceled_goals.clear()
        self._discard_commands()

    def forget_owner(self):
        self.owner_id = self.token = ''
        self.owner_at = None
        self.owner_stamp = 0

    def heartbeat(self, token, epoch, sequence, stamp, ros_ns, wall):
        if (not self.authorized(token, epoch, ros_ns, wall) or
                sequence <= self.heartbeat_sequence or stamp <= self.owner_stamp or
                not 0 <= (ros_ns - stamp) * 1e-9 < self.OWNER_TIMEOUT):
            return False
        self.heartbeat_sequence, self.owner_stamp = sequence, stamp
        self.owner_at = wall - (ros_ns - stamp) * 1e-9
        return True

    def begin_transition(self, mode):
        if mode not in ('DISABLED', 'MANUAL', 'AUTO'):
            return False, 'unsupported control mode'
        super().disable()
        self.mode, self.pending_mode = 'DISABLED', mode
        self.allowed_goals.clear()
        self.release_required = True
        return True, 'stopping before mode change'

    def finish_transition(self, ros_ns, wall):
        mode = self.pending_mode
        if mode is None or self.active_goals or not self.stopped(ros_ns, wall):
            return False, 'waiting for terminal goals and fresh measured stop'
        if mode != 'DISABLED':
            success, message = super().arm(ros_ns, wall)
            if not success:
                return False, message
        self.rotate_token(ros_ns, wall)
        self.mode, self.pending_mode = mode, None
        if mode == 'MANUAL':
            self.state, self.reason = 'RUNNING', 'manual ready; release input before driving'
        elif mode == 'AUTO':
            self.reason = 'auto ready; waiting for an authorized goal'
        else:
            self.state, self.reason = 'DISARMED', 'disabled; measured stop confirmed'
        return True, self.reason

    def disable(self):
        super().disable()
        self.mode, self.pending_mode = 'DISABLED', None
        self.release_required = True
        self.allowed_goals.clear()

    def fault(self, reason, restart=False):
        super().fault(reason, restart)
        self.mode, self.pending_mode = 'DISABLED', None
        self.release_required = True
        self.allowed_goals.clear()
        self.forget_owner()

    def sample_stop(self, key, stamp, vx, wz, valid, wall):
        previous = self.stop_samples.get(key)
        if previous and stamp <= previous[0]:
            if not valid:
                self.stopped_since = None
                self.stop_samples[key] = (previous[0], previous[1], False)
            return
        stationary = valid and math.isfinite(vx) and math.isfinite(wz) and abs(vx) <= 0.01 and abs(wz) <= 0.02
        self.stop_samples[key] = (stamp, wall, stationary)
        if not stationary:
            self.stopped_since = None

    def stopped(self, ros_ns, wall):
        valid = self.clock_fresh(wall)
        for key in ('wheel', 'odom'):
            sample = self.stop_samples.get(key)
            valid = valid and sample is not None and sample[2] and (
                0 <= (ros_ns - sample[0]) * 1e-9 < self.STOP_FRESH and
                0 <= wall - sample[1] < self.STOP_FRESH)
        if not valid:
            self.stopped_since = None
            return False
        if self.stopped_since is None:
            self.stopped_since = wall
        return wall - self.stopped_since >= self.STOP_HOLD

    def motion_allowed(self):
        if self.pending_mode is not None or not self.token:
            return False
        if self.mode == 'MANUAL':
            return self.state == 'RUNNING' and not self.release_required
        return (self.mode == 'AUTO' and self.goal_id in self.allowed_goals and
                self.goal_id not in self.canceled_goals and super().motion_allowed())

    def source_owner(self):
        if self.mode == 'MANUAL':
            return 'manual:' + str(self.epoch_ns) + ':' + self.token
        return super().source_owner()

    def _live(self, source, ros_ns, wall):
        return self.owner_live(ros_ns, wall) and super()._live(source, ros_ns, wall)

    def _valid_twist(self, frame, values):
        if frame != self.limits.frame or len(values) != 6 or not all(math.isfinite(x) for x in values):
            return False
        vx, vy, vz, wx, wy, wz = values
        minimum = -self.limits.max_vx if self.mode == 'MANUAL' and self.reverse_allowed else 0.0
        return (vy == vz == wx == wy == 0 and minimum - 1e-9 <= vx <= self.limits.max_vx + 1e-6
                and abs(wz) <= self.limits.max_wz + 1e-6)

    def limit_values(self, values):
        vx, vy, vz, wx, wy, wz = values
        if self.mode == 'MANUAL':
            linear, angular = self.linear_limit, self.angular_limit
        else:
            linear = self.limits.max_vx * self.speed_percent / 100.0
            angular = self.limits.max_wz * self.speed_percent / 100.0
        minimum = -linear if self.mode == 'MANUAL' and self.reverse_allowed else 0.0
        vx, wz = max(minimum, min(linear, vx)), max(-angular, min(angular, wz))
        # Coupled scaling caps both wheels even while translating and turning.
        # Use unsaturated kinematics: Geometry.rpm() itself clips at max_rpm.
        left = (vx - wz * self.geometry.separation / 2) / self.geometry.left_radius * 30 / math.pi
        right = (vx + wz * self.geometry.separation / 2) / self.geometry.right_radius * 30 / math.pi
        scale = min(1., self.wheel_rpm_limit / max(abs(left), abs(right), 1e-30))
        return (vx * scale, vy, vz, wx, wy, wz * scale)

    def brake_manual(self, reason, require_release=True):
        self._discard_commands()
        self.release_required = require_release
        self.reason = reason

    def manual(self, token, epoch, sequence, deadman, stamp, frame, values, ros_ns, wall):
        if (self.mode != 'MANUAL' or self.pending_mode is not None or
                not self.authorized(token, epoch, ros_ns, wall) or sequence <= self.manual_sequence):
            return False
        self.manual_sequence = sequence
        if not deadman:
            self.brake_manual('manual input released', require_release=False)
            return False
        if self.release_required or not self._valid_twist(frame, values):
            self.brake_manual('release input; invalid or unarmed manual command')
            return False
        direction = 1 if values[0] > 0 else -1 if values[0] < 0 else 0
        if direction and self.last_direction * direction < 0 and not self.stopped(ros_ns, wall):
            self.brake_manual('direction change requires measured stop and released input')
            return False
        if not super().requested(stamp, frame, values, ros_ns, wall):
            self.brake_manual('manual command is stale or not ready')
            return False
        self.manual_at = wall - (ros_ns - stamp) * 1e-9
        if direction:
            self.last_direction = direction
        return True

    def requested(self, stamp, frame, values, ros_ns, wall):
        # This entry point is exclusively the Nav2 source; manual uses manual().
        if self.mode != 'AUTO' or not self.owner_live(ros_ns, wall):
            self.rejected_sources += 1
            return False
        return super().requested(stamp, frame, values, ros_ns, wall)

    def register_goal(self, uid, ros_ns, wall):
        if self.mode != 'AUTO' or not self.armed or self.pending_mode or not self.owner_live(ros_ns, wall):
            return False
        if self.active_goals or self.allowed_goals or self.goal_id:
            return False
        self.allowed_goals.add(uid)
        return True

    def cancel_registered_goal(self):
        self.canceled_goals.update(self.allowed_goals)
        if self.goal_id:
            self.canceled_goals.add(self.goal_id)
        self._discard_commands()
        self.goal_status = 3
        self.cancel_required = bool(self.active_goals or self.allowed_goals)
        self.reason = 'operator goal cancellation requested; output stopped'

    def goals(self, statuses, ros_ns):
        unauthorized = {uid for uid, _, status in statuses if status in (1, 2, 3)
                        and uid not in self.allowed_goals}
        if unauthorized and self.armed:
            self.active_goals = {uid: (stamp, status) for uid, stamp, status in statuses if status in (1, 2, 3)}
            self.fault('unregistered navigation goal; operator ownership is required')
            return
        # A delayed EXECUTING status after a local cancel cannot reopen motion.
        effective = [(uid, stamp, 3 if uid in self.canceled_goals and status in (1, 2, 3) else status)
                     for uid, stamp, status in statuses]
        super().goals(effective, ros_ns)
        for uid, _, status in statuses:
            if status in (4, 5, 6):
                self.allowed_goals.discard(uid)
                self.canceled_goals.discard(uid)

    def map_transform(self, x, y, yaw):
        if self.mode != 'AUTO':
            self.map_pose = (x, y, yaw)
            return True
        return super().map_transform(x, y, yaw)

    def tick(self, ros_ns, wall):
        self.stopped(ros_ns, wall)
        if self.token and not self.owner_live(ros_ns, wall):
            self.fault('operator heartbeat expired; reconnect and acquire control again')
        if self.mode == 'MANUAL' and self.manual_at is not None and wall - self.manual_at >= self.limits.command_timeout:
            self.manual_at = None
            self.brake_manual('manual input expired; release before driving again')
        return super().tick(ros_ns, wall)
