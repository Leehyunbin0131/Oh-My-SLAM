"""Navigation permission and original-command leases, independent of ROS transport.

All ROS timestamps are integer nanoseconds. Wall times must come from a
monotonic clock. A successful downstream callback can never renew a source
lease, and a stop always takes precedence over a queued nonzero command.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Limits:
    command_timeout: float = 0.30
    clock_timeout: float = 0.30
    recovery_time: float = 1.0
    map_translation: float = 0.10
    map_rotation: float = 0.10
    max_vx: float = 0.20
    max_wz: float = 0.60
    frame: str = 'base_footprint'


@dataclass(frozen=True)
class Source:
    stamp: int
    issued_wall: float
    goal_id: str
    motion_requested: bool


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class NavigationGate:
    """One enabled session, one action goal, and bounded original command history."""

    GOAL_CLOCK_GRACE = 0.10

    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.state = 'DISARMED'
        self.reason = 'waiting for readiness'
        self.healthy = False
        self.healthy_since = None
        self.epoch_ns = 0
        self.clock_ns = None
        self.clock_at = None
        self.goal_id = None
        self.goal_stamp = 0
        self.goal_status = 0
        self.goal_clock_deadline = None
        self.active_goals = {}
        self.retired_goals = set()
        self.sources = {}
        self.latest_source = 0
        self.last_source_at = None
        self.first_source_at = None
        self.final_stamp = 0
        self.final_at = None
        self.stopped_stamp = 0
        self.output = (0.0, 0.0)
        self.cancel_required = False
        self.restart_required = False
        self.map_pose = None
        self.fault_count = 0
        self.accepted_sources = 0
        self.rejected_sources = 0
        self.accepted_finals = 0
        self.rejected_finals = 0

    @property
    def armed(self):
        return self.state in ('ARMED_IDLE', 'RUNNING')

    def motion_allowed(self):
        return self.state == 'RUNNING' and self.goal_status == 2

    def source_owner(self):
        return self.goal_id

    def _discard_commands(self):
        self.sources.clear()
        self.latest_source = 0
        self.last_source_at = None
        self.first_source_at = None
        self.final_at = None
        self.final_stamp = 0
        self.stopped_stamp = 0
        self.output = (0.0, 0.0)

    def clock(self, stamp, wall):
        rewound = self.clock_ns is not None and stamp < self.clock_ns
        if self.clock_ns is None or stamp != self.clock_ns:
            self.clock_ns, self.clock_at = stamp, wall
        if rewound:
            self.fault('ROS clock reversed; new observations and enable required')
            self.healthy = False
            self.healthy_since = None
            self.map_pose = None
        return rewound

    def clock_fresh(self, wall):
        return (self.clock_ns is not None and self.clock_ns > 0 and
                self.clock_at is not None and
                0 <= wall - self.clock_at < self.limits.clock_timeout)

    def health(self, good, reason, wall):
        if good:
            if not self.healthy:
                self.healthy_since = wall
            self.healthy = True
            if self.state in ('DISARMED', 'READY'):
                self.state, self.reason = 'READY', 'ready; explicit enable required'
        else:
            self.healthy = False
            self.healthy_since = None
            if self.armed:
                self.fault(reason)
            elif self.state != 'FAULT_LATCHED':
                self.state, self.reason = 'DISARMED', reason

    def arm_error(self, wall):
        if self.restart_required:
            return 'map geometry changed; restart this navigation session'
        if self.active_goals:
            return 'previous action goal has not reached a terminal state'
        if not self.healthy or not self.clock_fresh(wall):
            return 'fresh sensors, transforms and active servers are required'
        if self.healthy_since is None or wall - self.healthy_since < self.limits.recovery_time:
            return 'healthy observations must remain stable for the recovery interval'
        return ''

    def arm(self, ros_ns, wall):
        error = self.arm_error(wall)
        if error:
            return False, error
        self._discard_commands()
        self.epoch_ns = ros_ns
        self.goal_id, self.goal_stamp, self.goal_status = None, 0, 0
        self.cancel_required = False
        self.state, self.reason = 'ARMED_IDLE', 'enabled; waiting for a new goal'
        return True, self.reason

    def disable(self):
        if self.goal_id:
            self.retired_goals.add(self.goal_id)
        self.cancel_required = bool(self.active_goals)
        self.goal_id, self.goal_stamp, self.goal_status = None, 0, 0
        self.goal_clock_deadline = None
        self._discard_commands()
        if self.state != 'FAULT_LATCHED':
            self.state, self.reason = 'DISARMED', 'disabled by request'

    def fault(self, reason, restart=False):
        if self.state != 'FAULT_LATCHED':
            self.fault_count += 1
            self.reason = reason
        self.state = 'FAULT_LATCHED'
        self.restart_required = self.restart_required or restart
        if self.goal_id:
            self.retired_goals.add(self.goal_id)
        self.retired_goals.update(self.active_goals)
        self.cancel_required = bool(self.active_goals)
        self.goal_id, self.goal_stamp, self.goal_status = None, 0, 0
        self.goal_clock_deadline = None
        self._discard_commands()

    def goals(self, statuses, ros_ns, wall=None):
        """Receive (UUID hex, accepted ROS timestamp, GoalStatus integer)."""
        previous = self.goal_id
        live = {uid: (stamp, status) for uid, stamp, status in statuses
                if status in (1, 2, 3)}
        terminal = {uid for uid, _, status in statuses if status in (4, 5, 6)}
        self.retired_goals.update(terminal)
        self.active_goals = live
        if self.armed and previous and any(uid != previous for uid in live):
            # A depth-one status stream can omit the overlap during preemption.
            # Require a separately observed terminal/idle boundary before the
            # next UUID, even if the old terminal appears in this snapshot.
            self.fault('replacement goal arrived before the previous terminal boundary')
            return
        if not live:
            self.cancel_required = False
        if previous and previous in terminal:
            self.goal_id, self.goal_stamp, self.goal_status = None, 0, 0
            self.goal_clock_deadline = None
            self._discard_commands()
            if self.armed:
                self.state, self.reason = 'ARMED_IDLE', 'goal terminal; waiting for a new goal'
        if not self.armed:
            self.retired_goals.update(live)
            self.cancel_required = bool(live)
            return
        if len(live) > 1 or (self.goal_id and live and self.goal_id not in live):
            self.fault('concurrent or replacement navigation goals are not supported')
            return
        if self.goal_id and not live:
            self.fault('executing goal disappeared without a terminal status')
            return
        if not live:
            return
        uid, (stamp, status) = next(iter(live.items()))
        wall = self.clock_at if wall is None else wall
        if self.goal_id is None:
            if (uid in self.retired_goals or stamp <= self.epoch_ns or
                    stamp - ros_ns > self.GOAL_CLOCK_GRACE * 1e9):
                self.fault('goal belongs to an earlier or invalid activation epoch')
                return
            if status == 3:
                self.retired_goals.add(uid)
                self.cancel_required = True
                return
            self.goal_id, self.goal_stamp, self.goal_status = uid, stamp, 0
            self._discard_commands()
            if stamp > ros_ns:
                self.goal_clock_deadline = wall + self.GOAL_CLOCK_GRACE
        if self.goal_clock_deadline is not None and status != 3:
            if wall >= self.goal_clock_deadline:
                self.fault('goal acceptance timestamp did not catch up with the clock')
                return
            if ros_ns < self.goal_stamp:
                self.goal_status = 1
                self.reason = 'waiting for the goal acceptance clock tick'
                return
        self.goal_clock_deadline = None
        if self.goal_status == 3 and status in (1, 2):
            # A late EXECUTING snapshot cannot reopen a canceled goal.
            return
        self.goal_status = status
        if status == 3:
            self._discard_commands()
            self.reason = 'goal canceling; output stopped'
        elif status == 2:
            self.state, self.reason = 'RUNNING', 'executing fresh navigation goal'

    def map_transform(self, x, y, yaw):
        pose = (x, y, yaw)
        if not all(math.isfinite(v) for v in pose):
            if self.armed:
                self.fault('nonfinite map transform', restart=True)
            return False
        old = self.map_pose
        self.map_pose = pose
        if old is not None and self.armed:
            distance = math.hypot(x - old[0], y - old[1])
            rotation = abs(angle_difference(yaw, old[2]))
            if distance >= self.limits.map_translation or rotation >= self.limits.map_rotation:
                self.fault('map correction exceeded the active-session bound', restart=True)
                return False
        return True

    def _valid_twist(self, frame, values):
        if frame != self.limits.frame or len(values) != 6:
            return False
        if not all(math.isfinite(v) for v in values):
            return False
        vx, vy, vz, wx, wy, wz = values
        return (vy == vz == wx == wy == 0 and
                -1e-9 <= vx <= self.limits.max_vx + 1e-6 and
                abs(wz) <= self.limits.max_wz + 1e-6)

    def _live(self, source, ros_ns, wall):
        return (source.goal_id == self.source_owner() and
                0 <= (ros_ns - source.stamp) * 1e-9 < self.limits.command_timeout and
                0 <= wall - source.issued_wall < self.limits.command_timeout)

    def waiting_for_final(self, ros_ns, wall):
        """A live new intention is still traversing the downstream pipeline."""
        source = self.sources.get(self.latest_source)
        return (self.motion_allowed() and self.healthy and self.clock_fresh(wall)
                and source is not None and source.motion_requested
                and source.stamp > max(self.stopped_stamp, self.final_stamp)
                and self._live(source, ros_ns, wall))

    def requested(self, stamp, frame, values, ros_ns, wall):
        valid = self._valid_twist(frame, values)
        if not valid and self.armed:
            self.fault('invalid navigation command frame, axis or velocity')
        if (not valid or not self.motion_allowed() or
                not self.healthy or not self.clock_fresh(wall) or
                stamp < self.goal_stamp or stamp <= self.epoch_ns or
                stamp < self.latest_source or
                not 0 <= (ros_ns - stamp) * 1e-9 < self.limits.command_timeout):
            self.rejected_sources += 1
            return False
        source = self.sources.get(stamp)
        if source is None:
            source = Source(stamp, wall - (ros_ns - stamp) * 1e-9, self.source_owner(),
                            bool(values[0] or values[5]))
            self.sources[stamp] = source
            self.latest_source = stamp
            self.last_source_at = wall
            if self.first_source_at is None:
                self.first_source_at = wall
                # A timeout zero may arrive after the stopped burst was
                # discarded. It is not downstream evidence for this new
                # burst; allow the first new command its bounded response time.
                self.final_at = None
        if not self._live(source, ros_ns, wall):
            self.rejected_sources += 1
            return False
        self.sources = {key: item for key, item in self.sources.items()
                        if self._live(item, ros_ns, wall)}
        # At 20 Hz there are normally six live entries; bound malformed bursts.
        if len(self.sources) > 32:
            self.sources = dict(sorted(self.sources.items())[-32:])
        self.accepted_sources += 1
        return True

    def final(self, stamp, frame, values, ros_ns, wall):
        if len(values) == 6 and all(value == 0 for value in values):
            # Nav2 timeout stops can have a new stamp and an empty frame.
            # A pure stop is frame-independent and grants no driving lease.
            self.output = (0.0, 0.0)
            self.stopped_stamp = max(self.stopped_stamp, self.latest_source,
                                     stamp if 0 < stamp <= ros_ns else 0)
            self.final_at = wall
            self.accepted_finals += 1
            return True
        source = self.sources.get(stamp)
        if (not self.motion_allowed() or not self.healthy or
                not self.clock_fresh(wall) or source is None or
                stamp <= self.stopped_stamp or stamp < self.final_stamp or
                not self._live(source, ros_ns, wall)):
            self.rejected_finals += 1
            return False
        # The installed smoother may still emit deceleration samples after
        # its timeout replaced the source with a fresh, empty-frame header.
        # Such unmatched/expired output above grants no permission and is
        # dropped. Validate frame/axes only for a currently authorized source.
        if not self._valid_twist(frame, values):
            self.rejected_finals += 1
            self.fault('invalid collision monitor command')
            return False
        vx, _, _, _, _, wz = values
        self.output = (vx, wz)
        self.final_stamp, self.final_at = stamp, wall
        self.accepted_finals += 1
        return True

    def tick(self, ros_ns, wall):
        if self.armed and not self.clock_fresh(wall):
            self.fault('simulation clock paused or stalled')
        if self.goal_clock_deadline is not None:
            uid = self.goal_id
            stamp, status = self.active_goals[uid]
            self.goals([(uid, stamp, status)], ros_ns, wall)
        if self.motion_allowed() and self.latest_source:
            latest = self.sources.get(self.latest_source)
            if latest is None or not self._live(latest, ros_ns, wall):
                if latest is not None and latest.motion_requested and self.output != (0.0, 0.0):
                    self.fault('original navigation command expired')
                else:
                    # CancelControl followed by the bounded BT Wait is a stopped
                    # interval of the same action, not a new driving intention.
                    self._discard_commands()
            elif self.final_at is None and wall - self.first_source_at >= self.limits.command_timeout:
                self.fault('collision monitor output missing')
            elif (self.final_at is not None and wall - self.final_at >= self.limits.command_timeout and
                  (latest.motion_requested or self.output != (0.0, 0.0))):
                self.fault('collision monitor output expired')
        return self.output
