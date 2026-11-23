"""Steady watchdog around simulator time and the shared command lease."""
import math
from omslam_base.core import CLOCK_SKEW_S, MIN_WHEEL_RPM, CommandGate, Geometry


SIM_MIN_WHEEL_RPM = MIN_WHEEL_RPM
SIM_MAX_WHEEL_RPM = 70.
SIM_MAX_LINEAR_VELOCITY = Geometry().left_radius * math.pi / 30 * SIM_MAX_WHEEL_RPM


class SimCommandGuard:
    def __init__(self, geometry=None, clock_timeout=.3):
        self.geometry = geometry or Geometry(max_rpm=SIM_MAX_WHEEL_RPM, min_rpm=SIM_MIN_WHEEL_RPM)
        self.gate = CommandGate()
        self.clock_timeout = clock_timeout
        self.sim_ns = None
        self.advanced_at = None
        self.suspended = True
        self.active = False
        self.reason = 'unconfigured'
        self.target_stamp = None

    def clock(self, stamp_ns, now):
        if self.sim_ns is None or stamp_ns != self.sim_ns:
            reset = self.sim_ns is not None and stamp_ns < self.sim_ns
            self.sim_ns = stamp_ns
            self.advanced_at = now
            if reset:
                self.deactivate()
                self.suspended = False
                self.reason = 'clock rewound; fresh wheels and reactivation required'
            elif self.suspended:
                self.gate.clear()
                if self.active:
                    # A message created at the frozen clock edge is old intent.
                    self.gate.activate(stamp_ns + 1)
                self.suspended = False
                self.reason = 'new clock epoch; waiting for new intent'

    def clock_fresh(self, now):
        return self.advanced_at is not None and 0 <= now-self.advanced_at < self.clock_timeout

    def activate(self, now):
        if not self.clock_fresh(now) or self.sim_ns <= 0:
            raise ValueError('Activation requires an advancing simulator clock')
        self.active = True
        self.suspended = False
        self.gate.activate(self.sim_ns + 1)
        self.reason = 'active; waiting for new intent'

    def deactivate(self):
        self.active = False
        self.gate.clear()
        self.reason = 'inactive'

    def requested(self, stamp, now):
        return self.gate.requested(stamp, self.sim_ns, now)

    def final(self, stamp, vx, wz, now):
        accepted = self.gate.final(stamp, vx, wz, self.sim_ns, now)
        if accepted: self.target_stamp = stamp
        return accepted

    def sample(self, now):
        return self.geometry.twist(*self.sample_rpm(now))

    def sample_rpm(self, now):
        if not self.clock_fresh(now):
            if not self.suspended:
                self.gate.clear()
                self.suspended = True
                self.reason = 'clock paused/stalled; intent discarded'
            return 0., 0.
        if not self.active or self.suspended:
            return 0., 0.
        target = self.gate.current(now)
        if target is None:
            self.reason = 'no live original intent'
            return 0., 0.
        vx, wz, _ = target
        # Reject simulation-time expiry too: a speed-up must not lengthen lease.
        if self.target_stamp is None or not -CLOCK_SKEW_S <= (self.sim_ns-self.target_stamp)*1e-9 < self.gate.TIMEOUT:
            self.gate.target = None
            return 0., 0.
        try:
            # The applied output can be zero while the original lease stays live.
            left, right = self.geometry.rpm(vx, wz)
        except ValueError:
            self.gate.withdraw_target()
            self.reason = 'invalid wheel velocity; intent discarded'
            return 0., 0.
        self.reason = 'live intent'
        return left, right
