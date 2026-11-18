"""ROS-independent command leases and encoder SE(2) integration."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Geometry:
    left_radius: float = .068
    right_radius: float = .068
    separation: float = .480
    counts_per_rev: int = 2652
    max_rpm: float = 95.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in
               (self.left_radius, self.right_radius, self.separation, self.max_rpm)):
            raise ValueError('Geometry and RPM limit must be positive finite values')
        if type(self.counts_per_rev) is not int or self.counts_per_rev <= 0 or self.max_rpm > 100:
            raise ValueError('Invalid encoder resolution or physical limit above 100 RPM')

    def rpm(self, vx, wz):
        if not all(math.isfinite(v) for v in (vx, wz)):
            raise ValueError('Nonfinite velocity')
        left = (vx-wz*self.separation/2)/self.left_radius*30/math.pi
        right = (vx+wz*self.separation/2)/self.right_radius*30/math.pi
        scale = min(1., self.max_rpm/max(abs(left), abs(right), 1e-30))
        return left*scale, right*scale


class EncoderOdometry:
    def __init__(self, geometry):
        self.g = geometry
        self.last = None
        self.ros_stamp_ns = None
        self.receipt_ns = None
        self.x = self.y = self.yaw = 0.

    def update(self, tick, left, right, ros_ns):
        if any(type(x) is not int for x in (tick, left, right)) or not 0 <= tick <= 0xffffffff:
            raise ValueError('Malformed encoder measurement')
        if type(ros_ns) is not int or ros_ns <= 0:
            raise ValueError('Invalid ROS receipt time')
        if self.receipt_ns is not None and ros_ns < self.receipt_ns:
            raise ValueError('ROS receipt clock reversed')
        if self.last is None:
            self.last = (tick, left, right)
            self.ros_stamp_ns = ros_ns
            self.receipt_ns = ros_ns
            vx = wz = 0.
            dt = 0.
            dl = dr = 0.
        else:
            old_tick, old_l, old_r = self.last
            elapsed = (tick-old_tick) & 0xffffffff
            if elapsed == 0:
                if (left, right) != (old_l, old_r):
                    raise ValueError('Encoder counts changed at duplicate MCU tick')
                return None
            if elapsed > 400:
                raise ValueError('MCU clock reset/reversal or feedback gap')
            predicted_stamp = self.ros_stamp_ns + elapsed * 1000000
            error_ns = ros_ns - predicted_stamp
            # Detect discontinuities before updating pose or clock state. Slow
            # oscillator drift is expected; a sudden 400 ms offset is not.
            if abs(error_ns) > 400000000:
                raise ValueError('ROS/MCU clock offset jumped; reconfigure required')
            # Track receipt offset with a 2 s EMA, limited to 1% of MCU dt.
            # This changes message time only, never encoder integration dt.
            correction = max(-elapsed * 10000, min(elapsed * 10000,
                round(error_ns * min(1., elapsed / 2000.))))
            dt = elapsed/1000
            dl = (left-old_l)*2*math.pi/self.g.counts_per_rev
            dr = (right-old_r)*2*math.pi/self.g.counts_per_rev
            # Counts are already unwrapped, signed physical wheel counts.
            # Reject reset/jumps instead of integrating a discontinuity.
            limit = 100*math.pi/30*dt + 4*math.pi/self.g.counts_per_rev
            if max(abs(dl),abs(dr)) > limit:
                raise ValueError('Encoder jump exceeds physical 100 RPM bound')
            sl, sr = dl*self.g.left_radius, dr*self.g.right_radius
            ds = (sl+sr)/2
            angle = (sr-sl)/self.g.separation
            sinc = 1. if abs(angle) < 1e-9 else math.sin(angle/2)/(angle/2)
            self.x += ds*sinc*math.cos(self.yaw+angle/2)
            self.y += ds*sinc*math.sin(self.yaw+angle/2)
            self.yaw = math.atan2(math.sin(self.yaw+angle),math.cos(self.yaw+angle))
            vx, wz = ds/dt, angle/dt
            self.ros_stamp_ns = predicted_stamp + correction
            self.receipt_ns = ros_ns
            self.last = (tick,left,right)
        return {'stamp_ns': self.ros_stamp_ns, 'x':self.x, 'y':self.y, 'yaw':self.yaw,
                'vx':vx,'wz':wz,'positions':[left*2*math.pi/self.g.counts_per_rev,
                                          right*2*math.pi/self.g.counts_per_rev],
                'velocities':[dl/dt if dt else 0.,dr/dt if dt else 0.]}


class CommandGate:
    TIMEOUT = .3

    def __init__(self):
        self.source = None
        self.target = None
        self.stopped_stamp = -1
        self.activation_ns = 0
        self.active = False

    def activate(self, ros_ns):
        self.clear()
        self.activation_ns = ros_ns
        self.active = True

    def clear(self):
        self.source = self.target = None
        self.stopped_stamp = -1
        self.active = False

    def requested(self, stamp, ros_ns, now):
        if not self.active or stamp < self.activation_ns or stamp <= 0:
            return False
        age = (ros_ns-stamp)*1e-9
        if not 0 <= age < self.TIMEOUT:
            return False
        if self.source and stamp < self.source[0]:
            return False
        if not self.source or stamp > self.source[0]:
            issued = now-age
            # Independent ROS/steady sampling can make a newer stamp map to
            # an older issue time. Drop that sample instead of faulting the
            # hardware driver or extending the original lease by clamping it.
            if self.source and issued < self.source[1]:
                return False
            self.source = (stamp, issued)
        return True

    def final(self, stamp, vx, wz, ros_ns, now):
        if not self.active:
            return False
        if vx == 0 and wz == 0:
            # Smoother timeouts and downstream stops may carry a new header.
            # Stop immediately without granting a lease, and retire every
            # intent already seen so the next tick cannot revive its target.
            self.target = None
            self.stopped_stamp = max(
                self.stopped_stamp, self.source[0] if self.source else -1,
                stamp if 0 < stamp <= ros_ns else -1)
            if (self.source and 0 <= (ros_ns-self.source[0])*1e-9 < self.TIMEOUT
                    and 0 <= now-self.source[1] < self.TIMEOUT):
                # Preserve an already-live braking lease for the hardware
                # adapter; the stop cannot create or extend that lease.
                self.target = (0., 0., self.source[1], now)
            return True
        if not self.source or stamp != self.source[0]:
            return False
        age = (ros_ns-stamp)*1e-9
        if not 0 <= age < self.TIMEOUT or not 0 <= now-self.source[1] < self.TIMEOUT:
            return False
        if (vx or wz) and stamp <= self.stopped_stamp:
            return False
        self.target = (vx,wz,self.source[1],now)
        return True

    def current(self, now):
        if not self.active or not self.target:
            return None
        vx,wz,issued,received = self.target
        if not 0 <= now-issued < self.TIMEOUT or not 0 <= now-received < self.TIMEOUT:
            self.target = None
            return None
        return vx,wz,issued
