"""Verify a sustained stop from distinct, fresh odometry measurements."""
import math


class MeasuredStop:
    def __init__(self, after_ns, duration=.3):
        self.after_ns = after_ns
        self.duration_ns = round(duration * 1e9)
        self.first_ns = None
        self.last_ns = after_ns
        self.received_at = None
        self.count = 0
        self.motion_observed = False

    def observe(self, msg, wall):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp <= self.after_ns or stamp == self.last_ns:
            return
        if stamp < self.last_ns:
            self.first_ns = None
            self.count = 0
            return
        velocity = msg.twist.twist
        stationary = (msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_footprint'
                      and math.hypot(velocity.linear.x, velocity.linear.y) <= .01
                      and abs(velocity.angular.z) <= .02)
        if not stationary or stamp - self.last_ns > 250_000_000 or (
                self.received_at is not None and wall - self.received_at > .5):
            self.first_ns = None
            self.count = 0
        self.last_ns, self.received_at = stamp, wall
        if stationary:
            if self.first_ns is None:
                self.first_ns = stamp
            self.count += 1
        else:
            self.motion_observed = True

    def confirmed(self, now_ns, wall):
        return (self.first_ns is not None and self.count >= 2
                and self.last_ns - self.first_ns >= self.duration_ns
                and 0 <= now_ns - self.last_ns <= 250_000_000
                and 0 <= wall - self.received_at <= .5)
