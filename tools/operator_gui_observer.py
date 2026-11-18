#!/usr/bin/env python3
"""Record GUI integration evidence outside the Qt interpreter.

Only a small, latest-value snapshot crosses the pipe for scenario progress.
The pipe is nonblocking: a frozen GUI cannot prevent recording the robot stop.
"""
import argparse
import json
import math
import os
import signal
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from omslam_operator_msgs.msg import ManualCommand, OwnerHeartbeat, OperatorState

p = argparse.ArgumentParser()
p.add_argument('--output', required=True)
p.add_argument('--pipe-fd', type=int, required=True)
a = p.parse_args()
os.set_blocking(a.pipe_fd, False)
raw = Path(a.output).open('w')
latest = {}


def stamp(value):
    return value.sec * 1_000_000_000 + value.nanosec


def record(row):
    raw.write(json.dumps(row) + '\n')


class Observer(Node):
    def __init__(self):
        super().__init__('operator_verification_observer')
        for topic in ('/odom', '/wheel/odometry', '/sim/ground_truth'):
            self.create_subscription(Odometry, topic, lambda m, t=topic: self.motion(t, m), qos_profile_sensor_data)
        for topic in ('/cmd_vel/requested', '/cmd_vel'):
            self.create_subscription(TwistStamped, topic, lambda m, t=topic: self.motion(t, m), 100)
        self.create_subscription(ManualCommand, '/operator/manual', lambda m: self.intent('/operator/manual', m), 50)
        self.create_subscription(OwnerHeartbeat, '/operator/heartbeat', lambda m: self.intent('/operator/heartbeat', m), 50)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OperatorState, '/operator/state', self.operator, state_qos)
        self.create_timer(.05, self.snapshot)

    def motion(self, topic, message):
        twist = message.twist.twist if isinstance(message, Odometry) else message.twist
        row = {'wall': time.monotonic(), 'topic': topic, 'stamp_ns': stamp(message.header.stamp),
               'v': twist.linear.x, 'w': twist.angular.z}
        if isinstance(message, Odometry):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            row['pose'] = [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
        else:
            row['axes'] = [twist.linear.x, twist.linear.y, twist.linear.z,
                           twist.angular.x, twist.angular.y, twist.angular.z]
            row['frame'] = message.header.frame_id
        record(row)
        if topic != '/cmd_vel/requested': latest[topic] = row

    def intent(self, topic, message):
        header = message.command.header if isinstance(message, ManualCommand) else message.header
        row = {'wall': time.monotonic(), 'topic': topic, 'stamp_ns': stamp(header.stamp),
               'epoch': message.epoch, 'sequence': message.sequence}
        if isinstance(message, ManualCommand):
            row.update(deadman=message.deadman, v=message.command.twist.linear.x,
                       w=message.command.twist.angular.z)
        record(row)

    def operator(self, message):
        row = {'wall': time.monotonic(), 'topic': '/operator/state', 'stamp_ns': stamp(message.header.stamp),
               'owner_id': message.owner_id, 'base_active': message.base_active,
               'mode': message.mode, 'phase': message.phase, 'stopped': message.stopped, 'reason': message.reason,
               'session_id': message.session_id}
        record(row); latest['/operator/state'] = row

    def snapshot(self):
        raw.flush()
        payload = (json.dumps(latest)+'\n').encode()
        if len(payload) > 4096:
            raise RuntimeError('latest-value IPC exceeded its bounded frame')
        try: os.write(a.pipe_fd, payload)
        except BlockingIOError: pass
        except BrokenPipeError: rclpy.try_shutdown()


rclpy.init(args=[])
node = Observer()
try:
    while rclpy.ok(): rclpy.spin_once(node, timeout_sec=.05)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
    raw.flush(); raw.close(); os.close(a.pipe_fd)
