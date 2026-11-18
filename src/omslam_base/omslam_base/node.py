"""Lifecycle base; a single worker owns all motor transport operations."""
from collections import deque
from pathlib import Path
import math
import threading
import time

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray, Float64MultiArray, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
from omslam_operator_msgs.msg import MotorTelemetry
from .core import Geometry, EncoderOdometry, CommandGate
from .telemetry import AckObservation, encoder_snapshot, serial_snapshot, write_motor_message
from .vendor import load_control_module


class BaseBridge(LifecycleNode):
    def __init__(self):
        super().__init__('base_bridge')
        defaults = dict(backend='mock', port='', drive_calibration='', pid_config='',
                        report_dir='/tmp/omslam-base', left_radius=.068, right_radius=.068,
                        wheel_separation=.480, counts_per_rev=2652, max_rpm=95.,
                        wheel_vx_variance=.0025, wheel_wz_variance=.01)
        for name,value in defaults.items(): self.declare_parameter(name,value)
        self.lock = threading.RLock()
        self.gate = CommandGate()
        self.configured = self.active = False
        self.ready = False
        self.error = ''
        self.received_at = None
        self.sample_queue = deque(maxlen=200)
        self.output = [0.,0.,0.]
        self.thread = None
        self.exit_event = threading.Event()
        self.stopped = threading.Event()
        self.stopped.set()
        self.geometry = None
        self.last_ros = None
        self.encoder_telemetry = self.motor_telemetry = None
        self.raw_motor_receipt = None
        self.last_motor_publish = 0.
        self.telemetry_error = ''
        self.create_subscription(TwistStamped,'/cmd_vel/requested',self.requested,1)
        self.create_subscription(TwistStamped,'/cmd_vel',self.command,1)
        self.create_subscription(Int64MultiArray,'/mock/encoder_sample',self.mock_sample,10)
        self.create_subscription(Bool,'/mock/motor_fault',self.mock_fault,1)
        self.odom_pub = self.create_publisher(Odometry,'/wheel/odometry',10)
        self.joint_pub = self.create_publisher(JointState,'/joint_states',10)
        self.diag_pub = self.create_publisher(DiagnosticArray,'/diagnostics',10)
        self.motor_pub = self.create_publisher(MotorTelemetry,'/base/motor_state',10)
        self.target_pub = self.create_publisher(Float64MultiArray,'/mock/wheel_target',10)
        self.timer = self.create_timer(.02,self.publish,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_diag = 0.
        self.add_on_set_parameters_callback(self.parameters)

    def parameters(self, params):
        if self.configured:
            return SetParametersResult(successful=False, reason='cleanup before changing base parameters')
        return SetParametersResult(successful=True)

    def value(self,name): return self.get_parameter(name).value

    def on_configure(self,state):
        try:
            self.backend = self.value('backend')
            if self.backend not in ('mock','serial'): raise ValueError('backend must be mock or serial')
            if self.backend == 'serial':
                if self.value('use_sim_time'): raise ValueError('serial backend forbids use_sim_time')
                if not all(self.value(k) for k in ('port','drive_calibration','pid_config')):
                    raise ValueError('serial requires explicit port, drive_calibration and pid_config')
            self.geometry = Geometry(self.value('left_radius'), self.value('right_radius'),
                                     self.value('wheel_separation'), self.value('counts_per_rev'),self.value('max_rpm'))
            if self.geometry.max_rpm > 95.:
                raise ValueError('Base bridge operating limit must not exceed 95 RPM')
            if any(not math.isfinite(self.value(n)) or self.value(n) <= 0 for n in
                   ('wheel_vx_variance','wheel_wz_variance')): raise ValueError('invalid covariance')
            with self.lock:
                self.integrator = EncoderOdometry(self.geometry)
                self.gate.clear(); self.sample_queue.clear()
                self.active = False; self.ready = self.backend == 'mock'; self.error = ''
                self.received_at = None; self.last_ros = None
                self.encoder_telemetry = self.motor_telemetry = None
                self.raw_motor_receipt = None
                self.telemetry_error = ''
                self.output = [0.,0.,0.]; self.configured = True
            self.exit_event.clear(); self.stopped.clear()
            self.thread = threading.Thread(target=self.worker,daemon=True,name='motor_io')
            self.thread.start()
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(str(exc))
            return TransitionCallbackReturn.FAILURE

    def on_activate(self,state):
        with self.lock:
            if (self.error or not self.ready or self.received_at is None
                    or time.monotonic()-self.received_at > .4):
                self.get_logger().error('Activation requires ready transport and fresh encoder feedback; '+self.error)
                return TransitionCallbackReturn.FAILURE
            self.gate.activate(self.get_clock().now().nanoseconds)
            self.active = True
        return TransitionCallbackReturn.SUCCESS

    def stop_accepting(self):
        with self.lock:
            self.active = False; self.gate.clear(); self.output = [0.,0.,0.]
            self.stopped.clear()
        if self.thread and self.thread.is_alive():
            if not self.stopped.wait(1.):
                self.get_logger().error('Worker stop acknowledgement timed out; MCU lease remains bounded')
                return False
        return True

    def on_deactivate(self,state):
        return TransitionCallbackReturn.SUCCESS if self.stop_accepting() else TransitionCallbackReturn.FAILURE

    def close_worker(self):
        self.stop_accepting()
        self.exit_event.set()
        if self.thread:
            self.thread.join(10.)
            if self.thread.is_alive(): return False
        with self.lock:
            self.configured = self.ready = False
            self.sample_queue.clear()
        return True

    def on_cleanup(self,state):
        return TransitionCallbackReturn.SUCCESS if self.close_worker() else TransitionCallbackReturn.FAILURE

    def on_shutdown(self,state):
        return TransitionCallbackReturn.SUCCESS if self.close_worker() else TransitionCallbackReturn.FAILURE

    def on_error(self,state):
        self.close_worker()
        return TransitionCallbackReturn.SUCCESS

    def fail(self, message):
        with self.lock:
            self.error = str(message); self.active = False; self.gate.clear(); self.output = [0.,0.,0.]

    @staticmethod
    def stamp(msg): return msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec

    def valid_command(self,msg):
        t=msg.twist
        return (msg.header.frame_id == 'base_footprint' and
                all(math.isfinite(v) for v in (t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z))
                and all(v == 0 for v in (t.linear.y,t.linear.z,t.angular.x,t.angular.y)))

    def requested(self,msg):
        with self.lock:
            if not self.active: return
            if not self.valid_command(msg):
                self.gate.target = None
                return
            self.gate.requested(self.stamp(msg),self.get_clock().now().nanoseconds,time.monotonic())

    def command(self,msg):
        with self.lock:
            if not self.active: return
            if not self.valid_command(msg):
                self.gate.target = None
                return
            self.gate.final(self.stamp(msg),msg.twist.linear.x,msg.twist.angular.z,
                            self.get_clock().now().nanoseconds,time.monotonic())

    def ingest(self,tick,left,right,received):
        with self.lock:
            if self.error: return
            previous = self.integrator.last
            measurement = self.integrator.update(tick,left,right,self.get_clock().now().nanoseconds)
            if measurement is not None:
                self.received_at = received
                self.sample_queue.append(measurement)
                self.encoder_telemetry = encoder_snapshot(
                    tick, (left,right), None if previous is None else previous[1:],
                    measurement, received, self.backend)
                if self.backend == 'mock': self.motor_telemetry = self.encoder_telemetry

    def mock_sample(self,msg):
        if not self.configured or self.backend != 'mock': return
        try:
            if len(msg.data) != 3: raise ValueError('mock encoder sample requires tick,left,right')
            self.ingest(*(int(v) for v in msg.data),time.monotonic())
        except Exception as exc: self.fail(exc)

    def mock_fault(self,msg):
        if self.configured and self.backend == 'mock' and msg.data: self.fail('Injected mock motor fault')

    def worker(self):
        client = drive = None
        last_target = None
        try:
            if self.backend == 'serial':
                calibration = load_control_module('drive_calibration').DriveCalibration.load(self.value('drive_calibration'))
                settings = load_control_module('motor_pid').PIDSettings.load(self.value('pid_config'),calibration)
                if settings.max_rpm < self.geometry.max_rpm:
                    raise ValueError('Bridge max_rpm exceeds calibrated motor limit')
                if self.geometry.counts_per_rev != 2652: raise ValueError('OMR1 encoder CPR is 2652')
                report=Path(self.value('report_dir')); report.mkdir(parents=True,exist_ok=True)
                def sample_callback(sample):
                    with self.lock:
                        self.raw_motor_receipt = (sample['stamp'], self.get_clock().now().nanoseconds)
                    counts = calibration.wheel_data(sample)
                    self.ingest(sample['stamp'], counts['left']['count'],counts['right']['count'],sample['received_at'])
                acknowledgements = AckObservation()
                parent = self
                class SnapshotClient(load_control_module('motor_link').MotorClient):
                    # MotorClient.command() also calls this override. Capture
                    # before it consumes ACKs; no executor accesses the client.
                    def poll(client, *, block=True):
                        try:
                            return super().poll(block=block)
                        finally:
                            try:
                                acknowledgements.observe(client.acks, time.monotonic())
                                with parent.lock:
                                    parent.motor_telemetry = serial_snapshot(
                                        client, calibration, parent.encoder_telemetry,
                                        parent.raw_motor_receipt, acknowledgements)
                                    parent.telemetry_error = ''
                            except Exception as exc:
                                # An observer must not change transport/stop
                                # handling or replace the original poll error.
                                with parent.lock: parent.telemetry_error = str(exc)
                client = SnapshotClient(self.value('port'),report,
                            calibration=calibration,on_sample=sample_callback)
                client.__enter__()
                settings.configure(client)
                drive = load_control_module('wheel_drive').WheelDrive(client,settings)
                with self.lock: self.ready=True
            disabled = False
            while not self.exit_event.is_set():
                if client: client.poll(block=False)
                now=time.monotonic()
                with self.lock:
                    ros_now=self.get_clock().now().nanoseconds
                    if self.last_ros is not None and ros_now < self.last_ros:
                        self.fail('ROS clock reversed; reconfigure required')
                    self.last_ros=ros_now
                    if self.received_at is not None and now-self.received_at > .4:
                        self.fail('Encoder feedback timed out')
                    target = self.gate.current(now) if self.active and not self.error else None
                    active=self.active
                if target is None:
                    if not disabled and drive: drive.disable()
                    disabled=True; last_target=None
                    with self.lock: self.output=[0.,0.,0.]
                    if not active: self.stopped.set()
                else:
                    vx,wz,issued=target
                    rpms=self.geometry.rpm(vx,wz)
                    remaining=.3-(now-issued)
                    if drive:
                        if target != last_target: drive.set_target_rpm(*rpms,issued_at=issued)
                        drive.tick()
                    # Mirror the minimum wire lease rule for mock diagnostics.
                    if remaining < .1 and not any(rpms): rpms=(0.,0.); remaining=0.
                    with self.lock:
                        if self.active: self.output=[*rpms,max(0.,remaining)]
                    disabled=False; last_target=target
                self.exit_event.wait(.01)
        except Exception as exc:
            self.fail(exc)
        finally:
            if client: client.close()
            with self.lock:
                self.output=[0.,0.,0.]; self.ready=False
            self.stopped.set()

    def publish(self):
        if not self.configured: return
        with self.lock:
            samples=list(self.sample_queue);self.sample_queue.clear()
            output=list(self.output); error=self.error; active=self.active;ready=self.ready
            motor_snapshot = self.motor_telemetry
            telemetry_error = self.telemetry_error
        for sample in samples:
            msg=Odometry(); ns=sample['stamp_ns']
            msg.header.stamp.sec=ns//1000000000;msg.header.stamp.nanosec=ns%1000000000
            msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
            msg.pose.pose.position.x=sample['x'];msg.pose.pose.position.y=sample['y']
            msg.pose.pose.orientation.z=math.sin(sample['yaw']/2);msg.pose.pose.orientation.w=math.cos(sample['yaw']/2)
            msg.twist.twist.linear.x=sample['vx'];msg.twist.twist.angular.z=sample['wz']
            for i in (0,7,14,21,28,35): msg.pose.covariance[i]=1e3
            for i in (7,14,21,28):msg.twist.covariance[i]=1e3
            msg.twist.covariance[0]=self.value('wheel_vx_variance');msg.twist.covariance[35]=self.value('wheel_wz_variance')
            self.odom_pub.publish(msg)
            joint=JointState();joint.header=msg.header;joint.header.frame_id='base_footprint'
            joint.name=['left_wheel_joint','right_wheel_joint'];joint.position=sample['positions'];joint.velocity=sample['velocities']
            self.joint_pub.publish(joint)
        if self.backend == 'mock': self.target_pub.publish(Float64MultiArray(data=output))
        now=time.monotonic()
        if motor_snapshot is not None and now-self.last_motor_publish >= .05:
            msg = write_motor_message(MotorTelemetry(), motor_snapshot, now,
                                      self.get_clock().now().nanoseconds, KeyValue)
            self.motor_pub.publish(msg)
            self.last_motor_publish = now
        if now-self.last_diag >= 1. or error:
            status=DiagnosticStatus();status.name='omslam_base';status.hardware_id=self.backend
            status.level=DiagnosticStatus.ERROR if error else DiagnosticStatus.OK if ready else DiagnosticStatus.WARN
            status.message=error or ('active' if active else 'inactive' if ready else 'connecting')
            status.values=[KeyValue(key='backend',value=self.backend),KeyValue(key='active',value=str(active)),
                           KeyValue(key='feedback_age_s',value=str(None if self.received_at is None else now-self.received_at)),
                           KeyValue(key='motor_telemetry_error',value=telemetry_error)]
            msg=DiagnosticArray();msg.header.stamp=self.get_clock().now().to_msg();msg.status=[status];self.diag_pub.publish(msg)
            self.last_diag=now


def main(args=None):
    rclpy.init(args=args)
    node=BaseBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close_worker();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
