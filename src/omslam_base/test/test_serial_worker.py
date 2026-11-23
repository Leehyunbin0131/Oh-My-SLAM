"""Exercise the real serial-worker wiring with recording control API doubles.

No OS device is opened; protocol/lease behavior is covered by the vendor's PTY
suite. This catches miswired calibrated counts, callback ordering, issued_at,
and attempts to operate the transport from the executor thread.
"""
from collections import deque
import os
from types import SimpleNamespace
import signal
import subprocess
import sys
import threading
import pytest
import time

from omslam_base.core import CommandGate, Geometry, EncoderOdometry
from omslam_base.node import BaseBridge


def test_serial_worker_uses_public_api_and_measured_feedback(monkeypatch, tmp_path):
    events=[]
    params={'drive_calibration':'drive.json','pid_config':'pid.json','port':'fake-port','report_dir':str(tmp_path)}
    calibration=SimpleNamespace(wheel_data=lambda sample: {
        'left':{'count':-sample['counts'][1]}, 'right':{'count':sample['counts'][0]}}, wheels={
        'left': {'encoder_index':1,'driver_channel':'R','forward_encoder_sign':-1,'forward_command_sign':-1},
        'right': {'encoder_index':0,'driver_channel':'L','forward_encoder_sign':1,'forward_command_sign':1}})
    def record(name,*args):events.append((name,threading.get_ident(),args))
    settings=SimpleNamespace(max_rpm=95.,configure=lambda client:record('configure',client))
    class Client:
        def __init__(self,port,report,**kwargs):
            record('construct',port);self.callback=kwargs['on_sample'];self.n=0
            self.latest=None;self.acks={};self.decoder=SimpleNamespace(errors=0)
            assert kwargs['calibration'] is calibration
        def __enter__(self):
            record('open');self.emit();return self
        def emit(self):
            self.latest={'stamp':1000+self.n*20,'counts':[self.n*5,-self.n*5],
                         'raw_counts':[self.n*5,(-self.n*5)&0xffffffff], 'ab':[0,3], 'invalid':[1,2],
                         'target_rpm_centi':[1200,-1000], 'measured_rpm_centi':[1050,-900],
                         'command_pwm':[100,-100], 'received_at':time.monotonic()}
            self.callback(self.latest)
        def poll(self,block):
            assert block is False
            record('poll');self.n+=1;self.emit()
            self.acks[self.n]=['OK','accepted']
            if self.n==4:stub.exit_event.set()
        def close(self):record('close')
    class Drive:
        def __init__(self,client,config,*,clock):
            assert config is settings
            record('drive')
        def set_target_rpm(self,left,right,*,issued_at):record('target',left,right,issued_at)
        def tick(self):record('tick')
        def disable(self):record('stop')
    modules={'drive_calibration':SimpleNamespace(DriveCalibration=SimpleNamespace(load=lambda path:calibration)),
             'motor_pid':SimpleNamespace(PIDSettings=SimpleNamespace(load=lambda path,cal:settings)),
             'motor_link':SimpleNamespace(MotorClient=Client),'wheel_drive':SimpleNamespace(WheelDrive=Drive)}
    monkeypatch.setattr('omslam_base.node.load_control_module',lambda name:modules[name])
    gate=CommandGate();gate.activate(1)
    now=time.monotonic();gate.requested(1000000000,1000000000,now)
    gate.final(1000000000,.1,.2,1000000000,now)
    stub=SimpleNamespace(backend='serial',value=lambda name:params[name],lock=threading.RLock(),
                         geometry=Geometry(),gate=gate,active=True,error='',ready=False,received_at=None,
                         last_ros=None,output=[0.,0.,0.],exit_event=threading.Event(),stopped=threading.Event(),
                         integrator=EncoderOdometry(Geometry()),sample_queue=deque())
    stub.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000+round((time.monotonic()-now)*1e9)))
    stub.ingest=lambda *args:BaseBridge.ingest(stub,*args)
    stub.fail=lambda message:BaseBridge.fail(stub,message)
    worker=threading.Thread(target=BaseBridge.worker,args=(stub,))
    worker.start();worker.join(2.)
    assert not worker.is_alive() and not stub.error
    assert events[-1][0]=='close'
    assert {e[1] for e in events}=={worker.ident}
    targets=[e[2] for e in events if e[0]=='target']
    assert targets and targets[0][2]==now
    assert targets[0][:2]==stub.geometry.rpm(.1,.2)
    assert len(stub.sample_queue)==5
    assert stub.sample_queue[-1]['x']>0
    assert stub.sample_queue[-1]['yaw']==0
    assert stub.output==[0.,0.,0.]


@pytest.mark.parametrize('event', ['stop', 'deactivate', 'fault'])
def test_serial_worker_invalidates_delayed_arm_and_latches_idle_fault(monkeypatch, tmp_path, event):
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import TwistStamped
    from rclpy.lifecycle import TransitionCallbackReturn
    from rclpy.time import Time
    from omslam_base.vendor import load_control_module

    wheel_module = load_control_module('wheel_drive')
    arm, resume, stopped, fault_received = [threading.Event() for _ in range(4)]
    gate_cleared = threading.Event()
    wire = []
    calibration = SimpleNamespace(wheels={
        'left': {'encoder_index': 0, 'driver_channel': 'L', 'forward_encoder_sign': 1, 'forward_command_sign': 1},
        'right': {'encoder_index': 1, 'driver_channel': 'R', 'forward_encoder_sign': 1, 'forward_command_sign': 1}},
        wheel_data=lambda s: {name: {'count': 0} for name in ('left', 'right')})
    settings = SimpleNamespace(max_rpm=95., configure=lambda client: None, calibration=calibration,
                               commands=lambda *rpm: rpm)

    class Client:
        def __init__(self, port, report, *, calibration, on_sample):
            self.calibration, self.on_sample = calibration, on_sample
            self.info, self.acks, self.n = {'stream_supported': True}, {}, 0

        def __enter__(self):
            self.latest = None
            return self

        def poll(self, *, block):
            self.n += 1
            self.latest = {'stamp': 1000+self.n*10, 'counts': [0, 0], 'received_at': time.monotonic(),
                           'fault': 2 if event == 'fault' else 0, 'fault_reason': 'DRIVER_IO'}
            self.on_sample(self.latest)
            if self.n >= 2:
                fault_received.set()

        def command(self, op):
            wire.append(op)
            arm.set()
            assert resume.wait(2.)

        def set_speed(self, left, right, *, lease_ms):
            wire.append((left, right))
            if left == right == 0:
                stopped.set()

        def send(self, op):
            wire.append(op)

        def close(self):
            pass

    class StopAck(threading.Event):
        def clear(self):
            super().clear()
            gate_cleared.set()

    modules = {'drive_calibration': SimpleNamespace(DriveCalibration=SimpleNamespace(load=lambda path: calibration)),
               'motor_pid': SimpleNamespace(PIDSettings=SimpleNamespace(load=lambda path, cal: settings)),
               'motor_link': SimpleNamespace(MotorClient=Client), 'wheel_drive': wheel_module}
    monkeypatch.setattr('omslam_base.node.load_control_module', modules.__getitem__)
    start = time.monotonic()
    clock = SimpleNamespace(now=lambda: Time(nanoseconds=1_000_000_000+round((time.monotonic()-start)*1e9)))
    params = {'port': 'fake', 'report_dir': tmp_path, 'drive_calibration': '', 'pid_config': '',
              'wheel_vx_variance': .0025, 'wheel_wz_variance': .01}
    node = SimpleNamespace(backend='serial', lock=threading.RLock(), value=params.__getitem__,
        get_clock=lambda: clock, get_logger=lambda: SimpleNamespace(error=lambda *args: None),
        geometry=Geometry(), integrator=EncoderOdometry(Geometry()), gate=CommandGate(), error='',
        active=event != 'fault', ready=False, received_at=None, last_ros=None, configured=True,
        sample_queue=deque(), output=[0.,0.,0.], exit_event=threading.Event(), stopped=StopAck(),
        encoder_telemetry=None, motor_telemetry=None, raw_motor_receipt=None, stamp=BaseBridge.stamp,
        wheel_vx_variance=.0025, wheel_wz_variance=.01)
    node.ingest = lambda *args: BaseBridge.ingest(node, *args)
    node.fail = lambda message: BaseBridge.fail(node, message)
    node.valid_command = lambda msg: BaseBridge.valid_command(node, msg)
    msg = TwistStamped()
    msg.header.frame_id = 'base_footprint'
    msg.header.stamp = clock.now().to_msg()
    if node.active:
        node.gate.activate(1)
        msg.twist.linear.x = .1
        BaseBridge.requested(node, msg)
        BaseBridge.command(node, msg)
    node.thread = threading.Thread(target=BaseBridge.worker, args=(node,))
    node.thread.start()
    deactivation = None
    try:
        if event == 'fault':
            assert fault_received.wait(2.)
            with node.lock:
                assert BaseBridge.on_activate(node, None) == TransitionCallbackReturn.FAILURE
                assert 'MCU fault 2' in node.error and not node.active
                messages = []
                node.last_diag = 0.
                node.last_diag_error = None
                node.fault_deactivate_pending = False
                node.last_motor_publish = time.monotonic()
                node.odom_pub = node.joint_pub = SimpleNamespace(publish=lambda msg: None)
                node.diag_pub = SimpleNamespace(publish=messages.append)
                BaseBridge.publish(node)
                assert messages[-1].status[0].level == DiagnosticStatus.ERROR
        else:
            assert arm.wait(2.)
            if event == 'stop':
                msg.twist.linear.x = 0.
                BaseBridge.command(node, msg)
                assert node.gate.current(time.monotonic())[:2] == (0., 0.)
            else:
                deactivation = threading.Thread(target=BaseBridge.stop_accepting, args=(node,))
                deactivation.start()
                assert gate_cleared.wait(2.)
            resume.set()
            assert (stopped if event == 'stop' else node.stopped).wait(2.)
            assert not [frame for frame in wire if isinstance(frame, tuple) and any(frame)]
            assert (0, 0) in wire if event == 'stop' else 'STOP' in wire
    finally:
        resume.set()
        if deactivation:
            deactivation.join(2.)
        node.exit_event.set()
        node.thread.join(2.)
    assert not node.thread.is_alive()


def test_motor_log_preserves_latest_rotation(tmp_path):
    from omslam_base.node import MotorLog
    for mode, sample in [('ab', b'abcd'), ('a', 'abcd')]:
        path = tmp_path / mode
        log = MotorLog(path.open(mode), max_bytes=7)
        for _ in range(3):
            log.write(sample)
        log.close()
        assert path.read_bytes() == path.with_name(path.name + '.1').read_bytes() == b'abcd'


T0 = 100.


def scripted_worker(monkeypatch, tmp_path, backend, steps):
    """Run one worker loop iteration per (time, intent vx, write delay[, write vx]) step.

    The worker, CommandGate and the pinned WheelDrive read only this scripted
    monotonic clock. A write delay advances it after WheelDrive.tick() selected
    a lease and before SnapshotClient.set_speed() checks it. An optional write
    vx is then delivered as /cmd_vel at that same boundary. Each result holds
    the step's new wire frames, /mock/wheel_target output and drive.engaged.
    """
    from geometry_msgs.msg import TwistStamped
    from rclpy.time import Time
    from omslam_base.vendor import load_control_module

    wheel_module = load_control_module('wheel_drive')
    clock = SimpleNamespace(t=steps[0][0], delay=0., mark=0, write_vx=None)
    monkeypatch.setattr('omslam_base.node.time', SimpleNamespace(monotonic=lambda: clock.t))
    wire, results, drives, script = [], [], [], list(steps)
    calibration = SimpleNamespace(wheels={
        'left': {'encoder_index': 0, 'driver_channel': 'L', 'forward_encoder_sign': 1, 'forward_command_sign': 1},
        'right': {'encoder_index': 1, 'driver_channel': 'R', 'forward_encoder_sign': 1, 'forward_command_sign': 1}},
        wheel_data=lambda s: {name: {'count': 0} for name in ('left', 'right')})
    settings = SimpleNamespace(max_rpm=95., configure=lambda client: None, calibration=calibration,
                               commands=lambda *rpm: rpm)

    class Client:
        def __init__(self, port, report, *, calibration, on_sample):
            self.calibration, self.on_sample = calibration, on_sample
            self.info, self.acks, self.latest = {'stream_supported': True}, {}, None

        def __enter__(self):
            return self

        def poll(self, *, block):
            self.latest = {'stamp': 1000+round((clock.t-T0)*1000), 'counts': [0, 0],
                           'received_at': clock.t, 'fault': 0, 'fault_reason': ''}
            self.on_sample(self.latest)

        def command(self, op):
            wire.append(op)

        def set_speed(self, left, right, *, lease_ms):
            wire.append(('SET_SPEED', left, right, lease_ms))

        def send(self, op):
            wire.append(op)

        def close(self):
            pass

    class WriteBoundary:
        def __init__(self, client):
            self.client = client

        def __getattr__(self, name):
            return getattr(self.client, name)

        def set_speed(self, *args, **kwargs):
            clock.t += clock.delay
            clock.delay = 0.
            if clock.write_vx is not None:
                vx, clock.write_vx = clock.write_vx, None
                deliver(vx)
            return self.client.set_speed(*args, **kwargs)

    class Drive(wheel_module.WheelDrive):
        def __init__(self, client, config, **kwargs):
            super().__init__(client, config, **kwargs)
            self.client = WriteBoundary(client)
            drives.append(self)

    modules = {'drive_calibration': SimpleNamespace(DriveCalibration=SimpleNamespace(load=lambda path: calibration)),
               'motor_pid': SimpleNamespace(PIDSettings=SimpleNamespace(load=lambda path, cal: settings)),
               'motor_link': SimpleNamespace(MotorClient=Client), 'wheel_drive': SimpleNamespace(WheelDrive=Drive)}
    monkeypatch.setattr('omslam_base.node.load_control_module', modules.__getitem__)
    ros = SimpleNamespace(now=lambda: Time(nanoseconds=1_000_000_000+round((clock.t-T0)*1e9)))
    params = {'port': 'fake', 'report_dir': tmp_path, 'drive_calibration': '', 'pid_config': ''}
    node = SimpleNamespace(backend=backend, lock=threading.RLock(), value=params.__getitem__,
        get_clock=lambda: ros, geometry=Geometry(), integrator=EncoderOdometry(Geometry()),
        gate=CommandGate(), error='', active=True, ready=backend == 'mock', received_at=None,
        last_ros=None, sample_queue=deque(), output=[0., 0., 0.], stopped=threading.Event(),
        encoder_telemetry=None, motor_telemetry=None, raw_motor_receipt=None, telemetry_error='',
        stamp=BaseBridge.stamp)
    node.ingest = lambda *args: BaseBridge.ingest(node, *args)
    node.fail = lambda message: BaseBridge.fail(node, message)
    node.valid_command = lambda msg: BaseBridge.valid_command(node, msg)
    node.gate.activate(1)

    def deliver(vx):
        msg = TwistStamped()
        msg.header.frame_id = 'base_footprint'
        msg.header.stamp = ros.now().to_msg()
        msg.twist.linear.x = vx
        if vx:
            BaseBridge.requested(node, msg)
        BaseBridge.command(node, msg)

    def begin():
        t, vx, clock.delay, *write = script.pop(0)
        clock.t, clock.mark = t, len(wire)
        clock.write_vx = write[0] if write else None
        if vx is not None:
            deliver(vx)

    class Loop:
        done = False

        def is_set(self):
            return self.done

        def set(self):
            self.done = True

        def wait(self, timeout):
            results.append((wire[clock.mark:], list(node.output), drives[0].engaged if drives else None))
            if script:
                begin()
            else:
                self.done = True
            return self.done

    node.exit_event = Loop()
    begin()
    BaseBridge.worker(node)
    assert not node.error, node.error
    return results


@pytest.mark.parametrize('backend', ['serial', 'mock'])
def test_brake_and_moving_lease_lifetime_on_wire_and_mock_target(monkeypatch, tmp_path, backend):
    rpm = Geometry().rpm(.1, 0.)
    left, right = (round(v * 100) for v in rpm)
    results = scripted_worker(monkeypatch, tmp_path, backend, [
        (T0, .1, 0.),          # fresh moving intent
        (T0+.225, None, 0.),   # (a) moving, 75 ms left: last lease expires on the MCU
        (T0+.25, .1, 0.),      # fresh moving intent
        (T0+.30, 0., 0.),      # (b) early zero keeps the original 250 ms braking lease
        (T0+.46, None, 0.),    # (c) late zero, 90 ms left: released as STOP
        (T0+.56, None, 0.),    # (d) the original intent expired
    ])
    outputs = [output for _, output, _ in results]
    assert outputs == [pytest.approx(expected, abs=1e-9) for expected in (
        [*rpm, .3], [*rpm, .075], [*rpm, .3], [0., 0., .25], [0., 0., 0.], [0., 0., 0.])]
    if backend == 'serial':
        assert [frames for frames, _, _ in results] == [
            ['ARM', ('SET_SPEED', left, right, 300)], [], [('SET_SPEED', left, right, 300)],
            [('SET_SPEED', 0, 0, 250)], ['STOP'], ['STOP']]
        assert [engaged for _, _, engaged in results] == [True, True, True, True, False, False]
    else:
        assert all(not frames for frames, _, _ in results)


def test_brake_lease_lost_at_serial_write_stops_until_fresh_intent(monkeypatch, tmp_path):
    left, right = (round(v * 100) for v in Geometry().rpm(.1, 0.))
    results = scripted_worker(monkeypatch, tmp_path, 'serial', [
        (T0, .1, 0.),
        (T0+.05, 0., 0.),      # early zero: legal braking lease
        (T0+.19, None, .015),  # tick() selects 110 ms; 95 ms remain at the write
        (T0+.22, None, 0.),    # the brake target is still selected
        (T0+.25, .1, 0.),      # fresh intent
    ])
    frames = [step[0] for step in results]
    assert frames == [['ARM', ('SET_SPEED', left, right, 300)], [('SET_SPEED', 0, 0, 250)],
                      ['STOP'], [], ['ARM', ('SET_SPEED', left, right, 300)]]
    assert [engaged for _, _, engaged in results] == [True, True, False, False, True]
    assert [output for _, output, _ in results][2:4] == [[0., 0., 0.], [0., 0., 0.]]


def test_repeated_zero_at_serial_write_keeps_original_brake_lease(monkeypatch, tmp_path):
    left, right = (round(v * 100) for v in Geometry().rpm(.1, 0.))
    results = scripted_worker(monkeypatch, tmp_path, 'serial', [
        (T0, .1, 0.),
        (T0+.05, 0., 0.),      # early zero: legal braking lease
        (T0+.07, None, 0., 0.),  # a duplicate zero arrives at the write, 230 ms left
        (T0+.10, None, 0.),    # the same HOLD keeps its original deadline
        (T0+.25, None, 0.),    # 50 ms left: the original brake is released
    ])
    assert [frames for frames, _, _ in results] == [
        ['ARM', ('SET_SPEED', left, right, 300)], [('SET_SPEED', 0, 0, 250)],
        [('SET_SPEED', 0, 0, 230)], [('SET_SPEED', 0, 0, 200)], ['STOP']]
    assert [engaged for _, _, engaged in results] == [True, True, True, True, False]
    assert [output for _, output, _ in results][2:] == [
        pytest.approx(expected, abs=1e-9) for expected in ([0., 0., .23], [0., 0., .2], [0., 0., 0.])]


CONFIGURED_BRIDGE = """
import signal
import sys
import rclpy
from omslam_base import node as base

# Close the worker, then report a failed join as a hung motor worker would.
FAIL_JOIN = sys.argv[1:] == ['fail-join']


class Configured(base.BaseBridge):
    def __init__(self):
        super().__init__()
        self.trigger_configure()
        print('ready', self.configured and self.thread.is_alive(), flush=True)

    def close_worker(self):
        context_valid = rclpy.ok()
        closed = super().close_worker()
        print('closed', context_valid, closed, self.thread.is_alive(), flush=True)
        return closed and not FAIL_JOIN


base.BaseBridge = Configured
try:
    base.main()
finally:
    print('restored', signal.getsignal(signal.SIGINT) is signal.default_int_handler,
          signal.getsignal(signal.SIGTERM) == signal.SIG_DFL, rclpy.ok(), flush=True)
"""


def signal_configured_bridge(tmp_path, signum, *args):
    env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='OFF', ROS_LOG_DIR=str(tmp_path))
    child = subprocess.Popen([sys.executable, '-c', CONFIGURED_BRIDGE, *args], stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True, env=env)
    watchdog = threading.Timer(30., child.kill)
    watchdog.start()
    try:
        assert child.stdout.readline().split() == ['ready', 'True']
        child.send_signal(signum)
        out, err = child.communicate()
    finally:
        watchdog.cancel()
        if child.poll() is None:
            child.kill()
            child.communicate()
    return child.returncode, out, err


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_signal_closes_worker_while_context_is_valid(tmp_path, signum):
    returncode, out, err = signal_configured_bridge(tmp_path, signum)
    assert returncode == 0, err
    assert out.splitlines() == ['closed True True False', 'restored True True False']


def test_signal_exits_nonzero_when_worker_join_fails(tmp_path):
    returncode, out, err = signal_configured_bridge(tmp_path, signal.SIGTERM, 'fail-join')
    assert returncode != 0
    assert 'base_bridge: motor I/O worker did not stop' in err
    # Teardown still runs in order before the failure exits the process.
    assert out.splitlines() == ['closed True True False', 'restored True True False']
