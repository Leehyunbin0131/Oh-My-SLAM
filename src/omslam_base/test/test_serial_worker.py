"""Exercise the real serial-worker wiring with recording control API doubles.

No OS device is opened; protocol/lease behavior is covered by the vendor's PTY
suite. This catches miswired calibrated counts, callback ordering, issued_at,
and attempts to operate the transport from the executor thread.
"""
from collections import deque
from types import SimpleNamespace
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
    values=stub.motor_telemetry.values(time.monotonic())
    assert values['backend']=='serial' and values['counts']==[20,20]
    assert values['measured_rpm']==[9.,10.5] and values['target_rpm']==[10.,12.]
    assert values['encoder_rpm'] != values['measured_rpm']
    assert values['ack_sequence']==4 and values['ack_status']=='OK'
    assert not stub.telemetry_error


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
        encoder_telemetry=None, motor_telemetry=None, raw_motor_receipt=None, stamp=BaseBridge.stamp)
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
                assert node.motor_telemetry.fields['fault'] == 2
                assert BaseBridge.on_activate(node, None) == TransitionCallbackReturn.FAILURE
                assert 'MCU fault 2' in node.error and not node.active
                messages = []
                node.last_diag = 0.
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
