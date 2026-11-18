"""Exercise the real serial-worker wiring with recording control API doubles.

No OS device is opened; protocol/lease behavior is covered by the vendor's PTY
suite. This catches miswired calibrated counts, callback ordering, issued_at,
and attempts to operate the transport from the executor thread.
"""
from collections import deque
from types import SimpleNamespace
import threading
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
        def __init__(self,client,config):
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
