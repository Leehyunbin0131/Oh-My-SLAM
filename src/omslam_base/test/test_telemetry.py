import math
from types import SimpleNamespace as NS

import pytest

from omslam_base.telemetry import (AckObservation, JointTelemetry, MotorSnapshot,
                                   encoder_snapshot, serial_snapshot, with_sim_target,
                                   write_motor_message)


def test_serial_snapshot_preserves_units_calibration_subsample_age_and_unavailable_fields():
    calibration=NS(wheels={
        'left': {'encoder_index':1, 'driver_channel':'R', 'forward_encoder_sign':-1, 'forward_command_sign':-1},
        'right': {'encoder_index':0, 'driver_channel':'L', 'forward_encoder_sign':1, 'forward_command_sign':1}})
    measured={'stamp_ns':2_000_000_000, 'positions':[1.,2.], 'velocities':[math.pi,2*math.pi]}
    encoder=encoder_snapshot(100, [40,30], [35,20], measured, 10., 'serial')
    client=NS(latest={'stamp':100,'received_at':10.,'raw_counts':[30,0xffffffd8],
        'counts':[30,-40], 'ab':[1,2], 'invalid':[3,4], 'command_pwm':[70,-80],
        'target_rpm_centi':[1200,-1000], 'measured_rpm_centi':[1100,-950],
        'armed':True, 'fault':0, 'fault_reason':'none', 'sequence':42},
        control={'stamp':99,'received_at':9.99,'mode':'ACTIVE','applied_sequence':41,
                 'wheel_states':['TRACK','HOLD'],'position_error':[7,-8]},
        timing={'stamp_us':99000,'received_at':9.95,'missed_ticks':2},
        receive_diagnostic={'stamp':98,'received_at':9.9,'crc_errors':5},
        diagnostics=None, decoder=NS(errors=6))
    ack=AckObservation();ack.observe({42:['OK','accepted']},10.01)
    snapshot=serial_snapshot(client,calibration,encoder,(100,2_010_000_000),ack)
    client.latest['raw_counts'][0]=999
    client.control['position_error'][0]=999
    values=snapshot.values(10.2)
    assert snapshot.observation_ns==2_000_000_000
    assert values['raw_counts']==[0xffffffd8,30] and values['counts']==[40,30]
    assert values['delta_counts']==[5,10] and values['encoder_rpm']==pytest.approx([30.,60.])
    assert values['target_rpm']==[10.,12.] and values['measured_rpm']==[9.5,11.]
    assert values['pwm']==[80,70] and values['position_error']==[-8,7]
    assert values['feedback_age_s']==pytest.approx(.2)
    assert values['counters']['control.age_s']==pytest.approx(.21)
    assert values['counters']['timing.age_s']==pytest.approx(.25)
    assert values['counters']['receive.crc_errors']==5
    assert 'counters.receive.crc_errors' in values['valid_fields']
    assert values['host_decoder_errors']==6
    assert 'pid_mode' not in values['valid_fields'] and 'settled' not in values['valid_fields']
    ack.observe({42:['OK','accepted']},10.2)
    assert ack.latest[-1]==10.01  # polling the same ACK is not a new observation
    assert values['ack_age_s']==pytest.approx(.19)


def test_sim_joint_measurements_and_target_do_not_fabricate_mcu_fields_or_freshness():
    joints=JointTelemetry()
    names=['right_wheel_joint','caster','left_wheel_joint']
    assert joints.accept(names,[0.,0.,0.],[2.,0.,1.],1_000_000_000,5.)
    first=joints.snapshot.values(5.)
    assert first['velocity_rad_s']==[1.,2.] and 'encoder_rpm' not in first['valid_fields']
    assert joints.accept(names,[.2,0.,.1],[2.,0.,1.],1_100_000_000,5.1)
    assert not joints.accept(names,[9.,0.,9.],[9.,0.,9.],1_100_000_000,5.2)
    snapshot=with_sim_target(joints.snapshot,[20.,30.],1_500_000_000)
    values=snapshot.values(5.7)
    assert values['encoder_rpm']==pytest.approx([30/math.pi,60/math.pi])
    assert values['target_rpm']==[20.,30.] and 'host_command_to_gazebo' in values['source']
    assert values['feedback_age_s']==pytest.approx(.6)
    assert snapshot.observation_ns==1_100_000_000
    for unavailable in ('raw_counts','counts','measured_rpm','pwm','armed','mcu_stamp_ms','ack_sequence'):
        assert unavailable not in values['valid_fields']
    assert not joints.accept(names,[math.nan,0.,math.nan],[],1_200_000_000,5.8)


def test_first_mock_sample_and_ros_message_keep_unknown_velocity_and_original_time():
    measurement={'stamp_ns':3_100_000_000, 'positions':[1.,2.], 'velocities':[0.,0.]}
    snapshot=encoder_snapshot(10,[3,4],None,measurement,4.,'mock')
    values=snapshot.values(4.2)
    assert 'delta_counts' not in values['valid_fields'] and 'velocity_rad_s' not in values['valid_fields']
    assert 'mcu_stamp_ms' not in values['valid_fields']
    message=NS(header=NS(stamp=NS()),observation_stamp=NS())
    write_motor_message(message,snapshot,4.2,9_200_000_000,NS)
    assert (message.header.stamp.sec,message.header.stamp.nanosec)==(9,200_000_000)
    assert (message.observation_stamp.sec,message.observation_stamp.nanosec)==(3,100_000_000)
    assert message.feedback_age_s==pytest.approx(.2)
