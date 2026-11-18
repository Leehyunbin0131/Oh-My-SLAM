"""Read-only motor snapshots; unavailable measurements never become zeroes.

All arrays use physical left/right order. MCU control-state arrays retain their
reported values/signs, reordered by the calibrated driver channel. Encoder and
RPM/PWM fields explicitly apply the calibration's physical-forward convention.
This module does not own or operate a transport.
"""
from copy import deepcopy
from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class MotorSnapshot:
    observation_ns: int
    received_at: float
    fields: dict
    counters: dict = field(default_factory=dict)
    receipts: dict = field(default_factory=dict)

    def values(self, now):
        values = deepcopy(self.fields)
        counters = deepcopy(self.counters)
        if now >= self.received_at:
            values['feedback_age_s'] = now - self.received_at
        for name, received in self.receipts.items():
            if now < received:
                continue
            if name == 'ack_age_s':
                values[name] = now - received
            else:
                counters[name] = now - received
        values['valid_fields'] = sorted(
            [*values, *(['observation_stamp'] if self.observation_ns > 0 else []),
             *['counters.' + key for key in counters]])
        values['counters'] = counters
        return values


def write_motor_message(message, snapshot, now, published_ns, key_value):
    """Populate a newly allocated ROS message; transport time is not sample time."""
    message.header.frame_id = 'base_footprint'
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(published_ns, 1_000_000_000)
    message.observation_stamp.sec, message.observation_stamp.nanosec = divmod(snapshot.observation_ns, 1_000_000_000)
    values = snapshot.values(now)
    counters = values.pop('counters')
    for name, value in values.items():
        setattr(message, name, value)
    message.counters = [key_value(key=key, value=str(value)) for key, value in sorted(counters.items())]
    return message


def encoder_snapshot(tick, counts, previous_counts, measurement, received_at, backend):
    fields = {
        'backend': backend, 'source': 'mock_encoder' if backend == 'mock' else 'mcu_encoder',
        'counts': list(counts), 'position_rad': list(measurement['positions']),
        'velocity_rad_s': list(measurement['velocities']),
    }
    # The first count has no preceding interval; its initialized velocity is
    # useful to odometry but must not be advertised as a measured wheel speed.
    if previous_counts is None:
        fields.pop('velocity_rad_s')
    else:
        fields['delta_counts'] = [new - old for new, old in zip(counts, previous_counts)]
        fields['encoder_rpm'] = [v * 30. / math.pi for v in measurement['velocities']]
    return MotorSnapshot(measurement['stamp_ns'], received_at, fields,
                         {'source.tick_ms': tick})


class AckObservation:
    """Remember an ACK when poll observes it, including ACKs consumed by command()."""
    def __init__(self):
        self.previous = {}
        self.latest = None

    def observe(self, acknowledgements, now):
        current = {int(k): tuple(v) for k, v in acknowledgements.items()}
        for sequence, response in current.items():
            if self.previous.get(sequence) != response and len(response) == 2:
                self.latest = (sequence, response[0], response[1], now)
        self.previous = current


def serial_snapshot(client, calibration, encoder, raw_receipt, acknowledgements):
    """Called only by the existing I/O worker, after one protocol poll batch."""
    sample = client.latest
    if sample is None or raw_receipt is None or sample['stamp'] != raw_receipt[0]:
        return None
    fields = {'backend': 'serial', 'source': 'mcu_omr1; derived=encoder_count_difference'}
    observation_ns = raw_receipt[1]
    if encoder is not None and encoder.counters['source.tick_ms'] == sample['stamp']:
        fields.update(deepcopy(encoder.fields))
        fields.update(backend='serial', source='mcu_omr1; derived=encoder_count_difference')
        observation_ns = encoder.observation_ns
    specs = [calibration.wheels[name] for name in ('left', 'right')]
    encoder_indices = [spec['encoder_index'] for spec in specs]
    channels = [0 if spec['driver_channel'] == 'L' else 1 for spec in specs]
    for source, output in [('raw_counts', 'raw_counts'), ('ab', 'ab'), ('invalid', 'invalid_transitions')]:
        if source in sample:
            fields[output] = [sample[source][i] for i in encoder_indices]
    if 'counts' in sample:
        fields['counts'] = [sample['counts'][i] * spec['forward_encoder_sign']
                            for i, spec in zip(encoder_indices, specs)]
    for source, output, divisor, sign in [
        ('target_rpm_centi', 'target_rpm', 100., 'forward_command_sign'),
        ('measured_rpm_centi', 'measured_rpm', 100., 'forward_encoder_sign'),
        ('command_pwm', 'pwm', 1, 'forward_command_sign'),
    ]:
        if source in sample:
            fields[output] = [sample[source][i] * spec[sign] / divisor
                              for i, spec in zip(channels, specs)]
            if output == 'pwm':
                fields[output] = [int(value) for value in fields[output]]
    for name in ('armed', 'pid_mode', 'fault', 'fault_reason', 'sequence'):
        if name in sample:
            fields[name] = sample[name]
    fields['mcu_stamp_ms'] = sample['stamp']
    counters = {}
    receipts = {}
    if 'link' in sample:
        counters['telemetry.link'] = sample['link']
    for name, spec in zip(('left', 'right'), specs):
        for key in ('encoder_index', 'driver_channel', 'forward_encoder_sign', 'forward_command_sign'):
            counters['mapping.' + name + '.' + key] = spec[key]
    control = getattr(client, 'control', None)
    if control:
        for name in ('applied_sequence', 'applied_tick', 'target_age_ms', 'mode'):
            if name in control:
                fields[name] = control[name]
        for name in ('wheel_states', 'settled', 'limited', 'hold_reference', 'position_error'):
            if name in control:
                fields[name] = [control[name][i] for i in channels]
        counters['control.stamp_ms'] = control['stamp']
        counters['control.array_convention'] = 'physical_left_right; raw_controller_sign'
        if 'received_at' in control:
            receipts['control.age_s'] = control['received_at']
    for prefix, attribute in [('timing', 'timing'), ('receive', 'receive_diagnostic'),
                              ('encoder_sampler', 'diagnostics')]:
        group = getattr(client, attribute, None)
        if group:
            for name, value in group.items():
                if value is None:
                    continue
                if name == 'received_at':
                    receipts[prefix + '.age_s'] = value
                else:
                    counters[prefix + '.' + name] = deepcopy(value)
    decoder = getattr(client, 'decoder', None)
    if decoder is not None:
        fields['host_decoder_errors'] = decoder.errors
    if acknowledgements.latest:
        sequence, status, reason, received = acknowledgements.latest
        fields.update(ack_sequence=sequence, ack_status=status, ack_reason=reason)
        receipts['ack_age_s'] = received
        counters['ack.timestamp_source'] = 'host_poll_observed'
    return MotorSnapshot(observation_ns, sample['received_at'], deepcopy(fields), counters, receipts)


class JointTelemetry:
    """Gazebo joint measurements, without fabricated MCU/encoder counters."""
    def __init__(self):
        self.previous = None
        self.snapshot = None

    def accept(self, names, positions, velocities, stamp_ns, received_at):
        if stamp_ns <= 0 or (self.previous and stamp_ns <= self.previous[0]):
            return False
        try:
            indices = [names.index(name) for name in ('left_wheel_joint', 'right_wheel_joint')]
        except ValueError:
            return False
        fields = {'backend': 'sim', 'source': 'gazebo_joint_states; encoder_rpm=joint_position_difference'}
        position = None
        for source, output in [(positions, 'position_rad'), (velocities, 'velocity_rad_s')]:
            if all(i < len(source) and math.isfinite(source[i]) for i in indices):
                fields[output] = [float(source[i]) for i in indices]
                if output == 'position_rad':
                    position = fields[output]
        if len(fields) == 2:
            return False
        if position is not None and self.previous is not None and self.previous[1] is not None:
            dt = (stamp_ns - self.previous[0]) / 1e9
            if dt <= .4:
                fields['encoder_rpm'] = [(p - old) / dt * 30. / math.pi
                                         for p, old in zip(position, self.previous[1])]
        self.previous = (stamp_ns, position)
        self.snapshot = MotorSnapshot(stamp_ns, received_at, fields)
        return True


def with_sim_target(snapshot, target_rpm, stamp_ns):
    fields = deepcopy(snapshot.fields)
    fields['target_rpm'] = list(target_rpm)
    fields['source'] += '; target_rpm=host_command_to_gazebo'
    counters = {**snapshot.counters, 'target.observation_ros_ns': stamp_ns,
                'target.source': '/sim/cmd_vel; command_not_measurement'}
    return MotorSnapshot(snapshot.observation_ns, snapshot.received_at, fields, counters)
