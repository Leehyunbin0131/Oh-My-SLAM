"""Ownership, stopped transitions and replay boundaries for the shared guard."""

import math

from omslam_navigation.operator_core import OperatorGate


FRAME = 'base_footprint'
MOVE = (0.10, 0.0, 0.0, 0.0, 0.0, 0.30)
ZERO = (0.0,) * 6


def sample(gate, seconds, wall, vx=0.0, wz=0.0, heartbeat=True):
    stamp = round(seconds * 1e9)
    gate.clock(stamp, wall)
    gate.health(True, '', wall)
    for key in ('wheel', 'odom'):
        gate.sample_stop(key, stamp, vx, wz, True, wall)
    if heartbeat and gate.token and stamp > gate.owner_stamp:
        assert gate.heartbeat(gate.token, gate.epoch_ns, gate.heartbeat_sequence + 1,
                              stamp, stamp, wall)
    gate.stopped(stamp, wall)
    return stamp


def ready(mode='MANUAL', reverse=False):
    gate = OperatorGate(reverse_allowed=reverse)
    for index in range(12):
        sample(gate, 1.0 + index * 0.1, 10.0 + index * 0.1)
    assert gate.acquire('test-console', 2_100_000_000, 11.1)[0]
    assert gate.begin_transition(mode)[0]
    assert gate.finish_transition(2_100_000_000, 11.1)[0]
    return gate


def release(gate, seconds=2.12, wall=11.12):
    stamp = sample(gate, seconds, wall)
    assert not gate.manual(gate.token, gate.epoch_ns, 1, False, stamp,
                           FRAME, ZERO, stamp, wall)
    assert not gate.release_required


def test_manual_uses_shared_source_lease_without_a_nav_goal():
    gate = ready()
    stamp = sample(gate, 2.11, 11.11)
    # Held input from before a mode change must first be released.
    assert not gate.manual(gate.token, gate.epoch_ns, 1, True, stamp, FRAME, MOVE, stamp, 11.11)
    stamp = sample(gate, 2.12, 11.12)
    assert not gate.manual(gate.token, gate.epoch_ns, 2, False, stamp, FRAME, ZERO, stamp, 11.12)
    stamp = sample(gate, 2.14, 11.14)
    assert gate.manual(gate.token, gate.epoch_ns, 3, True, stamp, FRAME, MOVE, stamp, 11.14)
    assert gate.goal_id is None and gate.goal_status == 0
    assert gate.final(stamp, FRAME, MOVE, stamp, 11.14)
    assert gate.output == (0.1, 0.3)
    assert not gate.requested(stamp, FRAME, MOVE, stamp, 11.14)
    assert not gate.manual(gate.token, gate.epoch_ns, 3, True, stamp, FRAME, MOVE, stamp, 11.14)
    assert not gate.manual(gate.token, gate.epoch_ns, 4, False, stamp, FRAME, ZERO, stamp, 11.15)
    assert gate.output == (0.0, 0.0)
    assert not gate.final(stamp, FRAME, MOVE, stamp, 11.15)


def test_rejected_control_request_does_not_expose_current_owner_token():
    gate = ready()
    assert not gate.authorized('other-session', gate.epoch_ns, 2_100_000_000, 11.1)
    success, message = gate.acquire('other-console', 2_100_000_000, 11.1)
    result = gate.control_result(success, message)
    assert result[0] is False and result[2] == '' and result[3] == 0
    assert gate.token and gate.owner_id == 'test-console'


def test_manual_expiry_is_not_renewed_by_transport_or_owner_heartbeat():
    gate = ready()
    release(gate)
    stamp = sample(gate, 2.14, 11.14)
    assert gate.manual(gate.token, gate.epoch_ns, 2, True, stamp, FRAME, MOVE, stamp, 11.14)
    assert gate.final(stamp, FRAME, MOVE, stamp, 11.14)
    issued = gate.sources[stamp].issued_wall
    now = sample(gate, 2.34, 11.34)
    assert gate.manual(gate.token, gate.epoch_ns, 3, True, stamp, FRAME, MOVE, now, 11.34)
    assert gate.sources[stamp].issued_wall == issued
    now = sample(gate, 2.45, 11.45)
    assert gate.tick(now, 11.45) == (0.0, 0.0)
    assert gate.mode == 'MANUAL' and gate.release_required
    assert not gate.final(stamp, FRAME, MOVE, now, 11.45)
    assert not gate.manual(gate.token, gate.epoch_ns, 4, True, now, FRAME, MOVE, now, 11.45)


def test_owner_expiry_stops_auto_and_revokes_old_tokens():
    gate = ready('AUTO')
    token, epoch = gate.token, gate.epoch_ns
    assert gate.register_goal('owned', 2_100_000_000, 11.1)
    stamp = sample(gate, 2.14, 11.14)
    gate.goals([('owned', 2_120_000_000, 2)], stamp)
    assert gate.requested(stamp, FRAME, MOVE, stamp, 11.14)
    assert gate.final(stamp, FRAME, MOVE, stamp, 11.14)
    now = sample(gate, 2.45, 11.45, heartbeat=False)
    assert gate.tick(now, 11.45) == (0.0, 0.0)
    assert gate.mode == 'DISABLED' and not gate.owner_id and not gate.token
    assert gate.state == 'FAULT_LATCHED' and gate.cancel_required
    assert not gate.heartbeat(token, epoch, 100, now, now, 11.45)


def test_mode_transition_requires_fresh_measured_stop_and_rotates_epoch():
    gate = ready()
    release(gate)
    old_token, old_epoch = gate.token, gate.epoch_ns
    stamp = sample(gate, 2.14, 11.14, vx=0.05)
    assert gate.manual(old_token, old_epoch, 2, True, stamp, FRAME, MOVE, stamp, 11.14)
    assert gate.begin_transition('AUTO')[0]
    assert not gate.finish_transition(stamp, 11.14)[0]
    assert not gate.final(stamp, FRAME, MOVE, stamp, 11.14)
    # A zero command is not a measured stop; stale zero observations cannot be
    # held indefinitely to satisfy the 0.3-second continuous stopped interval.
    for seconds, wall in [(2.20, 11.20), (2.30, 11.30), (2.40, 11.40)]:
        sample(gate, seconds, wall)
        assert not gate.finish_transition(round(seconds * 1e9), wall)[0]
    sample(gate, 2.52, 11.52)
    assert gate.finish_transition(2_520_000_000, 11.52)[0]
    assert gate.mode == 'AUTO' and gate.token != old_token and gate.epoch_ns > old_epoch
    assert not gate.authorized(old_token, old_epoch, 2_520_000_000, 11.52)
    assert not gate.final(stamp, FRAME, MOVE, 2_520_000_000, 11.52)
    gate.clock(2_750_000_000, 11.75)
    assert not gate.stopped(2_750_000_000, 11.75)


def test_only_registered_uuid_can_drive_and_cancel_survives_delayed_status():
    gate = ready('AUTO')
    assert gate.register_goal('owned', 2_100_000_000, 11.1)
    stamp = sample(gate, 2.14, 11.14)
    gate.goals([('owned', 2_120_000_000, 2)], stamp)
    assert gate.requested(stamp, FRAME, MOVE, stamp, 11.14)
    assert gate.final(stamp, FRAME, MOVE, stamp, 11.14)
    gate.cancel_registered_goal()
    gate.goals([('owned', 2_120_000_000, 2)], stamp)
    assert gate.output == (0.0, 0.0) and gate.goal_status == 3
    assert not gate.requested(stamp, FRAME, MOVE, stamp, 11.14)
    assert not gate.final(stamp, FRAME, MOVE, stamp, 11.14)
    gate.goals([('owned', 2_120_000_000, 5)], stamp)
    assert not gate.active_goals and not gate.allowed_goals
    gate.goals([('external', 2_130_000_000, 2)], stamp)
    assert gate.state == 'FAULT_LATCHED' and gate.cancel_required and not gate.token


def test_manual_reverse_lock_invalid_axes_and_limit_values():
    gate = ready()
    release(gate)
    stamp = sample(gate, 2.14, 11.14)
    reverse = (-0.10, 0., 0., 0., 0., 0.)
    assert not gate.manual(gate.token, gate.epoch_ns, 2, True, stamp, FRAME, reverse, stamp, 11.14)
    assert gate.release_required and gate.output == (0.0, 0.0)
    gate = ready(reverse=True)
    release(gate)
    stamp = sample(gate, 2.14, 11.14)
    assert gate.manual(gate.token, gate.epoch_ns, 2, True, stamp, FRAME, reverse, stamp, 11.14)
    stamp = sample(gate, 2.16, 11.16, vx=-0.05)
    assert not gate.manual(gate.token, gate.epoch_ns, 3, True, stamp, FRAME, MOVE, stamp, 11.16)
    assert gate.release_required and gate.output == (0.0, 0.0)
    gate = ready()
    release(gate)
    stamp = sample(gate, 2.14, 11.14)
    assert not gate.manual(gate.token, gate.epoch_ns, 2, True, stamp, FRAME,
                           (math.nan, 0., 0., 0., 0., 0.), stamp, 11.14)
    assert not gate.sources
    assert gate.limit_values((0.2, 0., 0., 0., 0., 0.6)) == MOVE


def test_clock_rewind_discards_stop_history_and_allows_explicit_reactivation():
    gate = ready()
    old_token, old_epoch = gate.token, gate.epoch_ns
    assert gate.stopped(2_100_000_000, 11.1)
    assert gate.clock(1_900_000_000, 11.2)  # less than tf2's one-second threshold
    assert not gate.stop_samples and gate.stopped_since is None
    assert gate.mode == 'DISABLED' and not gate.token
    assert not gate.authorized(old_token, old_epoch, 1_900_000_000, 11.2)
    for index in range(12):
        stamp = sample(gate, 1.90 + index * .1, 11.2 + index * .1)
    assert gate.stopped(stamp, 12.3)
    assert gate.acquire('new-owner', stamp, 12.3)[0]
    assert gate.begin_transition('MANUAL')[0]
    assert gate.finish_transition(stamp, 12.3)[0]
    stamp = sample(gate, 3.02, 12.32)
    assert not gate.manual(gate.token, gate.epoch_ns, 1, False, stamp, FRAME, ZERO, stamp, 12.32)
    stamp = sample(gate, 3.04, 12.34)
    assert gate.manual(gate.token, gate.epoch_ns, 2, True, stamp, FRAME, MOVE, stamp, 12.34)
    assert gate.final(stamp, FRAME, MOVE, stamp, 12.34)
