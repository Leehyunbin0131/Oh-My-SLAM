"""Required command, fault/epoch and map/clock boundaries, without a drive sink."""

from omslam_navigation.guard_core import NavigationGate


FRAME = 'base_footprint'
MOVE = (0.20, 0.0, 0.0, 0.0, 0.0, 0.30)
ZERO = (0.0,) * 6


def running():
    gate = NavigationGate()
    gate.clock(1_000_000_000, 10.0)
    gate.health(True, '', 10.0)
    gate.clock(2_100_000_000, 11.1)
    assert gate.arm(2_100_000_000, 11.1)[0]
    gate.clock(2_120_000_000, 11.12)
    gate.goals([('first', 2_110_000_000, 2)], 2_120_000_000)
    assert gate.state == 'RUNNING'
    return gate


def test_original_stamp_and_timeout_chain():
    gate = running()
    # A downstream command before its source has no permission. It is never
    # relabelled with the current time or silently queued for a future goal.
    assert not gate.final(2_120_000_000, FRAME, MOVE, 2_120_000_000, 11.12)
    assert gate.requested(2_120_000_000, FRAME, MOVE, 2_120_000_000, 11.12)
    assert gate.final(2_120_000_000, FRAME, MOVE, 2_120_000_000, 11.12)
    issued = gate.sources[2_120_000_000].issued_wall
    gate.clock(2_300_000_000, 11.30)
    assert gate.requested(2_120_000_000, FRAME, MOVE, 2_300_000_000, 11.30)
    assert gate.sources[2_120_000_000].issued_wall == issued
    assert gate.final(2_120_000_000, FRAME, MOVE, 2_300_000_000, 11.30)
    # The installed smoother creates timeout zeros with a new stamp and empty
    # frame. They stop without faulting a normal BT wait or renewing intent.
    assert gate.final(2_300_000_000, '', ZERO, 2_300_000_000, 11.30)
    assert gate.output == (0.0, 0.0)
    assert gate.state == 'RUNNING'
    assert gate.sources[2_120_000_000].issued_wall == issued
    assert not gate.final(2_120_000_000, FRAME, MOVE, 2_300_000_000, 11.31)
    assert not gate.requested(2_500_000_000, FRAME, MOVE, 2_300_000_000, 11.31)
    gate.clock(2_310_000_000, 11.31)
    assert gate.requested(2_310_000_000, FRAME, MOVE, 2_310_000_000, 11.31)
    assert gate.final(2_310_000_000, FRAME, MOVE, 2_310_000_000, 11.31)
    # Simulation time may slow down; the monotonic lease still expires.
    gate.clock(2_350_000_000, 11.62)
    assert not gate.final(2_310_000_000, FRAME, MOVE, 2_350_000_000, 11.62)
    assert gate.tick(2_350_000_000, 11.62) == (0.0, 0.0)
    assert gate.state == 'FAULT_LATCHED'
    # An explicit controller stop followed by the BT's two-second Wait keeps
    # the action alive; it cannot keep the old nonzero lease alive.
    gate = running()
    assert gate.requested(2_120_000_000, FRAME, MOVE, 2_120_000_000, 11.12)
    assert gate.final(2_120_000_000, FRAME, MOVE, 2_120_000_000, 11.12)
    gate.clock(2_150_000_000, 11.15)
    assert gate.requested(2_150_000_000, FRAME, ZERO, 2_150_000_000, 11.15)
    assert gate.final(2_150_000_000, FRAME, ZERO, 2_150_000_000, 11.15)
    gate.clock(2_500_000_000, 11.50)
    assert gate.tick(2_500_000_000, 11.50) == (0.0, 0.0)
    # A smoother timeout can first emit nonzero deceleration remnants with
    # its generated empty-frame header. They have no source lease to forward.
    assert not gate.final(2_500_000_000, '', (0.03, 0., 0., 0., 0., 0.),
                          2_500_000_000, 11.50)
    assert gate.state == 'RUNNING' and gate.output == (0.0, 0.0)
    assert not gate.sources
    # The monitor's last timeout zero outlives the discarded source burst.
    assert gate.final(2_500_000_000, '', ZERO, 2_500_000_000, 11.50)
    gate.clock(4_200_000_000, 13.20)
    assert gate.tick(4_200_000_000, 13.20) == (0.0, 0.0)
    assert gate.state == 'RUNNING'
    assert not gate.final(2_120_000_000, FRAME, MOVE, 4_200_000_000, 13.20)
    assert gate.requested(4_200_000_000, FRAME, MOVE, 4_200_000_000, 13.20)
    assert gate.tick(4_200_000_000, 13.22) == (0.0, 0.0)
    assert gate.state == 'RUNNING'
    assert gate.final(4_200_000_000, FRAME, MOVE, 4_200_000_000, 13.24)


def test_fault_latch_cancel_and_new_epoch():
    gate = running()
    assert gate.requested(2_120_000_000, FRAME, MOVE, 2_120_000_000, 11.12)
    assert gate.final(2_120_000_000, FRAME, MOVE, 2_120_000_000, 11.12)
    gate.health(False, 'depth observation stale', 11.20)
    assert gate.state == 'FAULT_LATCHED' and gate.cancel_required
    assert gate.output == (0.0, 0.0)
    gate.health(True, '', 11.30)
    gate.clock(3_400_000_000, 12.40)
    assert not gate.arm(3_400_000_000, 12.40)[0]  # old goal is still executing
    assert not gate.requested(3_400_000_000, FRAME, MOVE, 3_400_000_000, 12.40)
    gate.goals([('first', 2_110_000_000, 5)], 3_400_000_000)
    assert not gate.cancel_required
    assert gate.arm(3_400_000_000, 12.40)[0]
    assert gate.state == 'ARMED_IDLE'
    assert not gate.requested(3_410_000_000, FRAME, MOVE, 3_410_000_000, 12.41)
    # A previously unseen UUID with an old acceptance timestamp is also stale.
    gate.goals([('stale_unseen', 3_000_000_000, 2)], 3_410_000_000)
    assert gate.state == 'FAULT_LATCHED'
    gate.goals([('stale_unseen', 3_000_000_000, 5)], 3_420_000_000)
    gate.clock(3_420_000_000, 12.42)
    assert gate.arm(3_420_000_000, 12.42)[0]
    gate.goals([('fresh', 3_430_000_000, 2)], 3_440_000_000)
    gate.clock(3_440_000_000, 12.44)
    assert gate.requested(3_440_000_000, FRAME, MOVE, 3_440_000_000, 12.44)
    assert gate.final(3_440_000_000, FRAME, MOVE, 3_440_000_000, 12.44)
    gate.goals([('fresh', 3_430_000_000, 2), ('replacement', 3_450_000_000, 1)], 3_450_000_000)
    assert gate.state == 'FAULT_LATCHED' and gate.cancel_required
    assert gate.output == (0.0, 0.0)
    gate = running()
    gate.goals([('first', 2_110_000_000, 4), ('replacement', 2_130_000_000, 2)], 2_140_000_000)
    assert gate.state == 'FAULT_LATCHED' and gate.cancel_required
    # Malformed axes cannot reach the final drive path.
    gate = running()
    lateral = (0.1, 0.1, 0.0, 0.0, 0.0, 0.0)
    assert not gate.requested(2_120_000_000, FRAME, lateral, 2_120_000_000, 11.12)
    assert gate.state == 'FAULT_LATCHED'
    gate = running()
    assert gate.requested(2_120_000_000, FRAME, MOVE, 2_120_000_000, 11.12)
    assert not gate.final(2_120_000_000, '', MOVE, 2_120_000_000, 11.12)
    assert gate.state == 'FAULT_LATCHED'


def test_map_correction_gate():
    gate = running()
    assert gate.map_transform(0.0, 0.0, 0.0)
    assert gate.map_transform(0.05, 0.0, 0.05)
    assert gate.state == 'RUNNING'
    assert not gate.map_transform(0.25, 0.0, 0.25)
    assert gate.state == 'FAULT_LATCHED' and gate.restart_required
    gate.goals([('first', 2_110_000_000, 5)], 2_130_000_000)
    assert not gate.arm(2_130_000_000, 11.13)[0]
    gate = running()
    assert gate.requested(2_120_000_000, FRAME, MOVE, 2_120_000_000, 11.12)
    assert gate.final(2_120_000_000, FRAME, MOVE, 2_120_000_000, 11.12)
    assert gate.clock(100_000_000, 11.13)
    assert gate.state == 'FAULT_LATCHED' and not gate.healthy
    assert gate.output == (0.0, 0.0) and not gate.sources
    assert not gate.requested(2_120_000_000, FRAME, MOVE, 100_000_000, 11.13)
    # Repeated paused /clock messages are not progress and never renew leases.
    gate = running()
    gate.clock(2_120_000_000, 11.50)
    gate.tick(2_120_000_000, 11.50)
    assert gate.state == 'FAULT_LATCHED'
