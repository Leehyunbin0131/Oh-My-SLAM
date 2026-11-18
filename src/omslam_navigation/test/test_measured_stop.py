import pytest
from nav_msgs.msg import Odometry

from omslam_navigation.measured_stop import MeasuredStop


def sample(stamp, vy=0.):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_footprint'
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
    msg.twist.twist.linear.y = vy
    return msg


def test_stop_needs_distinct_post_trigger_stamps_covering_the_interval():
    now = 10**9
    window = MeasuredStop(now)
    window.observe(sample(now - 1), .9)
    window.observe(sample(now + 1), 1.)
    for wall in (1.1, 1.2, 1.3):
        window.observe(sample(now + 1), wall)
    assert not window.confirmed(now + 300_000_001, 1.3)
    for i in (1, 2, 3):
        window.observe(sample(now + i * 100_000_000 + 1), 1. + i / 10)
    assert window.confirmed(now + 300_000_001, 1.3)
    assert not window.confirmed(now + 600_000_001, 1.3)
    assert not window.confirmed(now + 300_000_001, 2.)


@pytest.mark.parametrize('interruption', ['gap', 'sideways', 'out_of_order'])
def test_stop_interval_restarts_after_missing_or_moving_feedback(interruption):
    window = MeasuredStop(0)
    for ms in (100, 200, 300):
        window.observe(sample(ms * 10**6), ms / 1000.)
    if interruption == 'out_of_order':
        window.observe(sample(250 * 10**6), .35)
        end = 500
    elif interruption == 'sideways':
        window.observe(sample(350 * 10**6, .2), .35)
        end = 500
    else:
        end = 800
    window.observe(sample(end * 10**6), end / 1000.)
    assert not window.confirmed(end * 10**6, end / 1000.)
