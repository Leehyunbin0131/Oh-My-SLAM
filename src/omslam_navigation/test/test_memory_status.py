"""Compact memory metadata retains the original safety checks."""
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from omm_perception_msgs.msg import MemoryStatus
from omslam_navigation.navigation_guard import NavigationGuard


@pytest.mark.parametrize('fault', ['', 'sensor', 'geometry', 'memory', 'frame', 'voxel', 'stamp'])
def test_compact_memory_status_preserves_safety_validation(fault):
    message = MemoryStatus()
    message.header.frame_id = 'odom'
    message.header.stamp = Time(sec=10, nanosec=100000000)
    message.observation_stamp = Time(sec=10)
    message.sensor_healthy = message.geometry_healthy = message.memory_healthy = True
    message.voxel_size = .05
    if fault in ('sensor', 'geometry', 'memory'):
        setattr(message, fault + '_healthy', False)
    elif fault == 'frame':
        message.header.frame_id = 'map'
    elif fault == 'voxel':
        message.voxel_size = float('nan')
    elif fault == 'stamp':
        message.observation_stamp = Time(sec=11)
    seen = []
    observed = []
    guard = SimpleNamespace(p={'odom_frame': 'odom'}, memory_observation=None,
        remember=lambda *args: seen.append(args),
        perception_observed=lambda *args: observed.append(args))
    NavigationGuard.on_memory(guard, message)
    assert seen == [('memory', 10100000000, not bool(fault))]
    assert observed == [('memory', 11000000000 if fault == 'stamp' else 10000000000, not bool(fault))]
