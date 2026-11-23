"""The installed DWB generator, goal checker and progress checker run together."""
import os
from pathlib import Path
import subprocess

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from omslam_navigation.launch_support import load_configuration


def test_installed_dwb_moves_and_arrives_at_floor_and_mid_rpm_and_rejects_stalls(tmp_path):
    config = load_configuration(Path(get_package_share_directory('omslam_navigation')))
    controller = config['controller_server']
    controller['ros__parameters']['use_sim_time'] = False
    params = tmp_path / 'controller.yaml'
    params.write_text(yaml.safe_dump({'controller_server': controller}))
    package = 'omslam_navigation_contracts'
    probe = Path(get_package_prefix(package)) / 'lib' / package / 'operator_motion_probe'
    run = subprocess.run([str(probe), '--ros-args', '--params-file', str(params)], timeout=60,
                         env={**os.environ, 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'OFF',
                              'ROS_STATIC_PEERS': '', 'ROS_LOG_DIR': str(tmp_path / 'ros')},
                         text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    assert run.returncode == 0, run.stdout
    for rpm in (3, 10):
        for turn in (0, 1, -1):
            assert f'rpm={rpm} turn={turn} ' in run.stdout
        for arrival in ('inside', 'outside'):
            for sign in (1, -1):
                assert f'rpm={rpm} arrival={arrival} yaw_sign={sign} ' in run.stdout
    assert run.stdout.count('stall_at_10.1s=true') == 6, run.stdout
