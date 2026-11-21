#!/usr/bin/env bash
# Start the simulation, Nav2 and RViz for manually selected goals.
set -e
OMSLAM_NAV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$OMSLAM_NAV_ROOT/install/setup.bash" ]]; then
  echo "설치본이 없습니다. 먼저 omslam_navigation까지 빌드하세요." >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$OMSLAM_NAV_ROOT/install/setup.bash"
export OMSLAM_NAV_ROOT
export ROS_DOMAIN_ID="${OMSLAM_NAV_DOMAIN_ID:-86}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="${OMSLAM_GZ_PARTITION:-omslam_navigation_${UID}_${ROS_DOMAIN_ID}}"
exec /usr/bin/python3 - "$@" <<'PY'
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from omslam_navigation.measured_stop import MeasuredStop
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry

parser = argparse.ArgumentParser(description='Gazebo + Nav2 + RViz 수동 목표 주행')
parser.add_argument('--world', choices=('p4_course', 'airport'), default='p4_course')
args = parser.parse_args()
root = Path(os.environ['OMSLAM_NAV_ROOT'])
rviz = root / 'install/omslam_navigation/share/omslam_navigation/rviz/navigation.rviz'
if not rviz.is_file():
    sys.exit('P4 RViz 설치본이 없습니다. omslam_navigation을 빌드하세요.')
if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
    sys.exit('Gazebo/RViz를 표시할 데스크톱 터미널에서 실행하세요.')

lock = open(f'/tmp/omslam-nav-{os.getuid()}-{os.environ["ROS_DOMAIN_ID"]}.lock', 'w')
try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    sys.exit('같은 ROS domain에서 nav.sh가 이미 실행 중입니다.')

log_dir = root / 'reports/navigation' / datetime.now().strftime('%Y%m%d-%H%M%S-%f')
log_dir.mkdir(parents=True)
os.environ['ROS_LOG_DIR'] = str(log_dir / 'ros')
stopping = False
activation_requested = False
def request_stop(*_):
    global stopping
    stopping = True
signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('omslam_nav_session', parameter_overrides=[Parameter('use_sim_time', value=True)])
state = {}
state_at = 0.0
stop_window = None
def receive_state(message):
    global state, state_at
    state = json.loads(message.data)
    state_at = time.monotonic()
def receive_odom(message):
    if stop_window is not None:
        stop_window.observe(message, time.monotonic())
node.create_subscription(String, '/navigation/state', receive_state,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(Odometry, '/odom', receive_odom, 10)
enable = node.create_client(SetBool, '/navigation/enable')
processes = []
manifest = {'world': args.world, 'domain': os.environ['ROS_DOMAIN_ID'],
            'partition': os.environ['GZ_PARTITION'], 'processes': [], 'goals_sent': 0}

def record():
    (log_dir / 'session.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')

def pump(seconds=.1, cleanup=False):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if stopping and not cleanup:
            raise KeyboardInterrupt
        rclpy.spin_once(node, timeout_sec=min(.1, max(0., end-time.monotonic())))

def launch(label, command):
    if stopping:
        raise KeyboardInterrupt
    output = (log_dir / f'{label}.log').open('w')
    try:
        proc = subprocess.Popen(command, cwd='/tmp', stdout=output,
                                stderr=subprocess.STDOUT, start_new_session=True)
    except Exception:
        output.close()
        raise
    entry = {'label': label, 'pid': proc.pid, 'command': command}
    processes.append((proc, output, entry))
    manifest['processes'].append(entry)
    record()
    return proc

def set_enabled(value, cleanup=False):
    global activation_requested
    if not enable.wait_for_service(timeout_sec=2.):
        raise RuntimeError('/navigation/enable 서비스를 찾을 수 없습니다.')
    if stopping and value:
        raise KeyboardInterrupt
    activation_requested |= value
    future = enable.call_async(SetBool.Request(data=value))
    deadline = time.monotonic() + 5.
    while not future.done() and time.monotonic() < deadline:
        pump(cleanup=cleanup)
    if not future.done():
        raise RuntimeError('구동 활성화/해제 서비스 응답 시간 초과')
    response = future.result()
    if not response.success:
        raise RuntimeError(response.message)
    return response.message

def group_exists(pid):
    try:
        os.killpg(pid, 0)
        return True
    except ProcessLookupError:
        return False

exit_code = 0
try:
    # Allow DDS discovery before refusing a manually started, existing session.
    pump(2.)
    if (node.count_publishers('/clock') or node.count_publishers('/navigation/state')
            or node.count_publishers('/cmd_vel') or enable.service_is_ready()):
        raise RuntimeError('같은 ROS domain에 실행 중인 시뮬레이션/구동이 있습니다. 기존 실행을 종료한 뒤 다시 실행하세요.')
    print(f'Gazebo·Nav2 준비 중… 로그: {log_dir}', flush=True)
    nav = launch('navigation', ['ros2', 'launch', 'omslam_navigation', 'slam_navigation.launch.py',
        f'world:={args.world}', 'gui:=true', 'pedestrians:=false'])
    deadline = time.monotonic() + 60.
    stable_since = None
    while True:
        pump()
        if nav.poll() is not None:
            raise RuntimeError(f'Nav2 launch 종료: {nav.returncode}; navigation.log를 확인하세요.')
        if state.get('state') == 'FAULT_LATCHED':
            raise RuntimeError('준비 중 차단: ' + state.get('reason', ''))
        ready = (state.get('state') == 'READY' and state.get('healthy')
                 and time.monotonic() - state_at < 1.)
        stable_since = (stable_since or time.monotonic()) if ready else None
        if stable_since and time.monotonic() - stable_since >= 1.2:
            break
        if time.monotonic() >= deadline:
            raise RuntimeError('Nav2 준비 시간 초과: ' + json.dumps(state, ensure_ascii=False))
    set_enabled(True)
    manifest['enabled'] = True
    record()
    viewer = launch('rviz', ['rviz2', '-d', str(rviz), '--ros-args', '-p', 'use_sim_time:=true'])
    print('구동 활성화 완료. RViz의 Nav2 Goal로 위치와 도착 방향을 지정하세요.', flush=True)
    print('한 목표 도착·정지 후 다음 목표를 지정하세요. 종료: 이 터미널에서 Ctrl+C 또는 RViz 창 닫기.', flush=True)
    reported_fault = None
    while True:
        pump()
        if nav.poll() is not None:
            raise RuntimeError(f'Nav2 launch 종료: {nav.returncode}')
        if viewer.poll() is not None:
            if viewer.returncode != 0:
                raise RuntimeError(f'RViz 종료: {viewer.returncode}; rviz.log를 확인하세요.')
            break
        if state.get('state') == 'FAULT_LATCHED':
            signature = (state.get('reason', ''), state.get('restart_required', False))
            if signature != reported_fault:
                reported_fault = signature
                manifest['last_fault'] = dict(state)
                record()
                print('구동 차단: ' + signature[0], flush=True)
                if signature[1]:
                    print('지도·기억 재초기화가 필요합니다. Ctrl+C로 종료한 뒤 새 세션을 시작하세요.', flush=True)
                else:
                    print('원인 해소와 정지 확인 후 별도 터미널에서 명시적으로 재활성화하세요. 이전 목표는 재개하지 않습니다.', flush=True)
                    print('ROS_DOMAIN_ID=' + os.environ['ROS_DOMAIN_ID'] +
                          " ros2 service call /navigation/enable std_srvs/srv/SetBool '{data: true}'", flush=True)
        else:
            reported_fault = None
except KeyboardInterrupt:
    pass
except Exception as exc:
    exit_code = 1
    manifest['error'] = str(exc)
    print(f'오류: {exc}', file=sys.stderr, flush=True)
finally:
    # Disable while our navigation server is still alive, before stopping any child.
    if processes:
        print('구동 해제 및 실행 정리 중…', flush=True)
        if activation_requested or state.get('base_state') == 3:
            try:
                stop_window = MeasuredStop(node.get_clock().now().nanoseconds)
                manifest['disable_response'] = set_enabled(False, cleanup=True)
                deadline = time.monotonic() + 3.
                while time.monotonic() < deadline:
                    pump(cleanup=True)
                    if stop_window.confirmed(node.get_clock().now().nanoseconds, time.monotonic()):
                        manifest.update(stopped=True, stop_samples=stop_window.count,
                                        first_stop_stamp_ns=stop_window.first_ns, last_stop_stamp_ns=stop_window.last_ns)
                        break
                if not manifest.get('stopped'):
                    raise RuntimeError('종료 전 측정 정지를 확인하지 못했습니다.')
            except Exception as exc:
                manifest['disable_error'] = str(exc)
                print(f'구동 해제 확인 실패: {exc}', file=sys.stderr, flush=True)
                exit_code = 1
        for proc, output, entry in reversed(processes):
            for sig, timeout in ((signal.SIGINT, 18.), (signal.SIGTERM, 3.), (signal.SIGKILL, 2.)):
                proc.poll()
                if not group_exists(proc.pid):
                    break
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    break
                end = time.monotonic() + timeout
                while time.monotonic() < end and group_exists(proc.pid):
                    proc.poll()
                    time.sleep(.1)
            entry['exit_code'] = proc.poll()
            entry['group_remaining'] = group_exists(proc.pid)
            if entry['group_remaining']:
                exit_code = 1
            output.close()
    manifest['status'] = 'STOPPED' if exit_code == 0 else 'ERROR'
    record()
    node.destroy_node()
    rclpy.shutdown()
    lock.close()
sys.exit(exit_code)
PY
