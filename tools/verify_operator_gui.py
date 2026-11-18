#!/usr/bin/env python3
"""Exercise the installed Qt console and its owned ROS session once.

Input enters through ConsoleModel slots on the Qt thread. Ground truth and
command subscriptions are observers; they never supply control feedback.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime
from collections import deque

parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
parser.add_argument('--domain', type=int, default=187)
parser.add_argument('--timeout', type=float, default=900.)
parser.add_argument('--reuse-manual-result')
parser.add_argument('--continue-after-arrival')
parser.add_argument('--continue-at-maps')
parser.add_argument('--saved-map')
parser.add_argument('--hold-after-screenshots', action='store_true')
parser.add_argument('--skip-screenshots', action='store_true')
parser.add_argument('--scheduling-only', type=float, default=0.)
args = parser.parse_args()
ROOT = Path(__file__).resolve().parents[1]
OUT = (ROOT / args.output).resolve()
OUT.mkdir(parents=True, exist_ok=False)
os.environ['ROS_DOMAIN_ID'] = str(args.domain)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
PARTITION = f'operator_verification_{args.domain}_{time.time_ns()}'

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from omslam_operator_msgs.msg import ManualCommand, OwnerHeartbeat
from rclpy.qos import qos_profile_sensor_data
from PySide6.QtCore import QTimer, QUrl, QObject, QPointF
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine, QQmlExpression, qmlRegisterType
from PySide6.QtQuickControls2 import QQuickStyle
import omslam_operator.worker as worker_module
from omslam_operator.model import ConsoleModel
from omslam_operator.visuals import CameraProvider, RobotMap
from p4_geometry import compose, inverse, angle_delta


def json_text(value, **kwargs):
    def scalar(item):
        if isinstance(item, np.generic): return item.item()
        if isinstance(item, np.ndarray): return item.tolist()
        if isinstance(item, (bytes, bytearray)): return {'bytes_hex': item.hex()}
        raise TypeError(type(item).__name__)
    return json.dumps(value, default=scalar, **kwargs)


def stamp(msg):
    return msg.sec * 1_000_000_000 + msg.nanosec


class Evidence:
    def __init__(self):
        self.lock = threading.RLock()
        self.stage = 'startup'
        self.samples, self.events, self.checks = deque(maxlen=256), deque(maxlen=256), deque(maxlen=128)
        self.latest = {}
        self.log = (OUT / 'events.jsonl').open('w')
        self.raw_log = (OUT / 'gui_responses.jsonl').open('w')
        self.gui_latencies, self.pending_frames = deque(maxlen=128), deque(maxlen=32)
        self.ui_tick_stats = {'count': 0, 'max_duration': 0., 'max_normal_gap': 0., 'max_normal_gap_at': 0.}
        self.last_ui_tick = None
        self.report = {'started_at': datetime.now().astimezone().isoformat(),
                       'domain': args.domain, 'partition': PARTITION,
                       'checks': self.checks, 'screenshots': []}

    def event(self, event_name, **data):
        item = {'event': event_name, 'wall': time.monotonic(), 'stage': self.stage, **data}
        with self.lock:
            self.events.append(item)
            self.log.write(json_text(item, ensure_ascii=False) + '\n'); self.log.flush()
        print(json_text(item, ensure_ascii=False), flush=True)

    def check(self, name, good, **data):
        item = {'name': name, 'status': 'PASS' if good else 'FAIL', **data}
        self.checks.append(item)
        self.event('check', **item)
        if not good:
            raise AssertionError(name)

    def assign(self, key, value):
        if key not in ('operator', 'goal_result', 'control_response', 'map_response', 'speed_response', 'response'):
            return
        item = {'wall': time.monotonic(), 'stage': self.stage, 'key': key, 'value': copy.deepcopy(value)}
        with self.lock:
            self.raw_log.write(json_text(item, ensure_ascii=False) + '\n')


E = Evidence()


class ObservedConsoleNode(worker_module.ConsoleNode):
    def assign(self, key, value):
        super().assign(key, value)
        E.assign(key, value)


worker_module.ConsoleNode = ObservedConsoleNode


class ObservedModel(ConsoleModel):
    def tick(self):
        begin = time.monotonic()
        super().tick()
        metrics = E.ui_tick_stats
        metrics['count'] += 1
        metrics['max_duration'] = max(metrics['max_duration'], time.monotonic()-begin)
        if self.token and E.last_ui_tick is not None and E.stage != 'gui_freeze_begin':
            gap = begin-E.last_ui_tick
            if gap > metrics['max_normal_gap']:
                metrics['max_normal_gap'] = gap
                metrics['max_normal_gap_at'] = begin
        E.last_ui_tick = begin


def processes():
    result = []
    for path in Path('/proc').iterdir():
        if not path.name.isdecimal(): continue
        try:
            if f'GZ_PARTITION={PARTITION}'.encode() not in (path/'environ').read_bytes().split(b'\0'): continue
            fields = (path/'stat').read_text().rsplit(')', 1)[1].split()
            result.append({'pid': int(path.name), 'pgid': int(fields[2]), 'start_tick': fields[19],
                           'command': (path/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')})
        except (OSError, IndexError): pass
    return result


class Run:
    def __init__(self, app, model, shared, worker, window):
        self.app, self.model, self.shared, self.worker, self.window = app, model, shared, worker, window
        self.start = time.monotonic(); self.proc = None; self.owned = {}; self.pipe_buffer = b''
        self.done = False; self.failure = None; self.world_to_map = None
        self.close_requested = False; self.close_trigger_wall = None
        self.generator = self.scenario()
        self.timer = QTimer(); self.timer.setInterval(50); self.timer.timeout.connect(self.advance); self.timer.start()
        self.window.frameSwapped.connect(self.frame)

    def frame(self):
        now = time.monotonic()
        for name, sent in E.pending_frames: E.gui_latencies.append({'input': name, 'seconds': now-sent})
        E.pending_frames.clear()

    def data(self):
        with self.shared.lock: return copy.deepcopy(self.shared.data)

    def get(self, key):
        with self.shared.lock: return copy.deepcopy(self.shared.data.get(key, {}))

    def op(self): return self.get('operator')

    def do(self, name, fn):
        E.stage = name; E.event('input')
        E.pending_frames.append((name, time.monotonic())); fn()

    def wait(self, label, predicate, timeout=10.):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= end:
                raise TimeoutError(label + ': ' + json_text({'operator': self.op(), 'events': self.get('events')[:4]}, ensure_ascii=False))
            yield

    def delay(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end: yield

    def stopped(self):
        now = time.monotonic()
        return bool(self.op().get('stopped') and all(
            t in E.latest and now-E.latest[t]['wall'] < .25 and
            abs(E.latest[t]['v']) <= .01 and abs(E.latest[t]['w']) <= .02
            for t in ('/odom', '/wheel/odometry')))

    def moving(self): return abs(E.latest.get('/odom', {}).get('v', 0)) > .025

    def mode(self, mode):
        self.do('mode_'+mode, lambda: self.model.control(mode))
        yield from self.wait(mode+' activation', lambda: self.model.token
            and self.model.data.get('operator', {}).get('mode') == mode
            and self.model.data.get('operator', {}).get('phase') == 'ACTIVE'
            and self.model.epoch == self.model.data['operator']['epoch'])
        self.model.clearInput()
        yield from self.delay(.12)
        E.check(mode+'_activated', self.op().get('base_active'), operator=self.op())

    def stop_wait(self, label, fn):
        self.do(label, fn); trigger = time.monotonic()
        yield from self.wait(label+' measured stop', self.stopped, 3.)
        E.check(label, True, measured_stop_seconds=time.monotonic()-trigger,
                wheel=E.latest['/wheel/odometry'], odom=E.latest['/odom'])

    def screenshot(self, name, page, size=(1440, 900)):
        self.window.showNormal()
        self.window.setWidth(size[0]); self.window.setHeight(size[1])
        self.do('page_'+name, lambda: self.model.setPage(page))
        yield from self.delay(.6)
        target = OUT/(name+'.png')
        captured = self.window.grabWindow()
        dimensions = {'requested': list(size), 'logical': [self.window.width(), self.window.height()],
                      'pixels': [captured.width(), captured.height()],
                      'device_pixel_ratio': self.window.devicePixelRatio(), 'platform': self.app.platformName()}
        images = []
        for obj in self.window.findChildren(QObject):
            if not obj.inherits('QQuickImage'): continue
            source = obj.property('source')
            source = source.toString() if hasattr(source, 'toString') else str(source)
            if 'image://camera/' not in source: continue
            row = {'source': source, 'class': obj.metaObject().className()}
            for key in ('width', 'height', 'paintedWidth', 'paintedHeight', 'visible'):
                row[key] = obj.property(key)
            row['status'] = QQmlExpression(engine.rootContext(), obj, 'Number(status)').evaluate()[0]
            position = obj.mapToScene(QPointF(0, 0))
            row['scene_xy'] = [position.x(), position.y()]
            images.append(row)
        with self.shared.lock:
            provider = {key: {'size': [img.width(), img.height()], 'null': img.isNull()}
                        for key, img in self.shared.images.items()}
        E.check('screenshot_'+name, captured.save(str(target)) and dimensions['logical'] == list(size)
                and dimensions['pixels'] == list(size), dimensions=dimensions, images=images, provider=provider)
        (OUT/(name+'.json')).write_text(json_text(self.model.data, ensure_ascii=False, indent=2)+'\n')
        E.report['screenshots'].append({'path': str(target), 'dimensions': dimensions, 'page': page,
                                        'images': images, 'provider': provider})
        (OUT/'captures.json').write_text(json_text(E.report['screenshots'], ensure_ascii=False, indent=2)+'\n')

    def goal(self, x, y=0., yaw=0.):
        self.model.setGoal(float(x), float(y), float(yaw))
        E.check('goal_preview', self.model.goal['valid'], goal=dict(self.model.goal))
        self.do('navigate', self.model.navigate)
        yield from self.wait('goal accepted', lambda: self.shared.goal_active or self.get('goal_result'), 5.)

    def map_operation(self, operation, timeout=80.):
        old = self.get('map_response').get('_received', 0)
        self.do('map_'+operation, lambda: self.model.mapOperation(operation))
        yield from self.wait('map '+operation, lambda: self.get('map_response').get('_received', 0) > old, timeout)
        result = self.get('map_response')
        E.check('map_'+operation, result.get('success', False), response=result)

    def scenario(self):
        yield from self.wait('GUI ROS worker', lambda: self.shared.node is not None)
        yield from self.delay(1.)
        node = self.shared.node
        E.check('isolated_domain', not any(node.count_publishers(t) for t in ('/clock', '/navigation/state', '/operator/state')))
        env = os.environ.copy(); env['OMSLAM_CONSOLE_DOMAIN_ID'] = str(args.domain); env['OMSLAM_GZ_PARTITION'] = PARTITION
        self.output = (OUT/'session.log').open('w')
        self.proc = subprocess.Popen([str(ROOT/'console.sh'), '--start-sim', '--no-gui', '--world', 'p4_course'],
            cwd=ROOT, env=env, stdout=self.output, stderr=subprocess.STDOUT, start_new_session=True)
        E.report['session_pid'] = self.proc.pid
        self.model.setPage(1)  # Visit the visible camera page for live depth conversion.
        yield from self.wait('fresh initial data', lambda: self.op().get('auto_ready') and self.stopped()
            and self.get('pose') and 'depth' in self.get('cameras') and 'rgb' in self.get('cameras')
            and self.get('motor') and any(k.startswith('system_monitor/') for k in self.get('diagnostics')), 60.)
        self.world_to_map = compose(np.asarray(self.get('pose')), inverse(np.asarray(E.latest['/sim/ground_truth']['pose'])))
        E.check('observer_initially', not self.model.token and self.op()['mode'] == 'DISABLED' and not self.op()['base_active'])
        for p in processes(): self.owned[p['pid']] = p
        d = self.data()
        E.check('live_sensor_and_motor_data', d['cameras']['depth']['encoding'] in ('32FC1', '16UC1')
                and d['cameras']['rgb']['width'] > 0 and len(d['imu']['gyro']) == 3
                and 'encoder_rpm' in d['motor']['valid_fields']
                and not any(k in d['motor']['valid_fields'] for k in ('pwm', 'raw_counts', 'measured_rpm')),
                cameras=d['cameras'], imu=d['imu'], motor=d['motor'])
        if args.skip_screenshots and not args.scheduling_only:
            proof_path = ROOT/'reports/operator/verification-008/visual_qa.json'
            assert proof_path.is_file()
            E.report['screen_qa_reuse'] = {'proof': str(proof_path),
                'sha256': hashlib.sha256(proof_path.read_bytes()).hexdigest(),
                'reason': 'The four-page geometry/camera display is unchanged. The updated queue terminal state is captured after cancel in this run.'}
        if not args.skip_screenshots:
            yield from self.screenshot('driving-1440x900', 0)
            yield from self.screenshot('sensors-1280x800', 1, (1280, 800))
            yield from self.screenshot('motors-1440x900', 2)
            yield from self.screenshot('system-1280x800', 3, (1280, 800))
        if args.scheduling_only:
            self.model.setPage(0)
            for x in (1.4, 1.8): self.model.setGoal(x, 0., 0.); self.model.addGoal()
            self.model.acquireControl()
            yield from self.wait('diagnostic disabled ownership', lambda: self.model.token and self.op().get('owner_id') == self.model.owner)
            E.event('scheduling_begin', seconds=args.scheduling_only, operator=self.op())
            yield from self.delay(args.scheduling_only)
            E.check('diagnostic_no_movement', not self.op()['base_active'] and self.op()['mode'] == 'DISABLED' and self.stopped())
            E.event('scheduling_complete', operator=self.op())
            self.done = True; self.app.quit(); return
        if args.hold_after_screenshots:
            E.event('visual_qa_ready', captures=str(OUT/'captures.json'), continue_file=str(OUT/'continue'))
            yield from self.wait('visual QA release', lambda: (OUT/'continue').is_file(), 180.)
        if args.continue_at_maps:
            baseline_path = Path(args.continue_at_maps).resolve()
            baseline = json.loads(baseline_path.read_text())
            required = ('sequential_goals', 'auto_manual_cancels_and_stops',
                'cancel_goal', 'canceled_queue_does_not_advance', 'gui_freeze_stops_on_robot',
                'reconnect_starts_without_motion', 'operator_stop')
            assert all(any(c['name']==name and c['status']=='PASS' for c in baseline['checks']) for name in required)
            assert baseline['source_unchanged'] and not baseline['remaining_processes']
            current = E.report['build_source_sha256']
            changed_sources = {path: {'before': baseline['build_source_sha256'].get(path), 'after': current.get(path)}
                for path in set(baseline['build_source_sha256']) | set(current)
                if baseline['build_source_sha256'].get(path) != current.get(path)}
            assert set(changed_sources).issubset({
                'src/omslam_mapping/omslam_mapping/map_bundle.py',
                'src/omslam_operator_support/omslam_operator_support/session.py',
                'src/omslam_operator_support/test/test_map_operations.py'}), changed_sources
            E.report['reused_driving_result'] = {'report': str(baseline_path),
                'source_changes_since_driving': changed_sources,
                'sha256': hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
                'original_source_sha256': baseline['build_source_sha256'],
                'reason': 'These controls passed with the current GUI/guard/input implementation. Only the map save/load path changes afterward; this run exercises the remaining map operations.',
                'checks': list(required)}
        else:
            if args.continue_after_arrival:
                proof_path = Path(args.continue_after_arrival).resolve()
                proof = json.loads(proof_path.read_text())
                assert proof['arrival_pass']
                current = E.report['build_source_sha256']
                reuse_changes = {path: {'before': proof['source_sha256'].get(path), 'after': current.get(path)}
                    for path in set(proof['source_sha256']) | set(current)
                    if '.pytest_cache' not in Path(path).parts and proof['source_sha256'].get(path) != current.get(path)}
                allowed = {'src/omslam_operator/omslam_operator/worker.py',
                           'src/omslam_operator/test/test_ros_display_primitives.py',
                           'src/omslam_operator/omslam_operator/model.py',
                           'src/omslam_operator/qml/Main.qml',
                           'src/omslam_operator/test/test_presentation.py',
                           'src/omslam_operator/test/test_queue_lifecycle.py'}
                assert set(reuse_changes).issubset(allowed), reuse_changes
                original = json.loads((proof_path.parent/'results.json').read_text())
                assert any(c['name']=='manual_release' and c['status']=='PASS' for c in original['checks'])
                manual_path = Path(args.reuse_manual_result).resolve()
                manual = json.loads(manual_path.read_text())
                required = ('focus_loss', 'focus_return_does_not_resume', 'manual_speed_enforced')
                assert all(any(c['name']==name and c['status']=='PASS' for c in manual['checks']) for name in required)
                E.report['reused_results'] = {'arrival_and_current_manual': str(proof_path),
                    'arrival_proof_sha256': hashlib.sha256(proof_path.read_bytes()).hexdigest(),
                    'source_changes_since_arrival': reuse_changes,
                    'manual_focus_speed': str(manual_path), 'manual_focus_speed_sha256': hashlib.sha256(manual_path.read_bytes()).hexdigest(),
                    'reason': 'The single-goal arrival/manual behavior is retained. Subsequent changes normalize diagnostic/CameraInfo display scalars and track queue terminal rows through cancel/stop/mode changes, with edits guarded while a row is active. Navigation, ownership, and manual input paths are unchanged. Queue-specific behavior is exercised in this run; earlier arrival/manual evidence remains applicable.'}
                self.model.setPage(0)
                yield from self.mode('AUTO')
            else:
                self.model.setPage(0)
                yield from self.mode('MANUAL')
                self.model.setActive(True)
                self.do('manual_press', lambda: self.model.manual(1., 0., True))
                yield from self.wait('manual movement', self.moving)
                yield from self.delay(.7)
                yield from self.stop_wait('manual_release', lambda: self.model.manual(0., 0., False))
                if args.reuse_manual_result:
                    baseline_path = Path(args.reuse_manual_result).resolve()
                    baseline = json.loads(baseline_path.read_text())
                    required = ('focus_loss', 'focus_return_does_not_resume', 'manual_speed_enforced')
                    assert all(any(c['name'] == name and c['status'] == 'PASS' for c in baseline['checks']) for name in required)
                    E.report['reused_manual_checks'] = {'report': str(baseline_path),
                        'sha256': hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
                        'checks': list(required), 'original_source_sha256': baseline['build_source_sha256'],
                        'reason': 'Input-release/focus/speed paths unchanged; current presentation/liveness changes receive a fresh short manual move, release and mode transition above.'}
                    yield from self.mode('AUTO')
                else:
                    self.do('manual_focus_test', lambda: self.model.manual(1., 0., True))
                    yield from self.wait('manual focus movement', self.moving)
                    yield from self.stop_wait('focus_loss', lambda: self.model.setActive(False))
                    self.model.setActive(True); yield from self.delay(.4)
                    E.check('focus_return_does_not_resume', self.stopped() and not self.model.deadman)
                    self.do('manual_speed_limit', lambda: self.model.speed(.06, .2, 50.))
                    yield from self.wait('speed response', lambda: self.get('speed_response').get('success') and abs(self.op().get('linear_limit', 0)-.06) < 1e-6)
                    limit_begin = time.monotonic()
                    self.do('manual_limited_press', lambda: self.model.manual(1., 0., True))
                    yield from self.wait('limited movement', self.moving); yield from self.delay(.5)
                    yield from self.mode('AUTO')
                    limited = [p for p in E.samples if p['topic'] == '/cmd_vel' and p['wall'] >= limit_begin]
                    E.check('manual_speed_enforced', any(p['v'] > .02 for p in limited) and max(abs(p['v']) for p in limited) <= .060001)
                E.check('manual_auto_measured_boundary', self.stopped())
                yield from self.goal(1.)
                yield from self.wait('autonomous movement before camera view', self.moving)
                self.do('camera_while_navigating', lambda: self.model.setPage(1))
                yield from self.delay(.8)
                self.model.setPage(0)
                yield from self.wait('first autonomous arrival', lambda: not self.shared.goal_active and bool(self.get('goal_result')), 60.)
                yield from self.wait('arrival stopped observation', self.stopped, 2.)
                outcome = self.get('goal_result'); truth = compose(self.world_to_map, np.asarray(E.latest['/sim/ground_truth']['pose']))
                E.check('autonomous_arrival', outcome.get('success') and self.stopped()
                        and np.linalg.norm(truth[:2]-[1., 0.]) <= .15 and abs(angle_delta(truth[2], 0)) <= .20,
                        result=outcome, truth=truth.tolist())
            self.model.queue.clear()
            for x in (1.4, 1.8): self.model.setGoal(x, 0., 0.); self.model.addGoal()
            self.do('sequential_goals', self.model.runQueue)
            yield from self.wait('sequential terminal', lambda: not self.model.queueBusy, 60.)
            E.check('sequential_goals', len(self.model.goals) == 2 and all(g['status'] == '완료' for g in self.model.goals) and self.stopped(), goals=self.model.goals)
            yield from self.goal(2.8)
            yield from self.wait('goal before manual transition moving', self.moving)
            yield from self.mode('MANUAL')
            yield from self.wait('old action terminal after mode transition', lambda: not self.shared.goal_active)
            E.check('auto_manual_cancels_and_stops', self.stopped() and not self.op().get('goal_id'), result=self.get('goal_result'))
            yield from self.mode('AUTO')
            self.model.queue.clear()
            for x in (3., 3.4): self.model.setGoal(x, 0., 0.); self.model.addGoal()
            self.do('queue_to_cancel', self.model.runQueue)
            yield from self.wait('cancel goal moving', self.moving)
            yield from self.stop_wait('cancel_goal', self.model.cancel)
            yield from self.wait('canceled action terminal', lambda: not self.shared.goal_active)
            yield from self.wait('canceled queue terminal display', lambda: not self.model.queueBusy, 3.)
            E.check('canceled_queue_does_not_advance', not self.model.queueRunning
                    and self.model.goals[0]['status'] == '취소됨'
                    and self.model.goals[1]['status'] == '대기' and self.stopped(), goals=self.model.goals)
            yield from self.screenshot('queue-canceled-1440x900', 0)
            yield from self.goal(3.)
            yield from self.wait('goal before disconnect moving', self.moving)
            self.do('gui_freeze_begin', lambda: None)
            freeze_at = time.monotonic(); time.sleep(.7)
            E.pending_frames.clear()
            yield from self.wait('UI heartbeat expiry', lambda: self.op().get('mode') == 'DISABLED' and not self.op().get('owner_id') and self.stopped(), 3.)
            yield from self.wait('disconnected goal terminal', lambda: not self.shared.goal_active)
            E.check('gui_freeze_stops_on_robot', 'heartbeat expired' in self.op().get('reason', ''),
                    freeze_wall=freeze_at, operator=self.op(), odom=E.latest['/odom'])
            self.model.control('MANUAL')
            yield from self.wait('explicit reacquire after disconnect', lambda: self.model.token and self.op().get('mode') == 'MANUAL' and self.op().get('phase') == 'ACTIVE')
            E.check('reconnect_starts_without_motion', self.stopped() and not self.model.deadman)
            yield from self.stop_wait('operator_stop', self.model.stop)
            yield from self.wait('stop authority revoked', lambda: not self.model.token and not self.op().get('base_active'))
        self.model.setPage(0)
        self.model.acquireControl()
        yield from self.wait('map owner without activation', lambda: self.model.token and self.op().get('owner_id') == self.model.owner)
        E.check('acquire_only_disabled', self.op()['mode'] == 'DISABLED' and not self.op()['base_active'])
        yield from self.map_operation('list')
        if args.saved_map:
            yaml_path = Path(args.saved_map).resolve()
            assert yaml_path.suffix == '.yaml'
            prefix = yaml_path.with_suffix('')
            proof_path = yaml_path.parent.parent/'results.json'
            proof = json.loads(proof_path.read_text())
            assert proof['source_unchanged']
            assert all(any(c['name']==name and c['status']=='PASS' for c in proof['checks'])
                       for name in ('map_save', 'map_files_saved'))
            assert all(Path(str(prefix)+suffix).is_file() for suffix in
                       ('.yaml', '.pgm', '.posegraph', '.data', '.metadata.json'))
            E.report['reused_map_save'] = {'report': str(proof_path),
                'sha256': hashlib.sha256(proof_path.read_bytes()).hexdigest(),
                'original_source_sha256': proof['build_source_sha256'], 'map_yaml': str(yaml_path),
                'reason': 'The complete bundle was saved successfully in the prior run. Raster/posegraph save code is unchanged; only localization readiness is being repaired.'}
        else:
            prefix = OUT/'map'/'operator-map'; prefix.parent.mkdir()
            self.model.setMapPath(str(prefix))
            yield from self.map_operation('save')
            E.check('map_files_saved', all(Path(str(prefix)+suffix).is_file() for suffix in
                                         ('.yaml', '.pgm', '.posegraph', '.data', '.metadata.json')))
        initial = list(self.get('pose')); old_session = self.op()['session_id']
        self.model.acquireControl(); yield from self.wait('reacquire for map load', lambda: self.model.token and self.op().get('owner_id') == self.model.owner)
        self.model.setMapPath(str(prefix)+'.yaml')
        yield from self.map_operation('localize')
        yield from self.wait('GUI observes new localization session', lambda: self.op().get('session_id') != old_session
            and self.model.guard_session == self.op().get('session_id') and not self.op().get('base_active'), 10.)
        E.check('localize_load_without_goal_as_pose', not self.op()['auto_ready'] and not self.op()['base_active'], operator=self.op())
        self.model.acquireControl(); yield from self.wait('initialpose owner', lambda: self.model.token and self.op().get('owner_id') == self.model.owner)
        self.model.setGoal(*initial)
        yield from self.map_operation('initialpose')
        yield from self.wait('AMCL ready after initial pose', lambda: self.op().get('auto_ready'), 30.)
        E.check('initialpose_restores_auto_readiness', self.op()['auto_ready'] and not self.op()['base_active'])
        self.model.acquireControl(); yield from self.wait('SLAM switch owner', lambda: self.model.token and self.op().get('owner_id') == self.model.owner)
        amcl_session = self.op()['session_id']
        yield from self.map_operation('slam')
        yield from self.wait('GUI observes restored SLAM session', lambda: self.op().get('session_id') != amcl_session
            and self.model.guard_session == self.op().get('session_id') and self.op().get('auto_ready')
            and not self.op().get('base_active'), 10.)
        E.check('slam_mode_restored', True, operator=self.op())
        yield from self.mode('MANUAL')
        self.model.setActive(True)
        self.do('manual_before_close', lambda: self.model.manual(.5, 0., True))
        yield from self.wait('moving before actual window close', self.moving)
        yield from self.delay(.2)
        self.close_requested = True; self.close_trigger_wall = time.monotonic()
        self.do('window_close', self.window.close)
        yield  # The actual Qt close ends app.exec; external observation continues.

    def read_observer(self):
        if observer_proc.poll() is not None: raise RuntimeError('evidence observer exited: '+str(observer_proc.returncode))
        while True:
            try: chunk = os.read(observer_read, 65536)
            except BlockingIOError: break
            if not chunk: break
            self.pipe_buffer += chunk
        lines = self.pipe_buffer.split(b'\n'); self.pipe_buffer = lines.pop()
        if lines:
            latest = json.loads(lines[-1]); E.latest.update(latest)
            for row in latest.values(): E.samples.append(row)

    def advance(self):
        try:
            self.read_observer()
            if time.monotonic()-self.start > args.timeout: raise TimeoutError('overall GUI verification timeout')
            if self.proc is not None and self.proc.poll() is not None: raise RuntimeError('owned console session exited: '+str(self.proc.returncode))
            next(self.generator)
        except StopIteration: self.done = True; self.app.quit()
        except Exception as exc:
            self.failure = str(exc); E.report['error'] = str(exc); E.report['traceback'] = traceback.format_exc()
            E.report['failure_state'] = {'operator': self.op(), 'topics': self.get('topics'), 'layers': self.get('layers')}
            E.event('failure', error=str(exc)); self.model.stop(); self.timer.stop(); QTimer.singleShot(600, self.app.quit)


build = json.loads((ROOT/'reports/operator/latest_build.json').read_text())
changed = [p for p, h in build['source_sha256'].items() if not (ROOT/p).is_file() or hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != h]
E.check('latest_built_sources', build['exit_code'] == 0 and build['source_unchanged'] and not changed, changed=changed)
packages = ('omslam_operator', 'omslam_operator_msgs', 'omslam_operator_support', 'omslam_navigation', 'omslam_base', 'omslam_sim', 'omslam_system_monitor')
prefixes = {p: get_package_prefix(p) for p in packages}
E.check('installed_prefixes', all(Path(v).is_relative_to(ROOT/'install') for v in prefixes.values()), prefixes=prefixes)
E.report['build_manifest'] = str(ROOT/'reports/operator/latest_build.json')
E.report['build_source_sha256'] = build['source_sha256']
rclpy.init(args=[])
QQuickStyle.setStyle('Basic')
app = QGuiApplication([sys.argv[0]])
observer_read, observer_write = os.pipe()
os.set_blocking(observer_read, False)
observer_env = os.environ.copy(); observer_env['GZ_PARTITION'] = PARTITION
observer_output = (OUT/'observer.log').open('w')
observer_proc = subprocess.Popen([sys.executable, str(ROOT/'tools/operator_gui_observer.py'),
    '--output', str(OUT/'raw_observer.jsonl'), '--pipe-fd', str(observer_write)],
    env=observer_env, pass_fds=(observer_write,), stdout=observer_output, stderr=subprocess.STDOUT,
    start_new_session=True)
os.close(observer_write)
E.report['observer_pid'] = observer_proc.pid
shared = worker_module.Shared(); worker = worker_module.RosWorker(shared, True); worker.start()
model = ObservedModel(shared)
qmlRegisterType(RobotMap, 'OhMyRobot', 1, 0, 'RobotMap')
engine = QQmlApplicationEngine(); engine.rootContext().setContextProperty('operatorModel', model)
engine.addImageProvider('camera', CameraProvider(shared))
engine.load(QUrl.fromLocalFile(str(Path(get_package_share_directory('omslam_operator'))/'qml/Main.qml')))
if not engine.rootObjects(): raise RuntimeError('installed QML failed to load')
run = Run(app, model, shared, worker, engine.rootObjects()[0])
signal.signal(signal.SIGINT, lambda *_: app.quit())
signal.signal(signal.SIGTERM, lambda *_: app.quit())
app.exec()
model.timer.stop(); model.shutdown(); time.sleep(.2); shared.stop.set(); worker.join(3.)
if rclpy.ok(): rclpy.shutdown()
if run.close_requested:
    end = time.monotonic()+3.
    observed = False
    while time.monotonic() < end:
        run.read_observer()
        now = time.monotonic(); state = E.latest.get('/operator/state', {})
        measured = all(topic in E.latest and now-E.latest[topic]['wall'] < .25
            and abs(E.latest[topic]['v']) <= .01 and abs(E.latest[topic]['w']) <= .02
            for topic in ('/odom', '/wheel/odometry'))
        observed = bool(state and now-state['wall'] < .25 and measured and state['stopped']
            and not state['base_active'] and not state['owner_id'] and state['mode'] == 'DISABLED')
        if observed: break
        time.sleep(.02)
    try:
        E.check('actual_window_close_stops_and_releases', observed,
                trigger_wall=run.close_trigger_wall, observed_wall=time.monotonic(),
                operator=state, wheel=E.latest.get('/wheel/odometry'), odom=E.latest.get('/odom'))
        run.done = True; E.event('scenario_complete')
    except AssertionError:
        run.failure = 'actual window close stop/release not confirmed'
        E.report['error'] = run.failure
if run.proc is not None:
    for p in processes(): run.owned[p['pid']] = p
    if run.proc.poll() is None: run.proc.send_signal(signal.SIGINT)
    try: run.proc.wait(timeout=40.)
    except subprocess.TimeoutExpired: E.report['cleanup_timeout'] = True
    E.report['session_exit_code'] = run.proc.poll()
    run.output.close()
if observer_proc.poll() is None: observer_proc.send_signal(signal.SIGINT)
try: observer_proc.wait(timeout=5.)
except subprocess.TimeoutExpired:
    observer_proc.terminate(); observer_proc.wait(timeout=5.)
E.report['observer_exit_code'] = observer_proc.returncode
os.close(observer_read); observer_output.close()
remaining = []
for p in run.owned.values():
    try:
        fields = Path(f'/proc/{p["pid"]}/stat').read_text().rsplit(')', 1)[1].split()
        if fields[19] == p['start_tick']: remaining.append(p)
    except (OSError, IndexError): pass
E.report['owned_processes'] = list(run.owned.values()); E.report['remaining_processes'] = remaining
E.report['partition_remaining'] = processes()
# Analyze the complete evidence only after UI/ROS cleanup. During control the
# recorder keeps bounded windows so it cannot grow the interpreter's GC graph.
E.raw_log.flush()
recorded = [json.loads(line) for file in ('raw_observer.jsonl', 'gui_responses.jsonl')
            for line in (OUT/file).open()]
recorded.sort(key=lambda row: row['wall'])
stages = sorted((event['wall'], event['stage']) for event in E.events)
stage_index = 0
for row in recorded:
    while stage_index+1 < len(stages) and stages[stage_index+1][0] <= row['wall']: stage_index += 1
    row.setdefault('stage', stages[stage_index][1] if stages else 'startup')
with (OUT/'observations.jsonl').open('w') as combined:
    for row in recorded: combined.write(json_text(row)+'\n')
sources = {}
for row in recorded:
    if row.get('topic') == '/cmd_vel/requested': sources.setdefault(row['stamp_ns'], row)
final = [row for row in recorded if row.get('topic') == '/cmd_vel'
         and (abs(row['v']) > 1e-8 or abs(row['w']) > 1e-8)]
lease_errors = [row for row in final if row['stamp_ns'] not in sources
                or row['wall']-sources[row['stamp_ns']]['wall'] > .33]
E.report['recorder'] = {'raw_jsonl_complete': True, 'ram_sample_limit': 256,
                        'ram_response_limit': 0, 'ram_event_limit': 256, 'ram_check_limit': 128, 'raw_observer': 'separate process',
                        'detailed_callback_profiling': False, 'ui_tick_metrics': 'streaming maxima only'}
E.report['commands'] = {'nonzero_samples': len(final), 'unmatched_or_expired': len(lease_errors),
    'max_linear': max((abs(p['v']) for p in final), default=0), 'max_angular': max((abs(p['w']) for p in final), default=0)}
E.report['frame_latencies'] = list(E.gui_latencies)
E.report['ui_tick_metrics'] = E.ui_tick_stats
E.report['diagnostic_only'] = bool(args.scheduling_only)
E.report['source_unchanged'] = not [p for p, h in build['source_sha256'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != h]
E.report['status'] = 'PASS' if run.done and not run.failure and not remaining and not E.report['partition_remaining'] and E.report.get('session_exit_code') == 0 and E.report['observer_exit_code'] == 0 and (final or args.scheduling_only or args.continue_at_maps) and not lease_errors and E.report['source_unchanged'] else 'FAIL'
E.report['checks'] = list(E.checks)
E.report['completed_at'] = datetime.now().astimezone().isoformat()
(OUT/'results.json').write_text(json_text(E.report, ensure_ascii=False, indent=2)+'\n')
E.raw_log.close(); E.log.close()
print(json_text({'status': E.report['status'], 'report': str(OUT/'results.json'), 'error': E.report.get('error'), 'remaining': remaining}, ensure_ascii=False), flush=True)
raise SystemExit(0 if E.report['status'] == 'PASS' else 1)
