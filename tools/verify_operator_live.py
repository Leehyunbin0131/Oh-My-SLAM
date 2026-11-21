#!/usr/bin/env python3
"""Exercise the installed Qt console and its owned ROS session once.

Input enters through real QML controls and LiveConsoleModel slots on the Qt thread. Ground truth and
command subscriptions are observers; they never supply control feedback.
"""
import argparse
import copy
import gc
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
parser.add_argument('--continue-at', choices=['all', 'controls', 'focus', 'remainder', 'logs_remaining', 'sequential', 'goals', 'maps', 'close'], default='all')
parser.add_argument('--reuse-report')
parser.add_argument('--diagnostics-only', type=float, default=0.)
parser.add_argument('--profile-seconds', type=float, default=0.)
parser.add_argument('--qml-only', action='store_true', help='verify installed Qt startup and cleanup without starting the simulator')
parser.add_argument('--saved-map')
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
from PySide6.QtCore import QTimer, QUrl, QObject, QPointF, Qt, qInstallMessageHandler
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine, QQmlExpression, qmlRegisterType
from PySide6.QtQuickControls2 import QQuickStyle
from PySide6.QtTest import QTest
import omslam_operator.worker as worker_module
import omslam_operator.live_model as live_module
import omslam_operator.app as app_module
import omslam_operator.visuals as visuals_module
from omslam_operator.live_model import LiveConsoleModel
from omslam_operator.app import close_session
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
        self.qt_messages = deque(maxlen=128)
        self.profile_active=False
        self.profile={key:{'count':0,'wall_total':0.,'cpu_total':0.,'wall_max':0.,'cpu_max':0.} for key in ('paint','refresh','gc0','gc1','gc2')}
        self.gc_start={}
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

def qt_message(kind, context, message):
    if kind.name in ('QtWarningMsg', 'QtCriticalMsg', 'QtFatalMsg'):
        E.qt_messages.append({'kind': kind.name, 'file': context.file, 'line': context.line, 'text': message})
        E.event('qt_message', kind=kind.name, text=message)
qInstallMessageHandler(qt_message)

def profile_finish(key, wall, cpu):
    stats=E.profile[key];wall=time.monotonic()-wall;cpu=time.thread_time()-cpu
    stats['count']+=1;stats['wall_total']+=wall;stats['cpu_total']+=cpu
    stats['wall_max']=max(stats['wall_max'],wall);stats['cpu_max']=max(stats['cpu_max'],cpu)

def profile_gc(phase, info):
    if not E.profile_active:return
    key='gc'+str(info['generation'])
    if phase=='start':E.gc_start[key]=(time.monotonic(),time.thread_time())
    elif key in E.gc_start:profile_finish(key,*E.gc_start.pop(key))
if args.diagnostics_only or args.profile_seconds:gc.callbacks.append(profile_gc)

class ObservedMap(RobotMap):
    def paint(self,painter):
        if not E.profile_active:return super().paint(painter)
        wall,cpu=time.monotonic(),time.thread_time()
        try:return super().paint(painter)
        finally:profile_finish('paint',wall,cpu)



class ObservedConsoleNode(worker_module.ConsoleNode):
    def assign(self, key, value):
        super().assign(key, value)
        E.assign(key, value)


worker_module.ConsoleNode = ObservedConsoleNode


class ObservedModel(LiveConsoleModel):
    def refresh(self):
        if not E.profile_active:return super().refresh()
        wall,cpu=time.monotonic(),time.thread_time()
        try:return super().refresh()
        finally:profile_finish('refresh',wall,cpu)

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

    def stop_wait(self, label, fn):
        self.do(label, fn); trigger = time.monotonic()
        yield from self.wait(label+' measured stop', self.stopped, 3.)
        E.check(label, True, measured_stop_seconds=time.monotonic()-trigger,
                wheel=E.latest['/wheel/odometry'], odom=E.latest['/odom'])

    def screenshot(self, name, page, size=(1440, 900)):
        self.window.setWindowState(Qt.WindowNoState)
        self.window.showNormal()
        self.window.resize(size[0],size[1])
        self.window.setPosition(40,40)
        yield from self.delay(.2)
        self.window.resize(size[0],size[1])
        self.do('page_'+name, lambda: self.window.setProperty('page', page))
        yield from self.delay(.6)
        target = OUT/(name+'.png')
        captured = self.window.grabWindow()
        dimensions = {'requested': list(size), 'logical': [self.window.width(), self.window.height()],
                      'pixels': [captured.width(), captured.height()],
                      'device_pixel_ratio': self.window.devicePixelRatio(), 'platform': self.app.platformName(),
                      'window_state': str(self.window.windowState())}
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
        if page == 0:
            E.check('movement_status_label_'+name,self.model.ui['movementState']!='ready' or self.item('movementStatusText').property('text')=='이동 가능')
            E.check('live_camera_images_'+name, len(images)>=2 and all(row['status']==1 and row['paintedWidth']>0 and row['paintedHeight']>0 for row in images), images=images)
        E.check('screenshot_'+name, captured.save(str(target)) and dimensions['logical'] == list(size)
                and dimensions['pixels'] == [round(value*dimensions['device_pixel_ratio']) for value in size], dimensions=dimensions, images=images, provider=provider)
        (OUT/(name+'.json')).write_text(json_text({'ui': self.model.ui, 'goals': self.model.goals}, ensure_ascii=False, indent=2)+'\n')
        E.report['screenshots'].append({'path': str(target), 'dimensions': dimensions, 'page': page,
                                        'images': images, 'provider': provider})
        (OUT/'captures.json').write_text(json_text(E.report['screenshots'], ensure_ascii=False, indent=2)+'\n')

    def items(self):
        pending = [self.window.contentItem()]
        while pending:
            item = pending.pop()
            yield item
            pending.extend(item.childItems())

    def item(self, name):
        result = next((item for item in self.items() if item.objectName() == name), None)
        if result is None: raise AssertionError('Missing QML item ' + name)
        return result

    def click(self, name):
        item = self.item(name)
        if not item.isVisible() or not item.isEnabled(): raise AssertionError('QML item not interactive: '+name)
        point = item.mapToScene(QPointF(item.width()/2, item.height()/2)).toPoint()
        QTest.mouseClick(self.window, Qt.LeftButton, Qt.NoModifier, point)

    def press_pad(self):
        item = self.item('joystickArea')
        point = item.mapToScene(QPointF(item.width()/2, item.height()/2-30)).toPoint()
        QTest.mousePress(self.window, Qt.LeftButton, Qt.NoModifier, point)
        self.pad_point = point

    def release_pad(self):
        QTest.mouseRelease(self.window, Qt.LeftButton, Qt.NoModifier, self.pad_point)

    def fill(self, name, value, submit=True):
        field = self.item(name)
        field.forceActiveFocus()
        QTest.keyClick(self.window, Qt.Key_A, Qt.ControlModifier)
        field.setProperty('text', str(value))
        if submit:
            QTest.keyClick(self.window, Qt.Key_Return)

    def add_goal(self, x, y=0., yaw=0.):
        # The online map boundary varies with spawn and sensor timing. Preserve
        # the requested heading and pick the closest fully observed test pose.
        from omslam_navigation.keepouts import grid_goal_error
        grid = self.shared.map_layers['map']
        request = (x,y)
        candidates = sorted(((i*.1,j*.1) for i in range(-10,11) for j in range(-10,11)),
                            key=lambda d:d[0]*d[0]+d[1]*d[1])
        prior = self.model.goals
        for dx,dy in candidates:
            px,py = x+dx,y+dy
            if any(math.hypot(px-g['x'],py-g['y']) < .18 for g in prior):
                continue
            if all(not grid_goal_error(px+ex,py+ey,yaw,grid['origin'],grid['resolution'],
                       grid['width'],grid['height'],grid['cells'].ravel())
                   for ex,ey in ((0,0),(.05,0),(-.05,0),(0,.05),(0,-.05))):
                x,y = px,py
                break
        else:
            raise AssertionError('No observed footprint near GUI test waypoint '+str(request))
        E.report.setdefault('waypoint_fixtures',[]).append(dict(request=request,selected=(x,y),yaw=yaw))
        before = len(self.model.goals)
        yield from self.wait('add goal button enabled',lambda:self.item('addGoalButton').isEnabled(),5.)
        self.do('goal_add_ui', lambda: self.click('addGoalButton'))
        yield from self.delay(.12)
        native = self.item('nativeMap')
        point = native.screenPoint(x, y)
        if isinstance(point, dict): point = QPointF(point['x'], point['y'])
        point = native.mapToScene(point).toPoint()
        QTest.mouseClick(self.window, Qt.LeftButton, Qt.NoModifier, point)
        yield from self.wait('QML adds waypoint', lambda: len(self.model.goals) == before+1, 3.)
        # Edit through the visible fields, also verifying exact coordinates/yaw.
        self.click('goalCoordinates'+str(before))
        yield from self.delay(.12)
        self.fill('editGoalX', x); self.fill('editGoalY', y); self.fill('editGoalYaw', yaw*180/math.pi)
        QTest.keyClick(self.window, Qt.Key_Escape)
        yield from self.delay(.12)
        goal = self.model.goals[before]
        E.check('goal_ui_'+str(before), abs(goal['x']-x)<.015 and abs(goal['y']-y)<.015
                and abs(goal['yaw']-yaw)<.015, goal=goal)

    def remove_all_goals(self):
        while self.model.goals:
            yield from self.wait('delete goal button enabled',lambda:self.item('deleteGoal0').isEnabled(),5.)
            self.click('deleteGoal0')
            yield from self.delay(.12)

    def start_route(self):
        yield from self.wait('move button enabled',lambda:self.item('moveButton').isEnabled(),5.)
        self.do('route_start_ui', lambda: self.click('moveButton'))
        yield from self.wait('route accepted', lambda: self.model.queueBusy or self.shared.goal_active, 12.)

    def map_operation(self, operation, action, timeout=80.):
        old = self.get('map_response').get('_received', 0)
        yield from self.wait('map controls enabled',lambda:bool(self.item('workspace').property('editable')),5.)
        self.do('map_'+operation, action)
        yield from self.wait('map '+operation, lambda: self.get('map_response').get('_received', 0) > old, timeout)
        result = self.get('map_response')
        E.check('map_'+operation, result.get('success', False), response=result)
        yield from self.wait('map response reflected in UI',lambda:self.model._map_inflight is None and not self.model.ui.get('operationPending'),3.)

    def map_path(self, button, path):
        self.click('mapToolsButton'); self.click(button)
        submit_with_enter = button == 'mapSaveButton'
        self.fill('mapPathInput', path, submit=submit_with_enter)
        if not submit_with_enter:
            self.click('mapPathSubmit')

    def view_controls(self):
        self.do('camera_rgb_expand',lambda:self.click('cameraDockRgb'))
        yield from self.delay(.2)
        E.check('rgb_expand_live',self.item('workspace').property('cameraKind')=='rgb'
                and self.item('expandedCamera').isVisible() and self.model.ui['rgbFresh'])
        self.click('closeCameraButton')
        self.do('camera_depth_expand',lambda:self.click('cameraDockDepth'))
        yield from self.delay(.2)
        E.check('depth_expand_live',self.item('workspace').property('cameraKind')=='depth'
                and self.item('expandedCamera').isVisible() and self.model.ui['depthFresh'])
        self.click('closeCameraButton');self.click('mapToolsButton')
        checkbox=next(item for item in self.items() if item.isVisible() and item.property('text')=='라이다')
        point=checkbox.mapToScene(QPointF(checkbox.width()/2,checkbox.height()/2)).toPoint()
        QTest.mouseClick(self.window,Qt.LeftButton,Qt.NoModifier,point)
        E.check('scan_layer_ui',not self.item('nativeMap').layers['scan'])
        QTest.mouseClick(self.window,Qt.LeftButton,Qt.NoModifier,point)
        QTest.keyClick(self.window,Qt.Key_Escape)
        self.click('mapFitButton')
        self.click('logsButton');yield from self.delay(.25)
        self.fill('logSearch','omslam_no_such_log_2541')
        yield from self.delay(.15)
        E.check('log_search_filters_live_rows',bool(self.model.ui['logs']) and not any(item.isVisible()
                for item in self.items() if item.objectName()=='logRow'))
        self.fill('logSearch','');self.click('logsButton')
        yield from self.delay(.15)

    def visibility_and_logs(self):
        self.do('hide_while_moving',self.press_pad)
        yield from self.wait('movement before hide',self.moving,12.)
        self.do('actual_window_hide',self.window.hide);begin=time.monotonic()
        yield from self.wait('hidden window releases input and stops',self.stopped,3.)
        E.check('hidden_window_stops',not self.model.deadman,measured_stop_seconds=time.monotonic()-begin)
        self.release_pad();self.window.show();self.window.requestActivate()
        yield from self.delay(.6)
        E.check('show_does_not_resume',self.stopped() and not self.model.deadman)
        self.click('stopButton')
        yield from self.wait('control released before static captures',lambda:not self.model.token and not self.op().get('owner_id') and self.stopped(),5.)
        for size in ((1280,800),(1440,900)):
            yield from self.screenshot('logs-final-'+str(size[0]),1,size)
            rows=[item for item in self.items() if item.objectName()=='logRow' and item.isVisible()]
            E.check('logs_collapsed_height_'+str(size[0]),bool(rows) and all(row.height()<=100 for row in rows),heights=[row.height() for row in rows])
            self.window.setProperty('page',0);yield from self.delay(.15)
            native=self.item('nativeMap');point=native.screenPoint(.7,.3);world=native.worldPoint(point.x(),point.y())
            E.check('viewport_roundtrip_'+str(size[0]),abs(world.x()-.7)<1e-9 and abs(world.y()-.3)<1e-9,
                    screen=[point.x(),point.y()],world=[world.x(),world.y()],size=list(size))
        self.window.setProperty('page',1);yield from self.delay(.2)
        row=next(item for item in self.items() if item.objectName()=='logRow' and item.isVisible() and str(item.property('message')).startswith('[상세]'))
        collapsed=row.height();text=row.property('message')
        point=row.mapToScene(QPointF(row.width()/2,12)).toPoint()
        QTest.mouseClick(self.window,Qt.LeftButton,Qt.NoModifier,point);yield from self.delay(.2)
        E.check('logs_expand_retains_full_text',self.item('logsPage').property('expandedMessage')==text and row.height()>collapsed*2,
                collapsed_height=collapsed,expanded_height=row.height(),text_length=len(text))
        yield from self.screenshot('logs-expanded-1440',1,(1440,900))
        self.item('logsPage').setProperty('expandedMessage','')
        self.fill('logSearch','PWM');yield from self.delay(.15)
        E.check('logs_full_text_search',all('pwm' in str(item.property('message')).lower() for item in self.items() if item.objectName()=='logRow' and item.isVisible()))
        self.fill('logSearch','');self.window.setProperty('page',0);yield from self.delay(.2)

    def remaining_logs(self):
        self.window.setProperty('page',1);yield from self.delay(.4)
        self.fill('logSearch','freshTopics');self.item('logList').setProperty('contentY',0)
        yield from self.delay(.3)
        view=self.item('logList');top=view.mapToScene(QPointF(0,0)).y();bottom=top+view.height()
        rows=[item for item in self.items() if item.objectName()=='logRow' and item.isVisible()
            and top+1<=item.mapToScene(QPointF(12,12)).y()<=min(bottom,self.window.height())-1]
        row=max(rows,key=lambda item:len(str(item.property('message'))))
        text=str(row.property('message'));collapsed=row.height()
        E.check('logs_full_text_search','freshTopics' in text and text.index('freshTopics')>300,
                keyword_offset=text.index('freshTopics'),text_length=len(text),matched_visible_rows=len(rows))
        point=row.mapToScene(QPointF(row.width()/2,12)).toPoint()
        E.check('log_click_inside_viewport',top<=point.y()<bottom and 0<=point.x()<self.window.width(),point=[point.x(),point.y()],viewport=[top,bottom])
        QTest.mouseClick(self.window,Qt.LeftButton,Qt.NoModifier,point);yield from self.delay(.2)
        E.check('logs_expand_retains_full_text',self.item('logsPage').property('expandedMessage')==text and row.height()>collapsed*2,
                collapsed_height=collapsed,expanded_height=row.height(),text_length=len(text))
        yield from self.screenshot('logs-expanded-1440',1,(1440,900))
        self.item('logList').setProperty('contentY',0);yield from self.delay(.15)
        row=next(item for item in self.items() if item.objectName()=='logRow' and item.property('expanded'))
        self.item('logList').setProperty('contentY',max(0,row.y()));yield from self.delay(.15)
        point=row.mapToScene(QPointF(row.width()/2,12)).toPoint()
        QTest.mouseClick(self.window,Qt.LeftButton,Qt.NoModifier,point);yield from self.delay(.2)
        E.check('logs_recollapse',self.item('logsPage').property('expandedMessage')=='')
        self.fill('logSearch','');self.window.setProperty('page',0);yield from self.delay(.2)

    def initial_pose_ui(self, pose):
        self.click('mapToolsButton');self.click('initialPoseButton')
        native=self.item('nativeMap');p=native.screenPoint(pose[0],pose[1])
        start=native.mapToScene(p).toPoint()
        end=native.mapToScene(QPointF(p.x()+30*math.cos(pose[2]),p.y()-30*math.sin(pose[2]))).toPoint()
        QTest.mousePress(self.window,Qt.LeftButton,Qt.NoModifier,start)
        QTest.mouseMove(self.window,end)
        QTest.mouseRelease(self.window,Qt.LeftButton,Qt.NoModifier,end)

    def new_map_ui(self):
        self.click('mapToolsButton');self.click('newMapButton')

    def scenario(self):
        yield from self.wait('GUI ROS worker', lambda: self.shared.node is not None)
        yield from self.delay(1.)
        node = self.shared.node
        E.check('isolated_domain', not any(node.count_publishers(t) for t in ('/clock', '/navigation/state', '/operator/state')))
        E.check('disconnected_display', self.model.ui['movementState']=='not_connected'
                and all(self.model.ui.get(key)=='—' for key in ('leftRpm','rightRpm','roll','pitch','heading','hostCpu','robotCpu')), ui=self.model.ui)
        env = os.environ.copy(); env['OMSLAM_CONSOLE_DOMAIN_ID'] = str(args.domain); env['OMSLAM_GZ_PARTITION'] = PARTITION
        self.output = (OUT/'session.log').open('w')
        self.proc = subprocess.Popen([str(ROOT/'console.sh'), '--start-sim', '--no-gui', '--world', 'p4_course'],
            cwd=ROOT, env=env, stdout=self.output, stderr=subprocess.STDOUT, start_new_session=True)
        E.report['session_pid'] = self.proc.pid
        yield from self.wait('fresh live data', lambda: self.op().get('auto_ready') and self.stopped()
            and self.get('pose') and 'depth' in self.get('cameras') and 'rgb' in self.get('cameras')
            and self.get('motor') and self.get('scan')
            and any(k.startswith('system_monitor/') for k in self.get('diagnostics')), 80.)
        self.world_to_map = compose(np.asarray(self.get('pose')), inverse(np.asarray(E.latest['/sim/ground_truth']['pose'])))
        E.check('observer_initially', not self.model.token and self.op()['mode']=='DISABLED' and not self.op()['base_active'])
        for p in processes(): self.owned[p['pid']] = p
        d = self.data()
        with self.shared.lock:
            maps = {key: {'width': value['width'], 'height':value['height']} for key,value in self.shared.map_layers.items()}
        E.check('live_sensor_motor_map', d['cameras']['depth']['encoding'] in ('32FC1','16UC1')
            and d['cameras']['rgb']['width']>0 and len(d['imu']['gyro'])==3
            and 'encoder_rpm' in d['motor']['valid_fields']
            and not any(key in d['motor']['valid_fields'] for key in ('pwm','raw_counts','measured_rpm'))
            and bool(maps), cameras=d['cameras'], imu=d['imu'], motor=d['motor'], maps=maps)
        self.baseline_counts={topic:d['topics'][topic]['count'] for topic in ('/imu/data','/base/motor_state')}
        E.check('live_history_disabled',not self.shared.history_enabled and len(d['history'])==0,history_length=len(d['history']),trace_length=len(d['trace']))
        del d
        E.check('unavailable_robot_metrics', all(self.model.ui[key]=='—' for key in ('robotCpu','robotMemory','robotTemperature')), ui=self.model.ui)
        if args.diagnostics_only or args.profile_seconds:
            duration=args.diagnostics_only or args.profile_seconds
            self.model.acquireControl()
            yield from self.wait('diagnostic disabled ownership',lambda:self.model.token and self.op().get('owner_id')==self.model.owner)
            E.stage='diagnostic_idle';E.profile_active=True
            E.ui_tick_stats={'count':0,'max_duration':0.,'max_normal_gap':0.,'max_normal_gap_at':0.}
            E.event('diagnostic_begin',seconds=duration)
            yield from self.delay(duration)
            E.profile_active=False
            E.report['bounded_profile']=E.profile
            E.check('diagnostic_disabled_no_motion',self.stopped() and self.op()['mode']=='DISABLED' and not self.op()['base_active'])
            E.event('diagnostic_complete',profile=E.profile,ui_ticks=E.ui_tick_stats)
            if args.diagnostics_only:
                self.done=True;self.app.quit();return
            self.model.stop()
            yield from self.wait('profile control released',lambda:not self.model.token and not self.op().get('owner_id') and self.stopped(),5.)
        if args.continue_at=='all':
            for size in ((1280,800),(1440,900)):
                yield from self.screenshot('map-'+str(size[0]), 0, size)
                yield from self.screenshot('logs-'+str(size[0]), 1, size)
        self.window.setProperty('page',0)
        self.window.requestActivate()
        yield from self.delay(.2)
        if args.continue_at=='all':
            yield from self.view_controls()
        if args.continue_at in ('all','controls','focus'):
            if args.continue_at=='all':
                self.do('manual_direct_pad', self.press_pad)
                yield from self.wait('automatic manual acquisition and motion', lambda: self.op().get('mode')=='MANUAL' and self.moving(), 12.)
                yield from self.delay(1.5)
                E.check('manual_auto_acquired', self.op()['phase']=='ACTIVE' and self.model.token and self.model.deadman, operator=self.op())
                yield from self.stop_wait('manual_release', self.release_pad)
            if args.continue_at in ('all','controls'):
                maximum_rpm = float(self.op()['max_wheel_rpm'])
                requested_rpm = round(maximum_rpm * .25, 1)
                prior_rpm = self.op()['wheel_rpm_limit']
                self.do('fractional_rpm_input', lambda:self.fill('rpmLimitInput', requested_rpm))
                target = float(self.op()['max_linear_velocity']) * requested_rpm / maximum_rpm
                yield from self.wait('speed limit response',lambda:abs(self.op().get('wheel_rpm_limit',0)-requested_rpm)<1e-6 and abs(self.op().get('linear_limit',0)-target)<1e-6,5.)
                self.do('speed_limit_move',self.press_pad);limit_begin=time.monotonic()
                yield from self.wait('limited motion',self.moving,10.);yield from self.delay(.6)
                yield from self.stop_wait('speed_limit_release',self.release_pad)
                E.check('speed_limit_ui',self.get('speed_response').get('success') and abs(self.op()['linear_limit']-target)<1e-6,
                    target=target,begin=limit_begin,end=time.monotonic(),response=self.get('speed_response'))
                self.fill('rpmLimitInput', prior_rpm)
                yield from self.wait('speed restored',lambda:abs(self.op().get('wheel_rpm_limit',0)-prior_rpm)<1e-6,5.)
            self.do('focus_loss_start', self.press_pad)
            yield from self.wait('movement before focus loss', self.moving)
            # Move actual QML keyboard focus away from the pad. Offscreen hide has no native deactivation event.
            self.item('logsButton').forceActiveFocus()
            yield from self.wait('QML focus loss measured stop', self.stopped, 3.)
            self.release_pad(); self.item('directPad').forceActiveFocus()
            yield from self.delay(.6)
            E.check('focus_return_no_motion', self.stopped() and not self.model.deadman)
        if args.continue_at=='remainder':
            yield from self.visibility_and_logs()
        if args.continue_at=='logs_remaining':
            yield from self.remaining_logs()
        if args.continue_at in ('all','controls','focus','remainder','logs_remaining','goals'):
            yield from self.add_goal(1.)
            yield from self.add_goal(1.4)
            yield from self.add_goal(1.8)
            # Goal layout already captured; never synchronously grab a window while holding a control lease.
            original = [goal['uid'] for goal in self.model.goals]
            handle = self.item('reorderHandle2'); handle.forceActiveFocus()
            QTest.keyClick(self.window,Qt.Key_Up)
            yield from self.delay(.2)
            E.check('reorder_ui', [goal['uid'] for goal in self.model.goals]==[original[0],original[2],original[1]])
            self.click('deleteGoal1'); yield from self.delay(.15)
            self.click('deleteGoal1'); yield from self.delay(.15)
            E.check('delete_ui', len(self.model.goals)==1 and self.model.goals[0]['uid']==original[0])
            yield from self.start_route()
            yield from self.wait('single autonomous movement', self.moving, 15.)
            yield from self.wait('single arrival', lambda: not self.model.queueBusy and not self.shared.goal_active, 70.)
            yield from self.wait('single measured stop',self.stopped,3.)
            truth=compose(self.world_to_map,np.asarray(E.latest['/sim/ground_truth']['pose']))
            E.check('single_goal_arrival', self.model.goals[0]['status']=='완료' and np.linalg.norm(truth[:2]-[1.,0.])<=.15
                and abs(angle_delta(truth[2],0.))<=.20,
                goals=self.model.goals,truth=truth.tolist(),yaw_error=abs(angle_delta(truth[2],0.)))
        if args.continue_at in ('all','controls','focus','remainder','logs_remaining','sequential','goals'):
            yield from self.remove_all_goals()
            yield from self.add_goal(1.4);yield from self.add_goal(1.8)
            yield from self.start_route()
            yield from self.wait('sequential completion',lambda:not self.model.queueBusy and not self.shared.goal_active,80.)
            E.check('sequential_goals',len(self.model.goals)==2 and all(goal['status']=='완료' for goal in self.model.goals) and self.stopped(),goals=self.model.goals)
            yield from self.remove_all_goals();yield from self.add_goal(3.)
            yield from self.start_route();yield from self.wait('auto before takeover moving',self.moving,15.)
            self.do('manual_takeover_ui',self.press_pad);takeover=time.monotonic()
            yield from self.wait('manual takeover active',lambda:self.op().get('mode')=='MANUAL' and self.op().get('phase')=='ACTIVE' and self.moving(),12.)
            E.check('takeover_canceled_action',not self.shared.goal_active and not self.op().get('goal_id'),operator=self.op(),trigger=takeover)
            yield from self.delay(.5)
            yield from self.stop_wait('takeover_release',self.release_pad)
            self.do('stop_button_start',self.press_pad);yield from self.wait('moving before stop',self.moving)
            yield from self.stop_wait('operator_stop_ui',lambda:self.click('stopButton'))
            self.release_pad();yield from self.wait('stop released owner',lambda:not self.model.token and not self.op().get('owner_id'))
            E.check('stop_releases_control',self.op()['mode']=='DISABLED' and not self.op()['base_active'])
        if args.continue_at in ('all','controls','focus','remainder','logs_remaining','sequential','goals','maps'):
            if args.saved_map:
                prefix=Path(args.saved_map).resolve().with_suffix('')
                members={suffix:Path(str(prefix)+suffix) for suffix in ('.yaml','.pgm','.posegraph','.data','.metadata.json')}
                E.check('reused_saved_bundle',all(path.is_file() for path in members.values()))
                E.report['reused_map_bundle']={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in members.values()}
            else:
                prefix=OUT/'map'/'operator-map';prefix.parent.mkdir()
                yield from self.map_operation('save',lambda:self.map_path('mapSaveButton',str(prefix)))
                E.check('map_files_saved',all(Path(str(prefix)+suffix).is_file() for suffix in ('.yaml','.pgm','.posegraph','.data','.metadata.json')))
            initial=list(self.get('pose'));old_session=self.op()['session_id']
            yield from self.map_operation('localize',lambda:self.map_path('mapLoadButton',str(prefix)+'.yaml'))
            yield from self.wait('localization disabled waiting pose',lambda:self.op().get('session_id')!=old_session and not self.op().get('base_active'),10.)
            yield from self.wait('new localization state shown in UI',lambda:self.model.guard_session==self.op().get('session_id') and self.model.ui.get('needsInitialPose'),3.)
            yield from self.wait('new AMCL map available for pose placement',
                                 lambda: bool(self.model.controlUi.get('canSetInitialPose')), 10.)
            E.check('load_requires_initial_pose',not self.op()['auto_ready'] and self.model.ui.get('needsInitialPose'),operator=self.op())
            yield from self.map_operation('initialpose',lambda:self.initial_pose_ui(initial))
            yield from self.wait('AMCL ready after initialpose',lambda:self.op().get('auto_ready'),30.)
            yield from self.wait('initial pose readiness reflected in UI',lambda:not self.model.ui.get('needsInitialPose'),3.)
            E.check('initialpose_ready_without_motion',not self.op()['base_active'] and self.stopped())
            old_session=self.op()['session_id']
            yield from self.map_operation('slam',self.new_map_ui)
            yield from self.wait('new SLAM ready disabled',lambda:self.op().get('session_id')!=old_session and self.op().get('auto_ready') and not self.op().get('base_active'),15.)
            E.check('new_slam_ready_without_motion',self.stopped(),operator=self.op())
        self.window.setProperty('page',0);self.window.requestActivate()
        yield from self.delay(.3)
        self.do('manual_before_close',self.press_pad)
        yield from self.wait('moving before window close',self.moving,12.)
        yield from self.delay(.25)
        with self.shared.lock:
            now=time.monotonic();data=self.shared.data
            telemetry={topic:{'count':data['topics'][topic]['count'],'age':now-data['topics'][topic]['received']} for topic in self.baseline_counts}
            history_length=len(data['history']);trace_length=len(data['trace']);rpm=list(data['motor'].get('encoder_rpm',[]))
        E.check('live_history_disabled_with_fresh_motion_telemetry',not self.shared.history_enabled and history_length==0
            and trace_length>1 and any(abs(value)>.01 for value in rpm)
            and all(row['count']>self.baseline_counts[topic] and row['age']<.5 for topic,row in telemetry.items()),
            history_length=history_length,trace_length=trace_length,encoder_rpm=rpm,topics=telemetry)
        self.close_requested=True;self.close_trigger_wall=time.monotonic()
        self.do('window_close',self.window.close)
        yield

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


if not args.qml_only:
    manifest = ROOT/'reports/operator/latest_build.json'
    build = json.loads(manifest.read_text())
    changed = [p for p, h in build['source_sha256'].items() if not (ROOT/p).is_file() or hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != h]
    E.check('latest_built_sources', build['exit_code'] == 0 and build['source_unchanged'] and not changed, changed=changed)
    E.report['git_commit'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    E.report['build_commit'] = build.get('git_commit')
    E.report['build_manifest'] = str(manifest)
    E.report['build_source_sha256'] = build['source_sha256']
packages = ('omslam_operator', 'omslam_operator_msgs', 'omslam_operator_support', 'omslam_navigation', 'omslam_base', 'omslam_sim', 'omslam_system_monitor')
prefixes = {p: get_package_prefix(p) for p in packages}
E.check('installed_prefixes', all(Path(v).is_relative_to(ROOT/'install') for key, v in prefixes.items()
                                 if not (args.qml_only and key == 'omslam_operator')), prefixes=prefixes)
installed = {}
for module in (worker_module, live_module, app_module, visuals_module):
    source = ROOT/'src/omslam_operator/omslam_operator'/Path(module.__file__).name
    artifact = Path(module.__file__)
    installed[str(artifact)] = hashlib.sha256(artifact.read_bytes()).hexdigest()
    E.check('installed_'+module.__name__, artifact.is_relative_to(prefixes['omslam_operator'])
        and installed[str(artifact)] == hashlib.sha256(source.read_bytes()).hexdigest(), source=str(source), artifact=str(artifact))
if args.continue_at != 'all' and not args.qml_only:
    if not args.reuse_report: raise RuntimeError('--continue-at requires --reuse-report')
    previous_path=Path(args.reuse_report).resolve()
    previous=json.loads(previous_path.read_text())
    changes={path for path in set(previous['build_source_sha256'])|set(build['source_sha256'])
             if previous['build_source_sha256'].get(path)!=build['source_sha256'].get(path)}
    E.report['reused_evidence']={'path':str(previous_path),'sha256':hashlib.sha256(previous_path.read_bytes()).hexdigest(),
        'continue_at':args.continue_at,'source_changes':sorted(changes),
        'passed_checks':[check['name'] for check in previous['checks'] if check['status']=='PASS']}
    E.event('continuation',**E.report['reused_evidence'])
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
def close_observer():
    for action in (lambda: observer_proc.send_signal(signal.SIGINT), observer_proc.terminate, observer_proc.kill):
        if observer_proc.poll() is not None:
            break
        action()
        try:
            observer_proc.wait(timeout=5.)
            break
        except subprocess.TimeoutExpired:
            continue
    E.report['observer_exit_code'] = observer_proc.poll()
    os.close(observer_read)
    observer_output.close()

shared = worker_module.Shared(); worker = worker_module.RosWorker(shared, True); worker.start()
model = ObservedModel(shared)
qmlRegisterType(ObservedMap if args.diagnostics_only or args.profile_seconds else RobotMap, 'OhMyRobot', 1, 0, 'NativeMap')
engine = QQmlApplicationEngine(); engine.rootContext().setContextProperty('operatorModel', model)
engine.rootContext().setContextProperty('defaultMapDirectory', str(OUT/'map'))
engine.addImageProvider('camera', CameraProvider(shared))
startup_error = ''
try:
    if args.qml_only:
        ready = False
        deadline = time.monotonic() + 8.
        while time.monotonic() < deadline and observer_proc.poll() is None:
            try:
                ready = ready or bool(os.read(observer_read, 4096))
            except BlockingIOError:
                pass
            if ready and shared.node is not None:
                break
            time.sleep(.02)
        E.check('observer_and_worker_started', ready and shared.node is not None)
    share = Path(get_package_share_directory('omslam_operator'))
    entry = app_module.qml_path()
    E.report['qml_entry'] = str(entry)
    E.check('live_qml_entry', entry.resolve() == (share/'qml/design/Main.qml').resolve())
    engine.load(QUrl.fromLocalFile(str(entry)))
    if not engine.rootObjects():
        raise RuntimeError('installed QML failed to load')
    for source in (ROOT/'src/omslam_operator/qml/design').glob('*.qml'):
        artifact = share/'qml/design'/source.name
        installed[str(artifact)] = hashlib.sha256(artifact.read_bytes()).hexdigest()
        if installed[str(artifact)] != hashlib.sha256(source.read_bytes()).hexdigest():
            raise RuntimeError('Outdated installed '+str(artifact))
    E.report['installed_sha256'] = installed
    root = engine.rootObjects()[0]
    E.check('live_native_map_and_route_controls', root.findChild(QObject, 'nativeMap') is not None
            and root.findChild(QObject, 'moveButton') is not None)
except Exception as exc:
    startup_error = str(exc)
if startup_error or args.qml_only:
    close_session(model, shared, worker)
    rclpy.try_shutdown()
    close_observer()
    cleaned = not worker.is_alive() and observer_proc.poll() is not None
    E.report.update(status='FAIL' if startup_error or not cleaned else 'PASS', error=startup_error,
                    scope='installed QML startup', qt_messages=list(E.qt_messages), checks=list(E.checks),
                    worker_stopped=not worker.is_alive(), observer_stopped=observer_proc.poll() is not None)
    (OUT/'results.json').write_text(json_text(E.report, ensure_ascii=False, indent=2)+'\n')
    E.raw_log.close(); E.log.close()
    import shiboken6
    shiboken6.delete(engine)
    shiboken6.delete(model)
    raise SystemExit(0 if E.report['status']=='PASS' else 1)
window = engine.rootObjects()[0]
window.activeChanged.connect(lambda: model.setActive(window.isActive()))
model.setActive(window.isActive())
run = Run(app, model, shared, worker, window)
signal.signal(signal.SIGINT, lambda *_: app.quit())
signal.signal(signal.SIGTERM, lambda *_: app.quit())
app.exec()
close_session(model, shared, worker)
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
if run.close_requested and not run.failure:
    E.stage='reconnect_entrypoint'
    attach_log=(OUT/'reattach.log').open('w')
    attach_env=os.environ.copy(); attach_env['GZ_PARTITION']=PARTITION
    attach=subprocess.Popen([str(ROOT/'.venv-operator/bin/python'), '-m', 'omslam_operator.app',
        '--use-sim-time', '--snapshot', str(OUT/'reattach.json'), '--screenshot', str(OUT/'reattach.png'),
        '--capture-when-control-fresh', '--screenshot-delay', '0', '--quit-after', '8'], cwd='/tmp', env=attach_env,
        stdout=attach_log, stderr=subprocess.STDOUT, start_new_session=True)
    deadline=time.monotonic()+16.
    while attach.poll() is None and time.monotonic()<deadline:
        run.read_observer(); time.sleep(.05)
    if attach.poll() is None:
        os.killpg(attach.pid,signal.SIGINT)
        try: attach.wait(timeout=3.)
        except subprocess.TimeoutExpired: os.killpg(attach.pid,signal.SIGTERM);attach.wait(timeout=3.)
    attach_log.close();run.read_observer()
    try:
        snapshot=json.loads((OUT/'reattach.json').read_text())
        state=E.latest.get('/operator/state',{})
        E.check('real_entrypoint_reattach_no_motion', attach.returncode==0
            and snapshot.get('preview') is False and snapshot['ui'].get('controlFresh')
            and not state.get('owner_id') and not state.get('base_active')
            and state.get('mode')=='DISABLED' and state.get('stopped'), snapshot=snapshot,
            operator=state, exit_code=attach.returncode)
    except Exception as exc:
        run.failure='reattach: '+str(exc);E.report['error']=run.failure
if run.proc is not None:
    for p in processes(): run.owned[p['pid']] = p
    if run.proc.poll() is None: run.proc.send_signal(signal.SIGINT)
    try: run.proc.wait(timeout=40.)
    except subprocess.TimeoutExpired: E.report['cleanup_timeout'] = True
    E.report['session_exit_code'] = run.proc.poll()
    run.output.close()
close_observer()
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
post_checks=[]
limited=next((check for check in E.checks if check['name']=='speed_limit_ui' and check['status']=='PASS'),None)
if limited:
    samples=[row for row in final if limited['begin']<=row['wall']<=limited['end']]
    good=bool(samples) and max(abs(row['v']) for row in samples)<=limited['target']+1e-6
    post_checks.append({'name':'speed_limit_measured','status':'PASS' if good else 'FAIL',
        'target':limited['target'],'samples':len(samples),'max_linear':max((abs(row['v']) for row in samples),default=0.)})
takeover=next((check for check in E.checks if check['name']=='takeover_canceled_action' and check['status']=='PASS'),None)
if takeover:
    first=next((row for row in recorded if row.get('topic')=='/operator/manual' and row['wall']>=takeover['trigger']
        and row.get('deadman') and (abs(row.get('v',0.))>1e-8 or abs(row.get('w',0.))>1e-8)),None)
    between=[row for row in recorded if first and takeover['trigger']<=row['wall']<=first['wall']]
    stopped={topic:any(row.get('topic')==topic and abs(row['v'])<=.01 and abs(row['w'])<=.02 for row in between)
        for topic in ('/odom','/wheel/odometry')}
    state=any(row.get('topic')=='/operator/state' and row.get('stopped') for row in between)
    good=bool(first and all(stopped.values()) and state)
    post_checks.append({'name':'takeover_measured_stop_before_manual','status':'PASS' if good else 'FAIL',
        'trigger':takeover['trigger'],'first_nonzero_manual':first,'zero_velocity_observed':stopped,'stopped_state_observed':state})
E.checks.extend(post_checks)
post_ok=all(check['status']=='PASS' for check in post_checks)
E.report['recorder'] = {'raw_jsonl_complete': True, 'ram_sample_limit': 256,
                        'ram_response_limit': 0, 'ram_event_limit': 256, 'ram_check_limit': 128, 'raw_observer': 'separate process',
                        'detailed_callback_profiling': False, 'ui_tick_metrics': 'streaming maxima only'}
expired_states=[row for row in recorded if row.get('topic')=='/operator/state'
    and 'heartbeat expired' in row.get('reason','') and row.get('stage') not in ('window_close','reconnect_entrypoint')]
E.report['normal_gui_lease_expirations']=len(expired_states)
E.report['commands'] = {'nonzero_samples': len(final), 'unmatched_or_expired': len(lease_errors),
    'max_linear': max((abs(p['v']) for p in final), default=0), 'max_angular': max((abs(p['w']) for p in final), default=0)}
E.report['frame_latencies'] = list(E.gui_latencies)
E.report['ui_tick_metrics'] = E.ui_tick_stats
E.report['qt_messages'] = list(E.qt_messages)
E.report['diagnostics_only'] = bool(args.diagnostics_only)
E.report['source_unchanged'] = not [p for p, h in build['source_sha256'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != h]
E.report['status'] = 'PASS' if run.done and not run.failure and not remaining and not E.report['partition_remaining'] and E.report.get('session_exit_code') == 0 and E.report['observer_exit_code'] == 0 and (final or args.continue_at != 'all' or args.diagnostics_only) and post_ok and not lease_errors and not expired_states and E.report['source_unchanged'] and not E.qt_messages else 'FAIL'
E.report['checks'] = list(E.checks)
E.report['completed_at'] = datetime.now().astimezone().isoformat()
(OUT/'results.json').write_text(json_text(E.report, ensure_ascii=False, indent=2)+'\n')
E.raw_log.close(); E.log.close()
print(json_text({'status': E.report['status'], 'report': str(OUT/'results.json'), 'error': E.report.get('error'), 'remaining': remaining}, ensure_ascii=False), flush=True)
# Destroy the QML tree before Python tears down its QObject context/provider globals.
import shiboken6
shiboken6.delete(engine)
shiboken6.delete(model)
raise SystemExit(0 if E.report['status'] == 'PASS' else 1)
