"""Display work stays bounded without losing occupancy or update semantics."""
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from PySide6.QtCore import QCoreApplication, QUrl
from PySide6.QtQml import QQmlComponent, QQmlEngine

from omslam_operator.live_model import LiveConsoleModel
from omslam_operator.visuals import RobotMap
from omslam_operator.worker import ConsoleNode, Shared


def grid_message(key, values, width=None):
    message = OccupancyGrid() if key == 'map' else Costmap()
    message.header.frame_id = 'map'
    metadata = message.info if key == 'map' else message.metadata
    metadata.resolution = .1
    metadata.origin.orientation.w = 1.
    width = len(values) if width is None else width
    if key == 'map':
        metadata.width, metadata.height = width, len(values) // width
    else:
        metadata.size_x, metadata.size_y = width, len(values) // width
    message.data = values
    return message


def receive_grid(shared, key, message):
    receiver = SimpleNamespace(s=shared, transform=lambda *_: (0., 0., 0.))
    ConsoleNode.grid(receiver, key, message)
    return shared.map_layers[key]


@pytest.mark.parametrize('key,values,expected', [
    ('map', [-1, 0, 1, 64, 65, 100],
     ['#ff14212e', '#ff2a4052', '#ff496278', '#ff496278', '#ff99afc3', '#ff99afc3']),
    ('global', [0, 1, 252, 253, 254, 255],
     ['#00000000', '#70ddb56b', '#70ddb56b', '#b0e98087', '#b0e98087', '#405d6f84']),
])
def test_grid_snapshot_keeps_occupancy_and_exact_display_palette(key, values, expected):
    shared = Shared()
    message = grid_message(key, values)
    snapshot = receive_grid(shared, key, message)
    # A subscriber retains raw occupancy once, even before a renderer exists.
    assert 'image' not in snapshot
    assert snapshot['cells'].dtype == (np.int8 if key == 'map' else np.uint8)
    assert not snapshot['cells'].flags.writeable
    assert snapshot['cells'][0].tolist() == values
    message.data[0] = 10
    assert snapshot['cells'][0, 0] == values[0]
    renderer = SimpleNamespace(_grid_images={})
    image = RobotMap.grid_image(renderer, key, snapshot)
    assert [image.pixelColor(x, 0).name(image.pixelColor(x, 0).NameFormat.HexArgb)
            for x in range(len(values))] == expected
    if key == 'map':
        app = QCoreApplication.instance() or QCoreApplication([])
        model = LiveConsoleModel(shared)
        model.timer.stop()
        model.display_timer.stop()
        model.setGoal(.05, .05, 0.)
        assert not model.goal['valid'] and '관측되지' in model.goal['reason']
        model.setGoal(.15, .05, 0.)
        assert model.goal['valid']
        model.setGoal(.45, .05, 0.)
        assert not model.goal['valid'] and '점유' in model.goal['reason']
        model.deleteLater()
        assert app is not None


def test_grid_image_cache_uses_latest_snapshot_without_reconverting_repaints():
    shared = Shared()
    renderer = SimpleNamespace(_grid_images={})
    first = receive_grid(shared, 'local', grid_message('local', [0, 1, 255]))
    image = RobotMap.grid_image(renderer, 'local', first)
    assert RobotMap.grid_image(renderer, 'local', first).cacheKey() == image.cacheKey()
    # Multiple updates may arrive while the layer is hidden. Only the newest
    # raw snapshot is retained, and showing it must not reuse the old image.
    receive_grid(shared, 'local', grid_message('local', [253, 1, 255]))
    newest = receive_grid(shared, 'local', grid_message('local', [254, 0, 255]))
    assert len(shared.map_layers) == 1
    assert newest['revision'] > first['revision']
    newest_image = RobotMap.grid_image(renderer, 'local', newest)
    assert newest_image.cacheKey() != image.cacheKey()
    assert newest_image.pixelColor(0, 0).red() == 233
    assert newest_image.pixelColor(1, 0).alpha() == 0
    assert RobotMap.grid_image(renderer, 'local', newest).cacheKey() == newest_image.cacheKey()
    assert first['cells'][0].tolist() == [0, 1, 255]


@pytest.fixture
def display(monkeypatch):
    app = QCoreApplication.instance() or QCoreApplication([])
    wall = [1000.]
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    shared = Shared()
    model = LiveConsoleModel(shared)
    model.timer.stop()
    model.display_timer.stop()
    model.now = lambda: Time(sec=100)
    shared.data['ros_time'] = 100.
    shared.data['operator'] = {'mode': 'DISABLED', 'phase': 'READY', 'owner_id': '',
        'epoch': 0, 'session_id': 'display-test', 'stopped': True, 'base_active': False,
        'operator_enabled': True, 'manual_ready': True, 'auto_ready': True,
        'reverse_allowed': False, 'linear_limit': .1, 'angular_limit': .3,
        'max_linear_velocity': .2, 'max_angular_velocity': .6, 'reason': 'ready'}
    for topic in ('/operator/state', '/odom', '/wheel/odometry', '/joint_states'):
        shared.data['topics'][topic] = {'received': wall[0], 'stamp': 100.}
    shared.data['velocity'] = {'/odom': {'v': 0., 'w': 0.}, '/wheel/odometry': {'v': 0., 'w': 0.}}
    receive_grid(shared, 'map', grid_message('map', [0] * 400, width=20))
    model.tick()
    model.refresh()
    shared.controls.clear()
    yield SimpleNamespace(app=app, shared=shared, model=model, wall=wall)
    model.timer.stop()
    model.display_timer.stop()
    model.deleteLater()


def test_qml_notifications_do_not_cross_display_categories(display):
    f = display
    engine = QQmlEngine()
    engine.rootContext().setContextProperty('operatorModel', f.model)
    component = QQmlComponent(engine)
    component.setData(b'''import QtQml
QtObject {
    property var control: operatorModel.controlUi
    property var telemetry: operatorModel.telemetryUi
    property var resources: operatorModel.resourcesUi
    property var rgb: operatorModel.rgbUi
    property var depth: operatorModel.depthUi
    property var logs: operatorModel.logEntries
    property int controlUpdates: 0
    property int telemetryUpdates: 0
    property int resourcesUpdates: 0
    property int rgbUpdates: 0
    property int depthUpdates: 0
    property int logUpdates: 0
    onControlChanged: controlUpdates++
    onTelemetryChanged: telemetryUpdates++
    onResourcesChanged: resourcesUpdates++
    onRgbChanged: rgbUpdates++
    onDepthChanged: depthUpdates++
    onLogsChanged: logUpdates++
}''', QUrl())
    view = component.create()
    assert view is not None, [error.toString() for error in component.errors()]
    fields = ['controlUpdates', 'telemetryUpdates', 'resourcesUpdates', 'rgbUpdates', 'depthUpdates', 'logUpdates']
    for field in fields:
        view.setProperty(field, 0)
    map_updates = []
    f.model.mapChanged.connect(map_updates.append)
    f.shared.data['cameras']['rgb'] = {'revision': 1, 'received': f.wall[0], 'stamp': 100.}
    f.model.refresh()
    assert view.property('rgbUpdates') == 1
    assert all(view.property(field) == 0 for field in fields if field != 'rgbUpdates')
    assert map_updates == []
    f.model.refresh()
    assert view.property('rgbUpdates') == 1  # Same frame and freshness: no notification.
    f.shared.data['joints'] = {'left_wheel_joint': {'rpm': 12.}, 'right_wheel_joint': {'rpm': 13.}}
    f.model.refresh()
    assert view.property('telemetryUpdates') == 1
    assert f.model.telemetryUi['leftRpm'] == '12.0'
    assert view.property('controlUpdates') == view.property('resourcesUpdates') == view.property('depthUpdates') == 0
    assert view.property('rgbUpdates') == 1 and map_updates == []
    view.deleteLater()


def test_freshness_expiry_notifies_camera_control_and_pose_without_new_messages(display):
    f = display
    for key in ('rgb', 'depth'):
        f.shared.data['cameras'][key] = {'revision': 1, 'received': f.wall[0], 'stamp': 100.}
    f.shared.data['pose'] = [1., 1., .2]
    f.shared.data['pose_received'] = f.wall[0]
    f.model.refresh()
    assert f.model.rgbUi['rgbFresh'] and f.model.depthUi['depthFresh']
    assert f.model.controlUi['controlFresh'] and f.model.telemetryUi['poseX'] == 1.
    maps, signals = [], []
    f.model.mapChanged.connect(maps.append)
    f.model.rgbUiChanged.connect(lambda: signals.append('rgb'))
    f.model.depthUiChanged.connect(lambda: signals.append('depth'))
    f.model.controlUiChanged.connect(lambda: signals.append('control'))
    f.wall[0] += .81
    f.model.refresh()
    assert not f.model.rgbUi['rgbFresh'] and not f.model.depthUi['depthFresh']
    assert not f.model.controlUi['controlFresh']
    assert f.model.telemetryUi['poseX'] is None
    assert {'rgb', 'depth', 'control'} <= set(signals)
    assert 'pose' in maps
    assert not f.shared.controls  # Presentation expiry creates no commands.


def test_goal_changes_and_action_terminal_rows_have_dedicated_notifications(display):
    f = display
    updates = []
    f.model.goalsChanged.connect(lambda: updates.append([dict(g) for g in f.model.goals]))
    f.model.addWaypoint(.3, .4, .5)
    first = f.model.goals[0]['uid']
    assert updates[-1][0]['uid'] == first
    f.model.addWaypoint(.7, .6, .1)
    f.model.moveWaypoint(0, 1)
    assert updates[-1][1]['uid'] == first
    f.model.editWaypoint(1, .4, .5, 1.)
    assert updates[-1][1]['yaw'] == 1.
    f.model.queue_waiting, f.model.queue_index = True, 1
    f.model.queue[1]['status'] = '이동 중'
    f.model.refresh()
    f.shared.data['goal_result'] = {'success': False, 'status': 5}
    f.shared.goal_active = False
    f.model.tick()
    f.model.refresh()
    assert updates[-1][1]['status'] == '취소됨'
    count = len(updates)
    f.model.refresh()
    assert len(updates) == count
