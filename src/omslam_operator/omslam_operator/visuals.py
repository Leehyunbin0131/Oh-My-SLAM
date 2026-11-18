"""Native map canvas and bounded, thread-safe live camera image provider."""
import math
import numpy as np
from PySide6.QtCore import QObject, Property, Signal, Slot, QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PySide6.QtQuick import QQuickPaintedItem, QQuickImageProvider
from .geometry import grid_world


class CameraProvider(QQuickImageProvider):
    def __init__(self, shared):
        super().__init__(QQuickImageProvider.Image)
        self.shared = shared

    def requestImage(self, identifier, size, requestedSize):
        with self.shared.lock:
            image = self.shared.images.get(identifier.split('/')[0], QImage()).copy()
        if image.isNull():
            image = QImage(1, 1, QImage.Format_RGB888)
            image.fill(QColor('#101b28'))
        if size is not None:
            size.setWidth(image.width())
            size.setHeight(image.height())
        return image


class RobotMap(QQuickPaintedItem):
    sourceChanged = Signal()
    viewChanged = Signal()
    selectionChanged = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._source = None
        self.center = [0., 0.]
        self.scale = 65.
        self.follow = False
        self.layers = {'global': False, 'local': False, 'scan': True,
                       'memory': True, 'obstacles': False}
        self.insets = (0., 0., 0., 0.)
        self._screen_origin = (0., 0.)
        self.fitted = False
        self._selected = -1
        self._grid_images = {}
        self.setAntialiasing(True)
        self.widthChanged.connect(self._update_screen_origin)
        self.heightChanged.connect(self._update_screen_origin)
        self.visibleChanged.connect(self.refreshVisible)

    @Property(QObject, notify=sourceChanged)
    def source(self):
        return self._source

    @source.setter
    def source(self, value):
        if self._source is value:
            return
        if self._source:
            self._source.mapChanged.disconnect(self.refreshLayer)
            self._source.goalsChanged.disconnect(self.refreshVisible)
            if hasattr(self._source, 'keepoutsChanged'):
                self._source.keepoutsChanged.disconnect(self.refreshVisible)
        self._source = value
        self.fitted = False
        self._grid_images.clear()
        if value:
            value.mapChanged.connect(self.refreshLayer)
            value.goalsChanged.connect(self.refreshVisible)
            if hasattr(value, 'keepoutsChanged'):
                value.keepoutsChanged.connect(self.refreshVisible)
        self.sourceChanged.emit()
        self.refreshVisible()

    @Property(int, notify=selectionChanged)
    def selectedIndex(self):
        return self._selected

    @selectedIndex.setter
    def selectedIndex(self, value):
        if value != self._selected:
            self._selected = value
            self.selectionChanged.emit()
            self.update()

    @Property(bool, notify=viewChanged)
    def followEnabled(self):
        return self.follow

    @Property(float, notify=viewChanged)
    def pixelsPerMetre(self):
        return self.scale

    def viewport(self):
        left, right, bottom, top = self.insets
        return QRectF(left, top, max(1., self.width()-left-right),
                      max(1., self.height()-bottom-top))

    def _update_screen_origin(self):
        left, right, bottom, top = self.insets
        self._screen_origin = (left + max(1., self.width()-left-right)/2,
                               top + max(1., self.height()-bottom-top)/2)
        self.viewChanged.emit()
        self.update()

    @Slot(float, float, float, float)
    def setViewport(self, left, right, bottom, top):
        self.insets = (left, right, bottom, top)
        self._update_screen_origin()

    def _sync_follow(self):
        if not self.follow or not self._source:
            return
        ui = getattr(self._source, 'ui', {})
        with self._source.s.lock:
            pose = self._source.s.data.get('pose')
            valid = (pose and len(pose) >= 2 and all(math.isfinite(v) for v in pose[:2])
                     and (not ui or ui.get('poseX') is not None))
            center = list(pose[:2]) if valid else None
        if center is not None and center != self.center:
            self.center = center
            self.viewChanged.emit()

    def refreshVisible(self):
        self._sync_follow()
        if self.isVisible():
            if self._source:
                with self._source.s.lock:
                    available = 'map' in self._source.s.map_layers
                if not available and not self.follow:
                    self.fitted = False
            self.update()

    @Slot(str)
    def refreshLayer(self, key):
        layer = {'memory_points': 'memory'}.get(key, key)
        if layer not in self.layers or self.layers[layer]:
            self.refreshVisible()

    def point(self, x, y):
        ox, oy = self._screen_origin
        return QPointF(ox+(x-self.center[0])*self.scale,
                       oy-(y-self.center[1])*self.scale)

    def world(self, x, y):
        ox, oy = self._screen_origin
        return (self.center[0]+(x-ox)/self.scale,
                self.center[1]-(y-oy)/self.scale)

    @Slot(float, float, result=QPointF)
    def screenPoint(self, x, y):
        self._sync_follow()
        return self.point(x, y)

    @Slot(float, float, result=QPointF)
    def worldPoint(self, x, y):
        self._sync_follow()
        return QPointF(*self.world(x, y))

    @Slot(float, float)
    def pan(self, dx, dy):
        self.follow = False
        self.center[0] -= dx/self.scale
        self.center[1] += dy/self.scale
        self.viewChanged.emit()
        self.update()

    @Slot(float)
    def zoom(self, factor):
        if math.isfinite(factor) and factor > 0:
            self.scale = max(.2, min(1000., self.scale*factor))
            self.viewChanged.emit()
            self.update()

    @Slot(bool)
    def followRobot(self, value):
        if self.follow != value:
            self.follow = value
            self.viewChanged.emit()
        if value:
            # A later first-map repaint must not silently override follow.
            self.fitted = True
            self._sync_follow()
        self.update()

    @Slot(str, bool)
    def setLayer(self, key, value):
        if key in self.layers and self.layers[key] != value:
            self.layers[key] = value
            self.refreshVisible()

    @Slot()
    def fit(self):
        if not self._source:
            return
        with self._source.s.lock:
            grid = self._source.s.map_layers.get('map')
        if not grid:
            return
        points = [grid_world(x, y, grid['resolution'], grid['origin'])
                  for x, y in ((0, 0), (grid['width'], 0),
                               (0, grid['height']), (grid['width'], grid['height']))]
        xs, ys = zip(*points)
        self.center = [(max(xs)+min(xs))/2, (max(ys)+min(ys))/2]
        area = self.viewport()
        self.scale = max(.2, min((area.width()-36)/max(.1, max(xs)-min(xs)),
                                (area.height()-36)/max(.1, max(ys)-min(ys))))
        self.follow = False
        self.fitted = True
        self.viewChanged.emit()
        self.update()

    def grid_image(self, key, grid):
        """Convert a visible raw snapshot once, retaining its identity with the image."""
        cached = self._grid_images.get(key)
        if cached and cached[0] is grid:
            return cached[1]
        cells = grid['cells']
        pixels = np.zeros(cells.shape, dtype=np.uint8)
        if key == 'map':
            pixels[cells == 0] = 1
            pixels[(cells > 0) & (cells < 65)] = 2
            pixels[cells >= 65] = 3
            palette = ['#14212e', '#2a4052', '#496278', '#99afc3']
        else:
            pixels[cells > 0] = 1
            pixels[cells >= 253] = 2
            pixels[cells == 255] = 3
            palette = ['#00000000', '#70ddb56b', '#b0e98087', '#405d6f84']
        image = QImage(pixels.data, grid['width'], grid['height'],
                       grid['width'], QImage.Format_Indexed8).copy()
        image.setColorTable([QColor(value).rgba() for value in palette])
        self._grid_images[key] = (grid, image)
        return image

    def line(self, painter, points, color, width=2, dashed=False):
        if len(points) < 2:
            return
        pen = QPen(QColor(color), width)
        if dashed:
            pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawPolyline(QPolygonF([self.point(p[0], p[1]) for p in points]))

    def robot(self, painter, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        points = [self.point(x+c*a-s*b, y+s*a+c*b)
                  for a, b in ((.09, .26), (.09, -.26), (-.39, -.26), (-.39, .26))]
        painter.setPen(QPen(QColor('#97ead2'), 2))
        painter.setBrush(QColor('#21453e'))
        painter.drawPolygon(QPolygonF(points))
        p, q = self.point(x, y), self.point(x+.35*c, y+.35*s)
        painter.drawLine(p, q)
        painter.setBrush(QColor('#97ead2'))
        painter.drawEllipse(p, 3, 3)

    def paint(self, painter):
        painter.fillRect(self.boundingRect(), QColor('#14212e'))
        if not self._source:
            return
        with self._source.s.lock:
            raw = self._source.s.data
            data = {key: raw.get(key, []) for key in
                    ('pose', 'path', 'scan', 'memory_points', 'obstacles')}
            data['trace'] = list(raw.get('trace', []))
            grids = dict(self._source.s.map_layers)
        if not self.fitted and 'map' in grids and self.width() > 1:
            self.fit()
        ui = getattr(self._source, 'ui', {})
        pose_valid = not ui or ui.get('poseX') is not None
        self._sync_follow()
        painter.setRenderHint(QPainter.SmoothPixmapTransform, False)
        for key in ('map', 'global', 'local'):
            if key not in grids or (key != 'map' and not self.layers[key]):
                continue
            grid = grids[key]
            origin = grid['origin']
            painter.save()
            painter.translate(self.point(origin[0], origin[1]))
            painter.rotate(-math.degrees(origin[2]))
            painter.scale(grid['resolution']*self.scale, -grid['resolution']*self.scale)
            painter.drawImage(QPointF(0, 0), self.grid_image(key, grid))
            painter.restore()
        if 'map' not in grids:
            painter.setPen(QColor('#8ba4c0'))
            painter.drawText(self.viewport(), Qt.AlignCenter,
                             '지도를 기다리는 중입니다\nROS 연결과 지도 상태를 확인하세요')
        # Map-frame regions are shared with the navigation server; paint only
        # acknowledged state, never a speculative local obstacle.
        for index, zone in enumerate(getattr(self._source, 'keepoutZones', [])):
            corners = [(zone['x_min'], zone['y_min']), (zone['x_max'], zone['y_min']),
                       (zone['x_max'], zone['y_max']), (zone['x_min'], zone['y_max'])]
            painter.setBrush(QColor('#60D95D6B'))
            painter.setPen(QPen(QColor('#F18C97'), 1.5))
            polygon = QPolygonF([self.point(x, y) for x, y in corners])
            painter.drawPolygon(polygon)
            rect = polygon.boundingRect()
            if rect.width() > 38 and rect.height() > 22:
                painter.setPen(QColor('#FFD4DA'))
                painter.drawText(rect, Qt.AlignCenter, '금지 ' + str(index + 1))
        self.line(painter, data.get('trace', []), '#63cfb1', 1.5)
        self.line(painter, data.get('path', []), '#88baff', 2.5, True)
        for key, field, color in (('scan', 'scan', '#63cfb1'),
                                   ('memory', 'memory_points', '#ddb56b'),
                                   ('obstacles', 'obstacles', '#e98087')):
            if self.layers[key]:
                painter.setPen(QPen(QColor(color), 3))
                painter.drawPoints(QPolygonF([self.point(*point) for point in data.get(field, [])]))
        if data.get('pose') and pose_valid:
            self.robot(painter, data['pose'])
        for index, goal in enumerate(self._source.goals):
            p = self.point(goal['x'], goal['y'])
            selected = index == self._selected
            active = goal.get('status') == '이동 중'
            painter.setBrush(QColor('#477eb6' if selected else '#274d72'))
            painter.setPen(QPen(QColor('#9be8d2' if active else '#a4ceff'), 2.5 if selected else 1.5))
            painter.drawEllipse(p, 17, 17)
            yaw = goal['yaw']
            c, s = math.cos(yaw), math.sin(yaw)
            def arrow(a, b):
                return QPointF(p.x()+a*c-b*s, p.y()-a*s-b*c)
            painter.drawLine(arrow(18, 0), arrow(32, 0))
            painter.drawLine(arrow(32, 0), arrow(27, 4))
            painter.drawLine(arrow(32, 0), arrow(27, -4))
            painter.setPen(QColor('#f0f6ff'))
            painter.drawText(QRectF(p.x()-17, p.y()-17, 34, 34), Qt.AlignCenter, str(index+1))
        # Metres in the map frame; +Y is not a geographic compass bearing.
        metre = 10**math.floor(math.log10(80/self.scale))
        if metre*self.scale < 35:
            metre *= 5
        area = self.viewport()
        x, y = area.center().x()-metre*self.scale/2, area.bottom()-4
        painter.setPen(QPen(QColor('#9cb3cb'), 1.2))
        painter.drawLine(QPointF(x, y), QPointF(x+metre*self.scale, y))
        painter.drawText(QPointF(x, y-8), f'{metre:g} m  ·  map +Y ↑')
