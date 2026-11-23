"""Map-bound no-entry rectangles, authorized edits and final command barrier."""
import json
import math
from pathlib import Path
import time
import uuid

FOOTPRINT = ((.11, .28), (.11, -.28), (-.41, -.28), (-.41, .28))
MAX_ZONES = 64


def validate_polygons(polygons):
    """Validate JSON-like vertices, returning canonical map rectangles."""
    if not isinstance(polygons, (list, tuple)) or len(polygons) > MAX_ZONES:
        raise ValueError('진입 금지 구역은 최대 64개까지 지정할 수 있습니다')
    result = []
    for polygon in polygons:
        if not isinstance(polygon, (list, tuple)) or len(polygon) != 4:
            raise ValueError('진입 금지 구역은 네 꼭짓점의 사각형이어야 합니다')
        vertices = []
        for point in polygon:
            if (not isinstance(point, (list, tuple)) or len(point) != 2 or
                    any(isinstance(v, bool) or not isinstance(v, (float, int)) or
                        not math.isfinite(v) or abs(v) > 10000 for v in point)):
                raise ValueError('진입 금지 구역 좌표가 유효하지 않습니다')
            vertices.append(tuple(float(v) for v in point))
        xs, ys = zip(*vertices)
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        # ROS Polygon carries Point32. At airport coordinates its spacing can
        # exceed 1 micrometre. Bound the allowance to one ULP per axis.
        def min_width(lo, hi):
            exponent = math.frexp(max(abs(lo), abs(hi)))[1]
            return .05 - min(.0001, max(1e-6, math.ldexp(1., exponent - 24)))
        if x1 - x0 < min_width(x0, x1) or y1 - y0 < min_width(y0, y1):
            raise ValueError('진입 금지 구역의 너비와 높이는 5 cm 이상이어야 합니다')
        expected = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        if set(vertices) != set(expected):
            raise ValueError('지도 축에 맞춘 사각형만 지원합니다')
        result.append([list(p) for p in expected])
    return result


def load_keepout_file(path):
    if not path:
        return []
    value = json.loads(Path(path).read_text())
    if (not isinstance(value, dict) or value.get('schema_version') != 1 or
            value.get('frame_id') != 'map'):
        raise ValueError('지원하지 않는 진입 금지 구역 파일입니다')
    return validate_polygons(value.get('polygons'))


def intersects(a, b):
    """Convex SAT including touching edges, in world metres."""
    for polygon in (a, b):
        for i, p in enumerate(polygon):
            q = polygon[(i + 1) % len(polygon)]
            axis = (p[1] - q[1], q[0] - p[0])
            pa = [x * axis[0] + y * axis[1] for x, y in a]
            pb = [x * axis[0] + y * axis[1] for x, y in b]
            if max(pa) < min(pb) - 1e-9 or max(pb) < min(pa) - 1e-9:
                return False
    return True


def footprint_at(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in FOOTPRINT]


def pose_allowed(polygons, x, y, yaw):
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        return False
    footprint = footprint_at(x, y, yaw)
    return not any(intersects(footprint, polygon) for polygon in polygons)


def grid_goal_error(x, y, yaw, origin, resolution, width, height, cells):
    """Check every occupied or unknown grid cell touched by the padded footprint."""
    ox, oy, heading = origin
    c, s = math.cos(heading), math.sin(heading)
    polygon = [((c*(px-ox) + s*(py-oy))/resolution,
                (-s*(px-ox) + c*(py-oy))/resolution) for px, py in footprint_at(x, y, yaw)]
    xs, ys = zip(*polygon)
    if min(xs) < 0 or min(ys) < 0 or max(xs) >= width or max(ys) >= height:
        return '차체가 지도 범위를 벗어납니다'
    for j in range(math.floor(min(ys)), math.floor(max(ys))+1):
        for i in range(math.floor(min(xs)), math.floor(max(xs))+1):
            value = cells[j*width+i]
            if (value < 0 or value >= 65) and intersects(polygon, ((i,j),(i+1,j),(i+1,j+1),(i,j+1))):
                return '차체 일부가 관측되지 않은 영역과 겹칩니다' if value < 0 else '차체가 점유된 위치와 겹칩니다'
    return ''


def motion_allowed(polygons, pose, vx, wz):
    """Sample current footprint and a conservative command/braking horizon.

    This is a virtual barrier, not a physical stopping-distance certification.
    Base command expiry remains an independent safeguard.
    """
    if not all(math.isfinite(v) for v in (*pose, vx, wz)):
        return False
    if not polygons:
        return True
    # Allow 300 ms command transport/expiry plus a fixed virtual-wall projection.
    # These divisors are deliberately conservative constants, not the
    # configured actuator deceleration, and they do not limit cruising speed.
    horizon = .30 + max(abs(vx) / .50, abs(wz) / 1.0)
    # At most 1 cm of footprint vertex travel between sampled poses.
    radius = math.hypot(.41, .28)
    count = max(1, math.ceil(horizon * (abs(vx) + radius * abs(wz)) / .01))
    x, y, yaw = pose
    for i in range(count + 1):
        t = horizon * i / count
        angle = yaw + wz * t
        if abs(wz) < 1e-9:
            px, py = x + vx * t * math.cos(yaw), y + vx * t * math.sin(yaw)
        else:
            px = x + vx / wz * (math.sin(angle) - math.sin(yaw))
            py = y - vx / wz * (math.cos(angle) - math.cos(yaw))
        if not pose_allowed(polygons, px, py, angle):
            return False
    return True


class KeepoutManager:
    def __init__(self, node):
        from geometry_msgs.msg import Polygon, Point32
        from omslam_operator_msgs.msg import KeepoutZones
        from omslam_operator_msgs.srv import SetKeepoutZones
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.node = node
        self.polygons = load_keepout_file(node.p['keepout_file'])
        self.revision = 0
        self.session_id = node.operator.session_id if node.operator else str(uuid.uuid4())
        self.message_type, self.polygon_type, self.point_type = KeepoutZones, Polygon, Point32
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(KeepoutZones, '/operator/keepouts', qos)
        self.service = node.create_service(SetKeepoutZones, '/operator/set_keepouts', self.set_zones)
        self.publish()

    def publish(self):
        message = self.message_type()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.session_id, message.revision = self.session_id, self.revision
        message.polygons = [self.polygon_type(points=[self.point_type(x=x, y=y, z=0.)
                            for x, y in p]) for p in self.polygons]
        self.publisher.publish(message)

    def set_zones(self, request, response):
        core, operator = self.node.core, self.node.operator
        try:
            if operator is None or not operator.authorized(request.token, request.epoch):
                raise ValueError('진입 금지 구역을 바꾸려면 제어권이 필요합니다')
            base = self.node.states.get('sim_base', (0, 0.))
            if (core.mode != 'DISABLED' or core.pending_mode is not None or core.active_goals or
                    core.allowed_goals or operator.pending or self.node.operation or
                    self.node.base_transition or base[0] != 2 or time.monotonic() - base[1] > 1. or
                    not core.stopped(self.node.now_ns(), time.monotonic())):
                raise ValueError('구동 해제와 실제 정지를 확인한 뒤 구역을 변경하세요')
            if request.expected_revision != self.revision:
                raise ValueError('구역이 다른 창에서 변경되었습니다. 최신 목록에서 다시 시도하세요')
            if any(abs(p.z) > 1e-6 or not math.isfinite(p.z)
                   for polygon in request.polygons for p in polygon.points):
                raise ValueError('진입 금지 구역은 지도 평면에 있어야 합니다')
            polygons = validate_polygons([[[p.x, p.y] for p in poly.points]
                                          for poly in request.polygons])
            self.polygons = polygons
            self.revision += 1
            self.publish()
            response.success, response.message = True, '진입 금지 구역을 적용했습니다'
        except (ValueError, TypeError) as exc:
            response.success, response.message = False, str(exc)
        response.revision = self.revision
        return response

    def goal_allowed(self, pose):
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        if not pose_allowed(self.polygons, pose.position.x, pose.position.y, yaw):
            return False
        grid = self.node.map_grid
        if grid is None:
            return False
        origin = grid.info.origin
        q = origin.orientation
        heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return not grid_goal_error(pose.position.x, pose.position.y, yaw,
            (origin.position.x, origin.position.y, heading), grid.info.resolution,
            grid.info.width, grid.info.height, grid.data)

    def command_allowed(self, vx, wz):
        if (vx == 0. and wz == 0.) or not self.polygons:
            return True
        from rclpy.time import Time
        try:
            stamped = self.node.tf_buffer.lookup_transform('map', self.node.p['base_frame'], Time())
            stamp = stamped.header.stamp.sec * 1_000_000_000 + stamped.header.stamp.nanosec
            age = (self.node.now_ns() - stamp) * 1e-9
            if not -self.node.p['future_tf_tolerance'] <= age < min(self.node.p['map_tf_timeout'], self.node.p['odom_timeout']):
                return False
            tr = stamped.transform
            q = tr.rotation
            if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
                return False
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            return motion_allowed(self.polygons, (tr.translation.x, tr.translation.y, yaw), vx, wz)
        except Exception:
            # Missing map localization cannot authorize crossing a virtual wall.
            return False
