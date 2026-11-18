"""Small, page-specific Qt payloads; dense geometry stays in the native renderer."""

# Every callback replaces these values. Dense arrays are never marshalled to QML.
_DENSE_KEYS = {'history', 'trace', 'path', 'scan', 'memory_points', 'obstacles', 'points',
               'diagnostics', 'events', 'topics', 'cameras', 'velocity'}


def presentation_snapshot(raw, page, sensor_page, imu_frame):
    """Call under Shared.lock. Copy mutable containers, project visible plots only."""
    data = {key: value for key, value in raw.items() if key not in _DENSE_KEYS}
    data['topics'] = {key: dict(value) for key, value in raw.get('topics', {}).items()}
    data['cameras'] = dict(raw.get('cameras', {}))
    data['velocity'] = {key: dict(value) for key, value in raw.get('velocity', {}).items()}
    data['events'] = [dict(row) for row in raw.get('events', [])[:200 if page == 3 else 1]]
    data['diagnostics'] = dict(raw.get('diagnostics', {})) if page == 3 else {}
    data['gyroPlot'] = []
    data['rpmPlot'] = []
    data['velocityPlot'] = []
    history = raw.get('history', [])
    if history and (page == 2 or (page == 1 and sensor_page == 1)):
        start = history[-1]['t'] - 30.
        rows = [row for row in history[-300:] if row['t'] >= start]
        if page == 1:
            key = 'gyro_base' if imu_frame else 'gyro'
            data['gyroPlot'] = [{'t': row['t'], 'v': list(row.get(key, []))} for row in rows]
        else:
            data['rpmPlot'] = [{'t': row['t'], 'v': list(row.get('rpm', []))} for row in rows]
            data['velocityPlot'] = [{'t': row['t'], 'v': [
                row.get('velocity', {}).get(topic, {}).get('v', 0.)
                for topic in ('/cmd_vel/requested', '/odom', '/wheel/odometry')]} for row in rows]
    if imu_frame:
        data['imu'] = data.get('imu_base', {'rpy': [], 'valid': False, 'gyro': [], 'covariance': []})
    return data
