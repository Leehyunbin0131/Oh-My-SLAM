from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import pytest
from omslam_description import model


PACKAGE = Path(__file__).resolve().parents[1]


def build(monkeypatch):
    monkeypatch.setattr(model, 'get_package_share_directory', lambda _: str(PACKAGE))
    return ET.fromstring(model.simulation_urdf())


def test_pinned_geometry_and_sensor_contract(monkeypatch):
    source = ET.parse(PACKAGE/'upstream/urdf/base.urdf').getroot()
    generated = build(monkeypatch)
    for old in source.findall('joint'):
        new = generated.find(f"joint[@name='{old.get('name')}']")
        for tag in ('origin', 'axis', 'parent', 'child'):
            if old.find(tag) is not None:
                assert new.find(tag).attrib == old.find(tag).attrib
    for mesh in generated.findall('.//mesh'):
        assert Path(mesh.get('filename').removeprefix('file://')).is_file()
    sensors = {s.get('name'): s for s in generated.findall('.//sensor')}
    assert sensors['lidar'].get('type') == 'gpu_lidar'
    assert sensors['lidar'].findtext('gz_frame_id') == 'laser_frame'
    assert sensors['imu'].findtext('gz_frame_id') == 'imu_link'
    assert generated.find("joint[@name='imu_joint']/parent").get('link') == 'base_link'
    assert sensors['rgbd'].findtext('camera/optical_frame_id') == 'camera_optical_frame'
    assert sensors['rgbd'].findtext('pose') == '0 0 0 0 0 0'
    drive = generated.find("gazebo/plugin[@name='gz::sim::systems::DiffDrive']")
    assert drive.findtext('child_frame_id') == 'base_footprint'
    assert drive.findtext('odom_topic') == '/model/oh_my_mobilerobot/odometry'
    assert drive.findtext('tf_topic').endswith('/unbridged_wheel_tf')
    assert float(drive.findtext('max_linear_velocity')) == pytest.approx(2*model.math.pi*.068*70/60)
    assert float(drive.findtext('min_linear_velocity')) == pytest.approx(-2*model.math.pi*.068*70/60)
    assert float(drive.findtext('max_angular_velocity')) == pytest.approx(4*model.math.pi*.068*70/60/.480)
    assert [float(drive.findtext(name)) for name in (
        'max_linear_acceleration', 'min_linear_acceleration',
        'max_angular_acceleration', 'min_angular_acceleration')] == [.8, -1.5, 1.0, -1.0]
    truth = generated.find("gazebo/plugin[@name='gz::sim::systems::OdometryPublisher']")
    assert truth.findtext('odom_topic') == '/model/oh_my_mobilerobot/ground_truth'


def test_provenance_and_geometry_override_rejection(monkeypatch):
    manifest = json.loads((PACKAGE/'provenance.json').read_text())
    prefix = 'ros/oh_my_mobilerobot_description/'
    for path, expected in manifest['files'].items():
        assert hashlib.sha256((PACKAGE/'upstream'/path.removeprefix(prefix)).read_bytes()).hexdigest() == expected
    monkeypatch.setattr(model, 'get_package_share_directory', lambda _: str(PACKAGE))
    for settings in ({'wheel_radius': .1}, {'wheel_separation': .6}, {'max_wheel_rpm': 100.01}, {'base_mass': float('nan')}):
        with pytest.raises(ValueError):
            model.simulation_urdf(settings)
    # The override ceiling remains 100 RPM above the 70 RPM default.
    drive = ET.fromstring(model.simulation_urdf({'max_wheel_rpm': 100.})).find(
        "gazebo/plugin[@name='gz::sim::systems::DiffDrive']")
    assert float(drive.findtext('max_linear_velocity')) == pytest.approx(2*model.math.pi*.068*100/60)
