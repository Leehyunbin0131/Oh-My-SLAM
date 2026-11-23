"""Gazebo dynamics/sensors layered on the pinned, unmodified CAD base URDF.

Masses and friction are simulation assumptions, not physical calibration.
The wheel odometry plugin integrates joint motion. Ground truth is isolated for
verification and must never feed EKF/SLAM. No hardware device is opened here.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


DEFAULT_SETTINGS = {
    "wheel_radius": .068, "wheel_separation": .480, "max_wheel_rpm": 100.,
    "base_mass": 8., "wheel_mass": .30, "caster_swivel_mass": .10,
    "caster_wheel_mass": .10, "sensor_mass": .10,
}



def add_inertia(link, mass, size, origin=(0, 0, 0)):
    inertia = ET.SubElement(link, 'inertial')
    ET.SubElement(inertia, 'origin', xyz=' '.join(map(str, origin)), rpy='0 0 0')
    ET.SubElement(inertia, 'mass', value=str(mass))
    x, y, z = size
    ET.SubElement(inertia, 'inertia',
                  ixx=str(mass * (y*y + z*z) / 12),
                  iyy=str(mass * (x*x + z*z) / 12),
                  izz=str(mass * (x*x + y*y) / 12), ixy='0', ixz='0', iyz='0')


def simulation_urdf(settings=None):
    """Return complete URDF using only the installed omslam_description assets.

    Optional overrides only tune simulation dynamics. Geometric radius and
    separation match the pinned CAD collision/joint geometry and cannot drift
    independently from that model.
    """
    settings = DEFAULT_SETTINGS | (settings or {})
    if set(settings) != set(DEFAULT_SETTINGS):
        raise ValueError('Unknown simulation model setting')
    if any(not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0 for v in settings.values()):
        raise ValueError('Simulation settings must be positive finite values')
    if settings['wheel_radius'] != .068 or settings['wheel_separation'] != .480:
        raise ValueError('Drive geometry must match the pinned CAD joints and collisions')
    if settings['max_wheel_rpm'] > 100.:
        raise ValueError('Simulation wheel limit cannot exceed 100 RPM')
    description_dir = Path(get_package_share_directory('omslam_description')) / 'upstream'
    robot = ET.parse(description_dir / 'urdf/base.urdf').getroot()
    for link in robot.findall('link'):
        name = link.attrib['name']
        if name == 'base_link':
            add_inertia(link, settings['base_mass'], (.406, .446, .2), (-.14, 0, .046))
        elif name in ('left_wheel_link', 'right_wheel_link'):
            mass, radius = settings['wheel_mass'], settings['wheel_radius']
            inertial = ET.SubElement(link, 'inertial')
            ET.SubElement(inertial, 'mass', value=str(mass))
            transverse = mass * (3*radius*radius + .024**2) / 12
            ET.SubElement(inertial, 'inertia', ixx=str(transverse),
                          iyy=str(.5*mass*radius*radius), izz=str(transverse),
                          ixy='0', ixz='0', iyz='0')
        elif 'caster_swivel' in name:
            add_inertia(link, settings['caster_swivel_mass'], (.0715, .053, .075),
                        (-.00925, 0, -.0375))
        elif 'caster_wheel' in name:
            add_inertia(link, settings['caster_wheel_mass'], (.076, .033, .076))
        elif name in ('lidar_link', 'camera_link'):
            add_inertia(link, settings['sensor_mass'], (.08, .08, .04))

        if link.find('collision') is not None:
            gazebo = ET.SubElement(robot, 'gazebo', reference=name)
            friction = .8 if name in ('left_wheel_link', 'right_wheel_link') else .2
            ET.SubElement(gazebo, 'mu1').text = str(friction)
            ET.SubElement(gazebo, 'mu2').text = str(friction)
        for mesh in link.findall('.//mesh'):
            # Absolute URI works both in RViz and Gazebo, independent of Fuel.
            mesh.attrib['filename'] = mesh.attrib['filename'].replace(
                'package://oh_my_mobilerobot_description', Path(description_dir).as_uri())

    # A dedicated external IMU frame is an explicit simulation mount. Its
    # identity orientation is not a claim about the physical HFI-A9 mounting.
    ET.SubElement(robot, 'link', name='imu_link')
    imu_joint = ET.SubElement(robot, 'joint', name='imu_joint', type='fixed')
    ET.SubElement(imu_joint, 'parent', link='base_link')
    ET.SubElement(imu_joint, 'child', link='imu_link')
    ET.SubElement(imu_joint, 'origin', xyz='0 0 0', rpy='0 0 0')

    for joint in robot.findall('joint'):
        if joint.attrib['type'] == 'continuous':
            ET.SubElement(joint, 'dynamics', damping='0.001', friction='0.0')
    gazebo = ET.SubElement(robot, 'gazebo')
    drive = ET.SubElement(gazebo, 'plugin', filename='gz-sim-diff-drive-system',
                          name='gz::sim::systems::DiffDrive')
    for key, value in {
        'left_joint': 'left_wheel_joint', 'right_joint': 'right_wheel_joint',
        'wheel_separation': settings['wheel_separation'],
        'wheel_radius': settings['wheel_radius'], 'odom_publish_frequency': 50,
        'topic': '/model/oh_my_mobilerobot/cmd_vel',
        'odom_topic': '/model/oh_my_mobilerobot/odometry',
        'tf_topic': '/model/oh_my_mobilerobot/unbridged_wheel_tf',
        'frame_id': 'odom', 'child_frame_id': 'base_footprint',
        'max_linear_velocity': 2*math.pi*.068*settings['max_wheel_rpm']/60,
        'min_linear_velocity': -2*math.pi*.068*settings['max_wheel_rpm']/60,
        'max_angular_velocity': 4*math.pi*.068*settings['max_wheel_rpm']/60/.480,
        'min_angular_velocity': -4*math.pi*.068*settings['max_wheel_rpm']/60/.480,
        'max_linear_acceleration': .8, 'min_linear_acceleration': -1.5,
        'max_angular_acceleration': 1.0, 'min_angular_acceleration': -1.5,
    }.items():
        ET.SubElement(drive, key).text = str(value)
    joints = ET.SubElement(gazebo, 'plugin', filename='gz-sim-joint-state-publisher-system',
                           name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(joints, 'topic').text = '/model/oh_my_mobilerobot/joint_state'
    ET.SubElement(joints, 'update_rate').text = '50'
    truth = ET.SubElement(gazebo, 'plugin', filename='gz-sim-odometry-publisher-system',
                          name='gz::sim::systems::OdometryPublisher')
    for key, value in {'odom_frame': 'sim_world', 'robot_base_frame': 'base_footprint',
                       'odom_topic': '/model/oh_my_mobilerobot/ground_truth',
                       'tf_topic': '/model/oh_my_mobilerobot/ground_truth_tf',
                       'dimensions': 3, 'odom_publish_frequency': 50}.items():
        ET.SubElement(truth, key).text = str(value)

    lidar_gazebo = ET.SubElement(robot, 'gazebo', reference='lidar_link')
    lidar = ET.SubElement(lidar_gazebo, 'sensor', name='lidar', type='gpu_lidar')
    for key, value in {'pose': '0 0 0.0303 0 0 0', 'topic': '/scan',
                       'gz_frame_id': 'laser_frame', 'update_rate': '10',
                       'always_on': 'true', 'visualize': 'false'}.items():
        ET.SubElement(lidar, key).text = value
    ray = ET.SubElement(lidar, 'lidar')
    horizontal = ET.SubElement(ET.SubElement(ray, 'scan'), 'horizontal')
    for key, value in {'samples': 720, 'resolution': 1,
                       'min_angle': -math.pi, 'max_angle': math.pi}.items():
        ET.SubElement(horizontal, key).text = str(value)
    distance = ET.SubElement(ray, 'range')
    for key, value in {'min': .15, 'max': 12.0, 'resolution': .01}.items():
        ET.SubElement(distance, key).text = str(value)
    noise = ET.SubElement(ray, 'noise')
    ET.SubElement(noise, 'type').text = 'gaussian'
    ET.SubElement(noise, 'mean').text = '0'
    ET.SubElement(noise, 'stddev').text = '0.005'
    camera_gazebo = ET.SubElement(robot, 'gazebo', reference='camera_link')
    rgbd = ET.SubElement(camera_gazebo, 'sensor', name='rgbd', type='rgbd_camera')
    for key, value in {'pose': '0 0 0 0 0 0', 'topic': '/camera',
                       'gz_frame_id': 'camera_link', 'update_rate': '10',
                       'always_on': 'true', 'visualize': 'false'}.items():
        ET.SubElement(rgbd, key).text = value
    camera = ET.SubElement(rgbd, 'camera')
    ET.SubElement(camera, 'optical_frame_id').text = 'camera_optical_frame'
    ET.SubElement(camera, 'horizontal_fov').text = '1.3'
    image = ET.SubElement(camera, 'image')
    for key, value in {'width': '320', 'height': '240', 'format': 'R8G8B8'}.items():
        ET.SubElement(image, key).text = value
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.12'
    ET.SubElement(clip, 'far').text = '5.0'
    imu_gazebo = ET.SubElement(robot, 'gazebo', reference='imu_link')
    imu = ET.SubElement(imu_gazebo, 'sensor', name='imu', type='imu')
    for key, value in {'topic': '/imu', 'gz_frame_id': 'imu_link',
                       'update_rate': '50', 'always_on': 'true'}.items():
        ET.SubElement(imu, key).text = value
    ET.indent(robot)
    return ET.tostring(robot, encoding='unicode')
