"""Airport robot, measured wheel feedback, virtual sensors and local EKF."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from omslam_description.model import simulation_urdf


def setup(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    share = Path(get_package_share_directory('omslam_sim'))
    localization = Path(get_package_share_directory('omslam_localization'))
    world = get('world')
    if world not in ('airport', 'p4_course'):
        raise ValueError(f'Unsupported simulator world: {world}')
    spawn_x = get('x') or ('-108.0' if world == 'airport' else '0.0')
    urdf = simulation_urdf()
    airport = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/airport.launch.py')),
        launch_arguments={key:get(key) for key in ('world','gui','pedestrians','gui_renderer','verbosity')} .items())
    relay = get('fault_relay').lower() == 'true'
    actions = [
        airport,
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             parameters=[{'robot_description':urdf, 'use_sim_time':True}], output='screen'),
        Node(package='ros_gz_sim', executable='create', name='spawn_robot', output='screen',
             arguments=['-world',get('world'),'-name','oh_my_mobilerobot','-topic','/robot_description',
                        '-x',spawn_x,'-y',get('y'),'-z','0.015','-Y',get('yaw')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='robot_sensor_bridge',
             parameters=[{'config_file':str(share/'config/robot_bridge.yaml'),'use_sim_time':True}], output='screen',
             remappings=[('/scan','/sim/sensors/scan'),
                         ('/camera/depth/image_rect_raw','/sim/sensors/depth')] if relay else []),
        LifecycleNode(package='omslam_sim', executable='sim_base', name='sim_base', namespace='',
                      parameters=[str(share/'config/sim_base.yaml')], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(localization/'launch/localization.launch.py')),
            launch_arguments={'use_sim_time':'true','imu_source':'external','publish_static_tfs':'false'}.items()),
    ]
    if relay:
        actions.append(Node(package='omslam_sim', executable='sensor_fault_relay',
                            name='sensor_fault_relay', parameters=[{'use_sim_time':True}], output='screen'))
    return actions


def generate_launch_description():
    defaults = {'world':'airport','fault_relay':'false','gui':'true','pedestrians':'true','gui_renderer':'ogre','verbosity':'2',
                'x':'','y':'0.0','yaw':'0.0'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
