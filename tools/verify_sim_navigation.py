#!/usr/bin/env python3
"""One P4 acceptance route with simultaneous sensor, Nav2 and drive observation.

Starts a separate simulator and switches SLAM -> saved-map AMCL while keeping
that simulator alive. Ground truth only assesses goals/collision/stop; it never
feeds robot control or localization. Every run preserves its failures.
"""
from __future__ import annotations
import argparse
from collections import defaultdict, deque
from datetime import datetime
from functools import partial
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback
import numpy as np

from generate_p4_course import object_sdf
from p4_geometry import compose,inverse,angle_delta,polygon,rectangle,separation,intersects,interpolate,object_bounds
from verify_sim_robot import stamp_s,yaw,digest
from build_navigation import DEFAULT_BUILD, DEFAULT_VALIDATION, package_source_hashes, verified_build

WORKSPACE=Path(__file__).resolve().parents[1]
ALL_CHECKS=[f'V4-{n:02d}' for n in range(1,15)]
FIVE_GOAL_CHECKS=['installed_readiness','five_consecutive_goals','arrival_accuracy',
                  'collision_free','command_limits','normal_shutdown']


def jsonable(value):
    if isinstance(value,np.ndarray): return value.tolist()
    if isinstance(value,np.generic): return value.item()
    if isinstance(value,Path): return str(value)
    raise TypeError(str(type(value)))


def late_cancel_commands(events, request_time, terminal_wall_time):
    return [row for row in events if any(row['velocity']) and (
        (row['topic'] == '/cmd_vel' and row['stamp'] is not None and row['stamp'] > request_time)
        or (row['topic'] == '/sim/cmd_vel' and row['wall_time'] > terminal_wall_time))]


def last_valid_sensor_stamp(records, topic, initial_stamp, zero):
    def valid(row):
        if topic == '/scan':
            return row.get('valid_count', 0) > 0
        return row.get('valid_fraction', 0.) > .05
    return max([initial_stamp] + [row['stamp'] for row in records
        if row['topic'] == topic and row['wall_time'] <= zero['wall_time']
        and 0 < row['stamp'] <= zero['sim_time'] and valid(row)])


def command_limit_evidence(events, geometry, limits):
    """Check each velocity stage and the unsaturated kinematics at Gazebo's input."""
    stages = {}
    for topic in ('/cmd_vel/nav', '/cmd_vel/requested', '/cmd_vel/smoothed',
                  '/cmd_vel/monitored', '/cmd_vel', '/sim/cmd_vel'):
        values = np.asarray([e['velocity'] for e in events if e['topic'] == topic])
        stage = stages[topic] = {'samples': len(values), 'passed': False}
        if values.shape != (len(values), 6) or not len(values) or not np.isfinite(values).all():
            continue
        vx, wz = values[:, 0], values[:, 5]
        half_track = geometry['wheel_separation'] / 2
        left = (vx - half_track * wz) * (30 / math.pi) / geometry['left_radius']
        right = (vx + half_track * wz) * (30 / math.pi) / geometry['right_radius']
        peak_rpm = float(max(np.max(np.abs(left)), np.max(np.abs(right))))
        peak_vx, peak_wz = float(np.max(np.abs(vx))), float(np.max(np.abs(wz)))
        passed = (np.all(values[:, 1:5] == 0) and peak_vx <= limits['max_linear_velocity'] + 1e-6
                  and peak_wz <= limits['max_angular_velocity'] + 1e-6)
        if topic == '/sim/cmd_vel':
            passed = passed and 0 < peak_rpm <= geometry['max_rpm'] + 1e-6
        stage.update(passed=bool(passed), max_abs_vx=peak_vx, max_abs_wz=peak_wz, max_rpm=peak_rpm)
    return stages


def bounded_planner_evidence(entry,bt_events,states,ros_errors,marked=None,obstruction_epoch=None):
    begin=entry['start']['sim_time'];end=entry['end']['sim_time']
    events=[e for e in bt_events if begin<=e['sim_time'] and e['stage']==entry['start']['stage']]
    roots=[e['stamp'] for e in events if e['name']=='NavigateBounded' and e['current']=='RUNNING']
    if roots:events=[e for e in events if e['stamp']>=min(roots)]
    cancels=[e for e in events if e['name']=='CancelControl' and e['current']=='SUCCESS']
    waits=[e for e in events if e['name']=='Wait' and e['current']=='RUNNING']
    waited=[e for e in events if e['name']=='Wait' and e['current']=='SUCCESS']
    retries=[e for e in events if e['name']=='ComputePathToPose' and e['current']=='RUNNING'
             and waited and e['stamp']>=waited[0]['stamp']]
    failures=[e for e in ros_errors if e['name']=='planner_server' and 'failed to plan' in e['message']
              and retries and e.get('stage')==entry['start']['stage']
              and retries[0]['stamp'] <= e['stamp'] <= entry['end']['wall_epoch']]
    active_states=[e for e in states if begin<=e['sim_time']<=end]
    healthy=bool(active_states) and all(e['state']!='FAULT_LATCHED' for e in active_states)
    ordered=bool(cancels) and len(waits)==1 and len(waited)==1 and bool(retries)
    if ordered:ordered=cancels[0]['stamp']<=waits[0]['stamp']<waited[0]['stamp']<=retries[0]['stamp']
    after_obstruction=obstruction_epoch is None or (bool(cancels) and bool(waits)
        and cancels[0]['stamp']>=obstruction_epoch and waits[0]['stamp']>=obstruction_epoch)
    recoveries=max((f['recoveries'] for f in entry['feedback']),default=0)
    duration=end-(begin if marked is None else marked)
    # Jazzy flushes its BT logger before each tick; a terminating tick's final
    # transitions can remain buffered until the next goal. The recorded retry,
    # planner failure and terminal planner error code establish that outcome.
    return {'passed':entry['status']==6 and entry['error_code'] in (206,208) and healthy
            and ordered and after_obstruction and bool(failures) and recoveries<=1 and duration<=30,
            'error_code':entry['error_code'],'error_msg':entry.get('error_msg'),'guard_healthy':healthy,
            'wait_runs':len(waits),'recoveries':recoveries,'events':events,'ordered_recovery':ordered,
            'planner_failures_after_retry':failures,'terminal_action_status':entry['status'],
            'recovery_after_obstruction':after_obstruction,'obstruction_wall_epoch':obstruction_epoch,
            'terminal_evidence':'Recorded retry, planner_server failure and NavigateToPose planner error',
            'duration_s':duration}


class Evidence:
    def __init__(self,directory,checks,baseline=None,scenario='integrated'):
        self.directory=Path(directory).resolve()
        if (self.directory/'results.json').exists(): raise FileExistsError('Prior results are immutable; choose a new output directory')
        self.directory.mkdir(parents=True,exist_ok=True);(self.directory/'logs').mkdir(exist_ok=True)
        available=FIVE_GOAL_CHECKS if scenario=='five-goals' else ALL_CHECKS
        self.data={'scope':'Five consecutive simulation navigation goals' if scenario=='five-goals' else 'P4 simulation navigation',
                   'scenario':scenario,'status':'RUNNING','started_at':datetime.now().astimezone().isoformat(),
                   'requested_checks':checks,'checks':{k:{'status':'NOT_EXERCISED'} for k in available},'details':{}}
        self.baseline=baseline
        if baseline:
            old=json.loads(Path(baseline).read_text())
            self.data['baseline']={'path':str(Path(baseline).resolve()),'sha256':digest(baseline),
                                   'checks':{k:v for k,v in old.get('checks',{}).items() if k not in checks}}
        self.events=(self.directory/'events.jsonl').open('w')
        self.save()

    def save(self):
        (self.directory/'results.json').write_text(json.dumps(self.data,indent=2,default=jsonable)+'\n')

    def event(self,event,**values):
        self.events.write(json.dumps({'event':event,**values},default=jsonable)+'\n');self.events.flush()

    def detail(self,key,passed,**values):
        self.data['details'][key]={**values,'status':'PASS' if passed else 'FAIL'};self.save()
        print(key+(': PASS' if passed else ': FAIL'),flush=True)
        if not passed: raise AssertionError(key+': '+str(values))

    def check(self,key,passed,**values):
        if self.data['scenario']=='five-goals':
            key={'V4-01':'installed_readiness','V4-02':'arrival_accuracy',
                 'V4-03':'collision_free','V4-04':'command_limits'}.get(key,key)
        self.data['checks'][key]={**values,'status':'PASS' if passed else 'FAIL'};self.save()
        print(key+(': PASS' if passed else ': FAIL'),flush=True)
        if not passed: raise AssertionError(key+': '+str(values))


class NavigationRun:
    def __init__(self,result,timeout=2400):
        self.e=result;self.deadline=time.monotonic()+timeout;self.start_wall=time.monotonic()
        self.course=json.loads((WORKSPACE/'tools/p4_course.json').read_text())
        self.commands=[];self.processes={};self.logfiles={};self.samples={};self.counts=defaultdict(int);self.allowed_exit=set()
        self.timing=defaultdict(list);self.tf_edges={};self.states=[];self.goals=[];self.paths=[];self.trajectory=[]
        self.memory_events=[];self.cloud_events=[];self.costmap_events=[];self.command_events=[];self.sensor_observations=[]
        self.ros_errors=[];self.stage='I0';self.mode='slam';self.world_to_map=None;self.initial_odom=None
        self.odom_node=None;self.odom_executor=None
        self.node=None;self.context=None;self.executor=None;self.writer=None;self.recorded_topics=set()
        self.sim_time=0.;self.last_truth=None;self.minimum_clearance=float('inf');self.collision=[];self.safety_violation=None
        self.active_objects={k:list(v['center']) for k,v in self.course['objects'].items() if v['initial']}
        self.metrics={'stops':[],'faults':[],'memory':{},'source_final_matching':{},'dynamic_event':{}}
        self.current_goal=None;self.current_future=None;self.current_entry=None;self.guard_state={}
        self.depth_record_stages={'I4','I7','I9','I10'}
        self.tf_failures=[];self.clock_reset=False;self.last_sensor_stamps={};self.stopping=False
        self.depth_pending=deque(maxlen=5);self.depth_evidence=[]
        self.tf_observations=[];self.odom_observations=[];self.bt_events=[];self.object_intervals=[]
        self.map_events=[];self.graph_before=None
        self.object_positions=[]
        self.dwb_evaluations=[];self.last_evaluation=-float('inf')
        self.costmap_clients={};self.costmap_pending=[];self.costmap_snapshots=[]
        self.last_controller_failure_snapshot=-float('inf')
        self.reused_stages=set()

    def now(self): return {'sim_time':self.sim_time,'wall_time':time.monotonic()-self.start_wall,
                          'wall_epoch':time.time(),'stage':self.stage,'mode':self.mode}

    def event(self,event,**values): self.e.event(event,**self.now(),**values)

    def start(self,label,command):
        handle=(self.e.directory/'logs'/(label+'.log')).open('w')
        env=dict(os.environ,ROS_LOG_DIR=str(self.e.directory/'logs/ros'))
        proc=subprocess.Popen(command,cwd='/tmp',env=env,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        self.processes[label]=proc;self.logfiles[label]=handle
        self.commands.append({'label':label,'command':command,'cwd':'/tmp','pid':proc.pid,'start':self.now()})
        self.event('process_started',label=label,pid=proc.pid,command=command)
        return proc

    def stop_process(self,label):
        proc=self.processes.get(label)
        if proc is None:return
        for sig,seconds in ((signal.SIGINT,12),(signal.SIGTERM,5),(signal.SIGKILL,2)):
            if proc.poll() is not None:break
            os.killpg(proc.pid,sig)
            try:proc.wait(timeout=seconds)
            except subprocess.TimeoutExpired:continue
        self.event('process_stopped',label=label,pid=proc.pid,exit_code=proc.returncode)
        for command in self.commands:
            if command['pid']==proc.pid:command.update(exit_code=proc.returncode,end=self.now())
        self.logfiles.pop(label).close();del self.processes[label]

    def pump(self,duration=.02):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            if time.monotonic()>self.deadline:raise TimeoutError('P4 wall watchdog')
            if self.sim_time>1500:raise TimeoutError('P4 simulated-time watchdog')
            for label,proc in self.processes.items():
                if label not in self.allowed_exit and proc.poll() is not None:raise RuntimeError(f'{label} exited unexpectedly: {proc.returncode}')
            # Drain the odometry reader separately so busy bag/image subscriptions
            # cannot starve the independent physical-stop observer.
            for _ in range(16):
                observations = self.counts['/odom']
                self.odom_executor.spin_once(timeout_sec=0.)
                if self.counts['/odom'] == observations:
                    break
            self.executor.spin_once(timeout_sec=min(.01,max(0,end-time.monotonic())))
            self.process_depth_evidence()
            self.process_costmap_snapshots()
            if self.safety_violation and not self.stopping:
                self.stopping=True
                try:self.enable(False)
                finally:raise AssertionError('Independent observer stopped run: '+str(self.safety_violation))

    def until(self,predicate,timeout,reason):
        end=time.monotonic()+timeout;note=time.monotonic()+20
        while not predicate():
            if time.monotonic()>=end:raise TimeoutError(reason)
            if time.monotonic()>=note:
                print(f'{self.stage}: waiting {reason}; t={self.sim_time:.2f}; guard={self.guard_state}',flush=True);note+=20
            self.pump()

    def wait_sim(self,duration):
        start=self.sim_time;self.until(lambda:self.sim_time>=start+duration,max(10,duration*6),f'{duration}s simulation interval')

    def call(self,client,request,timeout=5):
        self.until(client.service_is_ready,timeout,'service '+client.srv_name)
        future=client.call_async(request);self.until(future.done,timeout,'response '+client.srv_name)
        if future.exception():raise future.exception()
        return future.result()

    def receive(self,topic,msg):
        wall=time.monotonic();self.counts[topic]+=1
        self.samples[topic].append((wall,msg))
        if topic=='/clock':
            stamp=stamp_s(msg.clock)
            if stamp<self.sim_time:self.clock_reset=True
            self.sim_time=stamp
        if hasattr(msg,'header'):
            stamp=stamp_s(msg.header.stamp)
            self.timing[topic].append([self.sim_time,wall-self.start_wall,stamp,self.sim_time-stamp,msg.header.frame_id])
        if topic=='/navigation/state':
            self.guard_state=json.loads(msg.data);self.states.append({**self.now(),**self.guard_state})
        if topic=='/tf':
            for tf in msg.transforms:
                edge=(tf.header.frame_id.lstrip('/'),tf.child_frame_id.lstrip('/'))
                self.tf_edges.setdefault(edge,deque(maxlen=1000)).append((wall,tf))
                if edge in (('map','odom'),('odom','base_footprint')):
                    tr=tf.transform;self.tf_observations.append({**self.now(),'edge':list(edge),'stamp':stamp_s(tf.header.stamp),
                        'pose':[tr.translation.x,tr.translation.y,yaw(tr.rotation)]})
        if topic=='/sim/ground_truth':self.observe_truth(msg)
        if topic=='/odom':
            p=msg.pose.pose;self.odom_observations.append({**self.now(),'stamp':stamp_s(msg.header.stamp),
                'frame':msg.header.frame_id,'child':msg.child_frame_id,'pose':[p.position.x,p.position.y,yaw(p.orientation)],
                'velocity':[msg.twist.twist.linear.x,msg.twist.twist.angular.z]})
        if topic=='/behavior_tree_log':
            self.bt_events.extend({**self.now(),'stamp':stamp_s(e.timestamp),'name':e.node_name,'uid':e.uid,
                'previous':e.previous_status,'current':e.current_status} for e in msg.event_log)
        if topic in ('/cmd_vel/nav','/cmd_vel/requested','/cmd_vel/smoothed','/cmd_vel/monitored','/cmd_vel','/sim/cmd_vel'):
            twist=msg.twist if hasattr(msg,'twist') else msg
            self.command_events.append({**self.now(),'topic':topic,'stamp':stamp_s(msg.header.stamp) if hasattr(msg,'header') else None,
                'frame':msg.header.frame_id if hasattr(msg,'header') else None,
                'velocity':[twist.linear.x,twist.linear.y,twist.linear.z,twist.angular.x,twist.angular.y,twist.angular.z]})
        if topic=='/perception/memory':self.observe_memory(msg)
        if topic=='/perception/obstacles':self.observe_cloud(msg)
        if topic.endswith('/costmap_raw'):self.observe_costmap(topic,msg)
        if topic=='/plan' and msg.poses:self.observe_plan(msg)
        if topic=='/map':
            self.map_events.append({**self.now(),'stamp':stamp_s(msg.header.stamp),'width':msg.info.width,'height':msg.info.height,
                                    'sha256':hashlib.sha256(np.asarray(msg.data,dtype=np.int8).tobytes()).hexdigest()})
        if topic=='/camera/depth/image_rect_raw':self.depth_pending.append(msg)
        if topic in ('/scan','/imu/raw','/imu/data','/camera/depth/image_rect_raw'):
            self.observe_sensor(topic,msg)
        if topic=='/rosout' and msg.level>=30:
            self.ros_errors.append({**self.now(),'stamp':stamp_s(msg.stamp),'name':msg.name,'level':msg.level,'message':msg.msg})
            planner_failure=msg.name=='planner_server' and 'failed to plan' in msg.msg
            controller_failure=msg.name=='controller_server' and 'No valid trajectories' in msg.msg
            if planner_failure or (controller_failure and self.sim_time-self.last_controller_failure_snapshot>=1.):
                if controller_failure:self.last_controller_failure_snapshot=self.sim_time
                for name,client in self.costmap_clients.items():
                    if client.service_is_ready():
                        self.costmap_pending.append((client.call_async(self.GetCostmap.Request()),
                            {'requested':self.now(),'service':name,'failure_source':msg.name,'failure_message':msg.msg}))
        if self.writer is not None and (topic!='/camera/depth/image_rect_raw' or self.stage in self.depth_record_stages):
            from rclpy.serialization import serialize_message
            self.writer.write(topic,serialize_message(msg),int(max(self.sim_time,0)*1e9))

    def initialize(self):
        import rclpy
        import rosbag2_py
        from rclpy.node import Node
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
        from rclpy.action import ActionClient
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan,Imu,Image,CameraInfo,PointCloud2
        from geometry_msgs.msg import Twist,TwistStamped
        from nav_msgs.msg import Odometry,OccupancyGrid,Path as RosPath
        from nav2_msgs.msg import Costmap,CollisionMonitorState,BehaviorTreeLog
        from nav2_msgs.action import NavigateToPose
        from nav2_msgs.srv import GetCostmap
        from action_msgs.msg import GoalStatusArray
        from diagnostic_msgs.msg import DiagnosticArray
        from rcl_interfaces.msg import Log
        from rcl_interfaces.srv import GetParameters,ListParameters
        from lifecycle_msgs.srv import GetState
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer,TransformListener
        from std_msgs.msg import String
        from std_srvs.srv import SetBool
        from omm_perception_msgs.msg import ObstacleMemory,PerceptionHealth
        from visualization_msgs.msg import MarkerArray
        from gz.transport13 import Node as GzNode
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from ament_index_python.packages import get_package_prefix,get_package_share_directory
        self.rclpy=rclpy;self.GetParameters=GetParameters;self.ListParameters=ListParameters;self.GetState=GetState;self.SetBool=SetBool;self.NavigateToPose=NavigateToPose;self.GetCostmap=GetCostmap
        self.context=Context();rclpy.init(context=self.context)
        self.node=Node('p4_acceptance_observer',context=self.context,parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.odom_node = Node('p4_odometry_observer', context=self.context)
        self.odom_executor = SingleThreadedExecutor(context=self.context)
        self.odom_executor.add_node(self.odom_node)
        self.pump(.5)
        self.e.detail('isolated_ros_graph',self.node.get_node_names()==[self.node.get_name()],nodes=self.node.get_node_names())
        self.buffer=Buffer(node=self.node);self.listener=TransformListener(self.buffer,self.node)
        transient=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        topics={'/clock':Clock,'/tf':TFMessage,'/tf_static':TFMessage,'/odom':Odometry,'/wheel/odometry':Odometry,
                '/sim/ground_truth':Odometry,'/scan':LaserScan,'/imu/raw':Imu,'/imu/data':Imu,
                '/camera/depth/image_rect_raw':Image,'/camera/depth/camera_info':CameraInfo,
                '/diagnostics':DiagnosticArray,'/rosout':Log,'/sim/cmd_vel':Twist,
                '/cmd_vel/nav':TwistStamped,'/cmd_vel/requested':TwistStamped,'/cmd_vel/smoothed':TwistStamped,
                '/cmd_vel/monitored':TwistStamped,'/cmd_vel':TwistStamped,'/navigation/state':String,
                '/perception/health':PerceptionHealth,'/perception/memory':ObstacleMemory,'/perception/obstacles':PointCloud2,
                '/map':OccupancyGrid,'/global_costmap/costmap_raw':Costmap,'/local_costmap/costmap_raw':Costmap,
                '/unsmoothed_plan':RosPath,'/plan':RosPath,'/collision_monitor_state':CollisionMonitorState,
                '/behavior_tree_log':BehaviorTreeLog,
                '/navigate_to_pose/_action/status':GoalStatusArray,'/slam_toolbox/graph_visualization':MarkerArray}
        transient_topics={'/tf_static','/map','/perception/memory','/perception/health','/navigation/state',
                          '/global_costmap/costmap_raw','/local_costmap/costmap_raw','/navigate_to_pose/_action/status'}
        self.writer=rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(self.e.directory/'rosbag'),storage_id='sqlite3'),
                         rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr'))
        for topic_id,(topic,cls) in enumerate(topics.items(),1):
            self.samples[topic]=deque(maxlen=3 if cls in (Image,Costmap,OccupancyGrid,ObstacleMemory,MarkerArray) else 2000)
            type_name=cls.__module__.split('.')[0]+'/msg/'+cls.__name__
            self.writer.create_topic(rosbag2_py.TopicMetadata(id=topic_id,name=topic,type=type_name,serialization_format='cdr'))
            qos=transient if topic in transient_topics else qos_profile_sensor_data
            if topic == '/odom':
                # Preserve the 50 Hz measurement window while camera/rosbag callbacks run.
                qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            reader = self.odom_node if topic == '/odom' else self.node
            reader.create_subscription(cls,topic,partial(self.receive,topic),qos)
        self.enable_client=self.node.create_client(SetBool,'/navigation/enable')
        self.fault_clients={k:self.node.create_client(SetBool,'/simulation/fault/'+k) for k in ('scan','depth')}
        self.action=ActionClient(self.node,NavigateToPose,'/navigate_to_pose')
        self.gz=GzNode();self.gz_stats=[]
        self.gz.subscribe(WorldStatistics,'/world/p4_course/stats',lambda msg:self.gz_stats.append([time.monotonic()-self.start_wall,stamp_s(msg.sim_time),msg.real_time_factor]))
        self.start('simulator',['ros2','launch','omslam_sim','sim.launch.py','world:=p4_course','gui:=false','pedestrians:=false',
                                'fault_relay:=true','x:=0','y:=0','yaw:=0'])
        self.start_navigation('slam')
        required=('/clock','/scan','/imu/raw','/imu/data','/camera/depth/image_rect_raw','/camera/depth/camera_info',
                  '/sim/ground_truth','/wheel/odometry','/odom','/perception/health','/perception/memory','/map')
        self.until(lambda:all(self.counts[t]>=2 for t in required),100,'all required sensors and observation streams')
        self.until(lambda:self.guard_state.get('ready') and self.action.server_is_ready(),35,'guard and Nav2 ready')
        from dwb_msgs.msg import LocalPlanEvaluation
        evaluation_topics=[name for name,types in self.node.get_topic_names_and_types() if 'dwb_msgs/msg/LocalPlanEvaluation' in types]
        for topic in evaluation_topics:
            self.node.create_subscription(LocalPlanEvaluation,topic,self.observe_dwb,qos_profile_sensor_data)
        self.event('controller_diagnostic_topics',topics=evaluation_topics)
        for name,types in self.node.get_service_names_and_types():
            if 'nav2_msgs/srv/GetCostmap' in types:
                self.costmap_clients[name]=self.node.create_client(GetCostmap,name)
        self.world_to_map=compose(self.map_pose(),inverse(np.array(self.course['spawn'])))
        self.initial_odom=self.pose('/odom')
        self.event('fixed_initial_alignment',world_to_map=self.world_to_map,source='first stationary SLAM pose and known spawn, fixed for entire run')
        self.stationary('before_enable',.5)
        self.e.detail('I0.base_inactive',self.guard_state.get('base_state')==2,guard=self.guard_state)
        self.parameters_and_graph()
        self.enable(True)

    def start_navigation(self,mode,map_path=None,seed=None):
        self.mode=mode
        command=['ros2','launch','omslam_navigation','slam_navigation.launch.py' if mode=='slam' else 'navigation.launch.py',
                 'start_sim:=false','world:=p4_course','gui:=false','pedestrians:=false','fault_relay:=true']
        if map_path:
            command+=['map:='+str(map_path),'initial_pose:=true','initial_x:='+str(seed[0]),'initial_y:='+str(seed[1]),'initial_yaw:='+str(seed[2])]
        self.start('navigation',command)

    def enable(self,value):
        response=self.call(self.enable_client,self.SetBool.Request(data=value),timeout=8)
        self.event('enable_response',requested=value,success=response.success,message=response.message)
        if not response.success:raise RuntimeError('navigation enable rejected: '+response.message)
        if value:self.until(lambda:self.guard_state.get('state')=='ARMED_IDLE',5,'armed idle status')
        return response

    def map_pose(self):
        from rclpy.time import Time
        tf=self.buffer.lookup_transform('map','base_footprint',Time()).transform
        return np.array([tf.translation.x,tf.translation.y,yaw(tf.rotation)])

    def pose(self,topic='/sim/ground_truth'):
        pose=self.samples[topic][-1][1].pose.pose
        return np.array([pose.position.x,pose.position.y,yaw(pose.orientation)])

    def sensor_stamp(self,topic):return stamp_s(self.samples[topic][-1][1].header.stamp)

    def stationary(self,label,duration=.3):
        begin = self.sim_time
        initial = self.pose().copy()
        window = self.measure_stop(label, begin, duration, max(1., duration + .7))
        distance = float(np.linalg.norm(self.pose()[:2] - initial[:2]))
        self.e.detail('stationary_'+label, not window.motion_observed and distance <= .01,
                      duration=duration, distance_m=distance, samples=window.count)

    def measure_stop(self, label, start, duration, timeout):
        from omslam_navigation.measured_stop import MeasuredStop
        window = MeasuredStop(round(start * 1e9), duration)
        last_receipt = -math.inf
        deadline = time.monotonic() + max(6., timeout * 6.)
        while True:
            for wall, msg in self.samples['/odom']:
                if wall > last_receipt:
                    window.observe(msg, wall)
                    last_receipt = wall
                    if (window.last_ns - window.after_ns <= round(timeout * 1e9)
                            and window.confirmed(round(self.sim_time * 1e9), time.monotonic())):
                        return window
            # DDS can deliver an on-time odometry sample after the clock advances.
            # Only stamps within the original window can qualify above.
            if (window.last_ns - window.after_ns > round(timeout * 1e9)
                    or self.sim_time - start > timeout + .25 or time.monotonic() > deadline):
                raise AssertionError(f'{label}: no fresh measured stop within {timeout:g} seconds')
            self.pump()

    def wait_stop(self, label, start=None):
        start = self.sim_time if start is None else start
        window = self.measure_stop(label, start, .3, 1.)
        settled = window.last_ns * 1e-9
        entry = {'label': label, 'trigger_time': start, 'settled_time': settled,
                 'duration_s': settled-start, 'samples': window.count,
                 'first_stop_stamp_ns': window.first_ns, 'last_stop_stamp_ns': window.last_ns}
        self.metrics['stops'].append(entry)
        self.event('measured_stop', **entry)
        return entry

    def parameter(self,node,names):
        client=self.node.create_client(self.GetParameters,'/'+node+'/get_parameters')
        reply=self.call(client,self.GetParameters.Request(names=names))
        self.node.destroy_client(client)
        def convert(value):
            return {1:value.bool_value,2:value.integer_value,3:value.double_value,4:value.string_value,
                    5:list(value.byte_array_value),6:list(value.bool_array_value),7:list(value.integer_array_value),
                    8:list(value.double_array_value),9:list(value.string_array_value)}.get(value.type)
        return dict(zip(names,map(convert,reply.values)))

    def parameters_and_graph(self):
        from ament_index_python.packages import get_package_prefix,get_package_share_directory
        from rclpy.qos import qos_check_compatible,QoSCompatibility
        expected={'planner_server':{'GridBased.plugin':'nav2_smac_planner::SmacPlannerLattice'},
                  'controller_server':{'FollowPath.plugin':'dwb_core::DWBLocalPlanner','enable_stamped_cmd_vel':True,
                                       'goal_checker.plugin':'nav2_controller::StoppedGoalChecker',
                                       'goal_checker.trans_stopped_velocity':.01,'goal_checker.rot_stopped_velocity':.02},
                  'velocity_smoother':{'enable_stamped_cmd_vel':True,'stamp_smoothed_velocity_with_smoothing_time':False},
                  'collision_monitor':{'enable_stamped_cmd_vel':True}}
        observed={node:self.parameter(node,list(values)) for node,values in expected.items()}
        self.e.detail('I0.actual_plugin_parameters',all(observed[node]==values for node,values in expected.items()),parameters=observed)
        self.drive_geometry = self.parameter('sim_base', ['left_radius', 'right_radius', 'wheel_separation', 'max_rpm'])
        self.drive_limits = self.parameter('navigation_guard', ['max_linear_velocity', 'max_angular_velocity'])
        self.e.detail('I0.drive_limits', all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0
                      for v in (*self.drive_geometry.values(), *self.drive_limits.values())),
                      geometry=self.drive_geometry, limits=self.drive_limits)
        dumps={}
        for node in ('planner_server','controller_server','velocity_smoother','collision_monitor','bt_navigator',
                     'global_costmap/global_costmap','local_costmap/local_costmap','navigation_guard','obstacle_memory'):
            client=self.node.create_client(self.ListParameters,'/'+node+'/list_parameters')
            if not client.service_is_ready():
                self.pump(.1)
            if client.service_is_ready():
                names=self.call(client,self.ListParameters.Request(depth=100)).result.names
                dumps[node]=self.parameter(node,list(names))
            self.node.destroy_client(client)
        (self.e.directory/'parameters.json').write_text(json.dumps(dumps,indent=2)+'\n')
        endpoints={};incompat=[]
        for topic in ('/tf','/tf_static','/scan','/imu/data','/camera/depth/image_rect_raw','/perception/health','/perception/memory','/cmd_vel','/sim/ground_truth'):
            pubs=self.node.get_publishers_info_by_topic(topic);subs=self.node.get_subscriptions_info_by_topic(topic)
            endpoints[topic]={'publishers':[p.node_name for p in pubs],'subscribers':[s.node_name for s in subs],
                'publisher_endpoints':[{'node':p.node_name,'namespace':p.node_namespace,'gid':bytes(p.endpoint_gid).hex()} for p in pubs]}
            for p in pubs:
                for s in subs:
                    if qos_check_compatible(p.qos_profile,s.qos_profile)[0]==QoSCompatibility.ERROR:
                        incompat.append([topic,p.node_name,s.node_name])
        self.e.detail('I0.qos_and_command_ownership',not incompat and endpoints['/cmd_vel']['publishers']==['navigation_guard']
                      and endpoints['/sim/ground_truth']['subscribers']==[self.node.get_name()],endpoints=endpoints,incompatible=incompat)
        self.authority('slam_toolbox' if self.mode=='slam' else 'amcl')
        names=('omslam_navigation','omslam_navigation_contracts','nav2_smac_planner','omslam_sim','omslam_description','omslam_base',
               'omslam_localization','omslam_mapping','omm_perception','omm_perception_msgs','omm_costmap_layers')
        prefixes={name:str(Path(get_package_prefix(name)).resolve()) for name in names}
        planner_processes=[]
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():continue
            try:
                if os.getpgid(int(process.name))!=self.processes['navigation'].pid:continue
                executable=(process/'exe').resolve()
                if executable.name!='planner_server':continue
                libraries=sorted({line.split()[-1] for line in (process/'maps').read_text().splitlines()
                                  if 'libnav2_smac_planner' in line and line.split()[-1].startswith('/')})
                planner_processes.append({'pid':int(process.name),'executable':str(executable),'smac_libraries':libraries})
            except (ProcessLookupError,PermissionError,FileNotFoundError):continue
        expected_library=WORKSPACE/'install/nav2_smac_planner/lib/libnav2_smac_planner_lattice.so'
        self.e.detail('I0.loaded_smac_library',len(planner_processes)==1
                      and str(expected_library) in planner_processes[0]['smac_libraries']
                      and all(Path(p).is_relative_to(WORKSPACE/'install/nav2_smac_planner')
                              for p in planner_processes[0]['smac_libraries']),processes=planner_processes,
                      expected_library=str(expected_library),sha256=digest(expected_library))
        files={str(p):digest(p) for name in names for p in Path(prefixes[name]).rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.xml','.json','.so','.sdf')}
        manifest={'source_sha256':package_source_hashes(),'installed_sha256':files,'prefixes':prefixes,
                  'planner_processes':planner_processes,
                  'build_manifest':str(self.build_manifest),'course_sha256':digest(WORKSPACE/'tools/p4_course.json'),
                  'domain':os.environ['ROS_DOMAIN_ID'],'gz_partition':os.environ['GZ_PARTITION'],'verifier_sha256':digest(__file__)}
        (self.e.directory/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        nav=Path(get_package_share_directory('omslam_navigation'));lattice=nav/'config/lattice/diff_5cm_16h_r0p5.json'
        meta=json.loads(lattice.read_text())['lattice_metadata']
        self.e.check('V4-01',all(Path(prefixes[n])==WORKSPACE/'install'/n for n in names)
                     and digest(lattice)=='105b37351d15e340cc2921fa8878c8ea727a65cd7e89c39c977be0654be3b2a9'
                     and meta['motion_model']=='diff' and meta['grid_resolution']==.05 and meta['num_of_headings']==16,
                     parameters_file='parameters.json',lattice_metadata=meta)

    def authority(self,expected):
        nodes=self.node.get_node_names();backends=[name for name in nodes if name in ('slam_toolbox','amcl')]
        pubs=self.node.get_publishers_info_by_topic('/tf');static=self.node.get_publishers_info_by_topic('/tf_static')
        graph_nodes=self.node.get_node_names_and_namespaces()
        actual=sorted((p.node_namespace,p.node_name) for p in pubs);static_actual=[(p.node_namespace,p.node_name) for p in static]
        required=sorted([('/','ekf_filter_node'),('/','robot_state_publisher'),('/',expected)])
        self.e.detail('authority_'+self.stage,backends==[expected] and ('map','odom') in self.tf_edges and sorted(set(actual))==required
                      and all(graph_nodes.count((name,namespace))==1 for namespace,name in required)
                      and static_actual==[('/','robot_state_publisher')],backends=backends,tf_publishers=actual,static_tf_publishers=static_actual,
                      evidence_type='Configured unique node graph and allowed endpoint owners; slam_toolbox has two TF endpoints in one node. Jazzy rclpy does not expose sender GID per TF message')

    def observe_truth(self,msg):
        pose=msg.pose.pose;world=np.array([pose.position.x,pose.position.y,yaw(pose.orientation)])
        stamp=stamp_s(msg.header.stamp)
        footprint=np.asarray(self.course['footprint'])
        obstacles=[(o['name'],rectangle(o['bounds'])) for o in self.course['static_boxes']]
        obstacles += [(name,rectangle(object_bounds(self.course['objects'][name],center))) for name,center in self.active_objects.items()]
        points=interpolate(self.last_truth[1],world) if self.last_truth else [world]
        for point in points:
            shape=polygon(point,footprint)
            for name,other in obstacles:
                distance=separation(shape,other)
                self.minimum_clearance=min(self.minimum_clearance,distance)
                if distance<=1e-8:
                    self.collision.append({**self.now(),'obstacle':name,'pose':point.tolist()})
                if distance<.03 and self.guard_state.get('state')=='RUNNING':
                    self.safety_violation={'reason':'truth footprint clearance below 0.03 m','obstacle':name,'distance':distance,'pose':point.tolist()}
        self.last_truth=(stamp,world)
        try:estimate=self.map_pose().tolist()
        except Exception:estimate=None
        self.trajectory.append({**self.now(),'stamp':stamp,'world':world.tolist(),
                                'truth_map':compose(self.world_to_map,world).tolist() if self.world_to_map is not None else None,
                                'estimate':estimate,'velocity':[msg.twist.twist.linear.x,msg.twist.twist.angular.z]})

    def observation_points(self,msg):
        from rclpy.time import Time
        from scipy.spatial.transform import Rotation
        from sensor_msgs_py import point_cloud2
        points=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True)
        tf=self.buffer.lookup_transform('odom',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
        q=tf.rotation;rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        return points@rotation.T+np.array([tf.translation.x,tf.translation.y,tf.translation.z])

    def points_near(self,points,name):
        return len(self.near_points(points,name))

    def near_points(self,points,name):
        if not len(points):return np.empty((0,3))
        target=np.array(self.course['objects'][name]['center'])
        if self.initial_odom is not None:target=compose(self.initial_odom,np.array([*target,0]))[:2]
        values=np.asarray(points)
        return values[(np.linalg.norm(values[:,:2]-target,axis=1)<.28)
                      &(values[:,2]>.015)&(values[:,2]<.25)]

    def observe_cloud(self,msg):
        try:
            points=self.observation_points(msg)
            record={**self.now(),'stamp':stamp_s(msg.header.stamp),'points':len(points),
                    'low_a':self.points_near(points,'low_a'),'low_b':self.points_near(points,'low_b')}
        except Exception as exc:
            record={**self.now(),'stamp':stamp_s(msg.header.stamp),'tf_error':str(exc)}
        self.cloud_events.append(record)

    def observe_memory(self,msg):
        points=[[p.x,p.y,p.z] for p in list(msg.points)+list(msg.archived_points)]
        record={**self.now(),'stamp':stamp_s(msg.header.stamp),'observation_stamp':stamp_s(msg.observation_stamp),
                'sensor':msg.sensor_healthy,'geometry':msg.geometry_healthy,'memory':msg.memory_healthy,
                'low_a':self.points_near(points,'low_a'),'low_b':self.points_near(points,'low_b'),
                'low_b_points':self.near_points(points,'low_b').tolist(),
                'cleared':msg.cleared_voxels_total,'outside':msg.outside_fov_voxels,'visible':msg.visible_voxels,
                'status':msg.status}
        self.memory_events.append(record)

    def observe_costmap(self,topic,msg):
        from rclpy.time import Time
        from scipy.spatial.transform import Rotation
        record={**self.now(),'topic':topic,'stamp':stamp_s(msg.header.stamp),'frame':msg.header.frame_id,'objects':{}}
        grid=np.asarray(msg.data).reshape(msg.metadata.size_y,msg.metadata.size_x)
        if msg.header.frame_id=='map' and self.memory_events and self.memory_events[-1]['low_b_points']:
            try:
                tf=self.buffer.lookup_transform('map','odom',Time()).transform
                transform=[tf.translation.x,tf.translation.y,yaw(tf.rotation)]
                points=self.memory_events[-1]['low_b_points']
                projected=[compose(transform,[p[0],p[1],0.])[:2] for p in points]
                cells=[(math.floor((p[0]-msg.metadata.origin.position.x)/msg.metadata.resolution),
                        math.floor((p[1]-msg.metadata.origin.position.y)/msg.metadata.resolution)) for p in projected]
                costs=[int(grid[y,x]) for x,y in cells if 0<=x<msg.metadata.size_x and 0<=y<msg.metadata.size_y]
                record['memory_projection']={'map_from_odom':transform,'memory_stamp':self.memory_events[-1]['stamp'],
                    'observation_stamp':self.memory_events[-1]['observation_stamp'],'points':len(points),'in_bounds':len(costs),
                    'lethal_fraction':sum(v==254 for v in costs)/len(costs) if costs else 0.,
                    'odom_centroid':np.mean(points,axis=0).tolist(),'map_centroid':np.mean(projected,axis=0).tolist()}
            except Exception as exc:record['projection_error']=str(exc)
        for name in ('low_a','low_b','gate','crossing'):
            if name=='crossing' and name not in self.active_objects:continue
            center=self.active_objects.get(name,self.course['objects'][name]['center'])
            world=np.array([*center,0.])
            if self.world_to_map is None:continue
            location=compose(self.world_to_map,world)
            if msg.header.frame_id=='odom':
                try:
                    tf=self.buffer.lookup_transform('odom','map',Time()).transform
                    location=compose([tf.translation.x,tf.translation.y,yaw(tf.rotation)],location)
                except Exception:continue
            x=int((location[0]-msg.metadata.origin.position.x)/msg.metadata.resolution)
            y=int((location[1]-msg.metadata.origin.position.y)/msg.metadata.resolution)
            radius=math.ceil(.20/msg.metadata.resolution)
            if x-radius>=0 and x+radius<msg.metadata.size_x and y-radius>=0 and y+radius<msg.metadata.size_y:
                roi=grid[y-radius:y+radius+1,x-radius:x+radius+1]
                record['objects'][name]={'lethal':int(np.sum(roi==254)),'center':int(grid[y,x]),'max':int(roi.max()),'world_center':list(center)}
        self.costmap_events.append(record)

    def observe_plan(self,msg):
        data={'time':self.sim_time,'stage':self.stage,'stamp':stamp_s(msg.header.stamp),'frame':msg.header.frame_id,
              'poses':[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in msg.poses]}
        self.paths.append(data)

    def observe_dwb(self,msg):
        if self.sim_time-self.last_evaluation<.5 or not msg.twists:return
        self.last_evaluation=self.sim_time
        def scored(item):
            return {'velocity':[item.traj.velocity.x,item.traj.velocity.y,item.traj.velocity.theta],
                    'total':item.total,'critics':{s.name:{'raw':s.raw_score,'scale':s.scale} for s in item.scores},
                    'first_pose':[item.traj.poses[0].x,item.traj.poses[0].y,item.traj.poses[0].theta] if item.traj.poses else None,
                    'last_pose':[item.traj.poses[-1].x,item.traj.poses[-1].y,item.traj.poses[-1].theta] if item.traj.poses else None}
        valid=sorted((item for item in msg.twists if item.total>=0),key=lambda item:item.total)
        rejected=defaultdict(int)
        for item in msg.twists:
            if item.total<0:
                for score in item.scores:
                    if score.raw_score<0:rejected[score.name]+=1
        self.dwb_evaluations.append({**self.now(),'stamp':stamp_s(msg.header.stamp),'best':scored(msg.twists[msg.best_index]),
            'best_forward':[scored(item) for item in valid if item.traj.velocity.x>.01][:5],
            'best_zero':[scored(item) for item in valid if abs(item.traj.velocity.x)<1e-6][:3],
            'rejected_by_critic':dict(rejected),
            'straight_candidates':[scored(item) for item in msg.twists if item.traj.velocity.x>.01 and abs(item.traj.velocity.theta)<1e-9],
            'valid_candidates':len(valid),'total_candidates':len(msg.twists)})

    def process_costmap_snapshots(self):
        for future,record in list(self.costmap_pending):
            if not future.done():continue
            self.costmap_pending.remove((future,record))
            record['received']=self.now()
            try:
                msg=future.result().map;meta=msg.metadata
                directory=self.e.directory/'failure_costmaps';directory.mkdir(exist_ok=True)
                label=f"{len(self.costmap_snapshots):03d}"+record['service'].replace('/','_')
                filename=directory/(label+'.npz')
                np.savez_compressed(filename,grid=np.asarray(msg.data,dtype=np.uint8).reshape(meta.size_y,meta.size_x))
                record.update(file=str(filename),stamp=stamp_s(msg.header.stamp),frame=msg.header.frame_id,
                    resolution=meta.resolution,origin=[meta.origin.position.x,meta.origin.position.y,yaw(meta.origin.orientation)],
                    size=[meta.size_x,meta.size_y])
                record.update(map_pose=self.map_pose().tolist(),odom_pose=self.pose('/odom').tolist(),truth_pose=self.pose().tolist())
            except Exception as exc:record['error']=str(exc)
            self.costmap_snapshots.append(record)
            self.event('navigation_failure_costmap_snapshot',**record)

    def observe_sensor(self,topic,msg):
        stamp=stamp_s(msg.header.stamp);previous=self.last_sensor_stamps.get(topic)
        record={**self.now(),'topic':topic,'stamp':stamp,'age':self.sim_time-stamp,
                'frame':msg.header.frame_id,'gap':stamp-previous if previous is not None else None}
        self.last_sensor_stamps[topic]=stamp
        if topic=='/scan':
            ranges=np.asarray(msg.ranges)
            record.update(valid_count=int(np.sum(np.isfinite(ranges)&(ranges>=msg.range_min)&(ranges<=msg.range_max))),total=len(ranges))
        elif topic in ('/imu/raw','/imu/data'):
            q=msg.orientation;record.update(quaternion_norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w),
                orientation_valid=msg.orientation_covariance[0]>=0,wz=msg.angular_velocity.z)
            try:
                from rclpy.time import Time
                from scipy.spatial.transform import Rotation
                q=self.buffer.lookup_transform('base_footprint',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform.rotation
                vector=msg.angular_velocity
                record['base_wz']=float(Rotation.from_quat([q.x,q.y,q.z,q.w]).apply([vector.x,vector.y,vector.z])[2])
            except Exception as exc:record['base_transform_error']=str(exc)
        else:
            dtype=('<' if not msg.is_bigendian else '>')+('f4' if msg.encoding=='32FC1' else 'u2')
            data=np.frombuffer(msg.data,dtype=dtype)
            record.update(encoding=msg.encoding,width=msg.width,height=msg.height,valid_fraction=float(np.mean(np.isfinite(data)&(data>0))))
        self.sensor_observations.append(record)


    def process_depth_evidence(self):
        if not self.depth_pending or self.initial_odom is None or not self.samples.get('/camera/depth/camera_info'):
            return
        from rclpy.time import Time
        from scipy.spatial.transform import Rotation
        msg=self.depth_pending[0]
        if self.sim_time-stamp_s(msg.header.stamp)>.3:
            self.depth_pending.popleft();return
        try:tf=self.buffer.lookup_transform('camera_optical_frame','odom',Time.from_msg(msg.header.stamp)).transform
        except Exception:return
        self.depth_pending.popleft()
        info=self.samples['/camera/depth/camera_info'][-1][1]
        dtype=('<' if not msg.is_bigendian else '>')+('f4' if msg.encoding=='32FC1' else 'u2')
        data=np.frombuffer(msg.data,dtype=dtype).reshape(msg.height,msg.step//np.dtype(dtype).itemsize)[:,:msg.width]
        q=tf.rotation;rot=Rotation.from_quat([q.x,q.y,q.z,q.w]);offset=np.array([tf.translation.x,tf.translation.y,tf.translation.z])
        evidence={**self.now(),'stamp':stamp_s(msg.header.stamp)}
        for name in ('low_a','low_b'):
            point=compose(self.initial_odom,np.array([*self.course['objects'][name]['center'],0]))
            point=rot.apply([point[0],point[1],.05])+offset
            value={'visible':False,'free':False}
            if .12<point[2]<5.:
                u=int(round(info.k[0]*point[0]/point[2]+info.k[2]));v=int(round(info.k[4]*point[1]/point[2]+info.k[5]))
                if 1<=u<msg.width-1 and 1<=v<msg.height-1:
                    observed=data[v-1:v+2,u-1:u+2].astype(float)*(1. if msg.encoding=='32FC1' else .001)
                    good=np.isfinite(observed)&(observed>.12)&(observed<5.)
                    value={'visible':True,'free':bool(np.all(good)&np.all(observed>point[2]+.10)),
                           'expected_depth':float(point[2]),'measured_min':float(np.min(observed[good])) if np.any(good) else None}
            evidence[name]=value
        self.depth_evidence.append(evidence)

    def grid_free(self,goal):
        if not self.samples['/map']:return False
        msg=self.samples['/map'][-1][1];res=msg.info.resolution
        x=int((goal[0]-msg.info.origin.position.x)/res);y=int((goal[1]-msg.info.origin.position.y)/res)
        return 0<=x<msg.info.width and 0<=y<msg.info.height and 0<=msg.data[y*msg.info.width+x]<50

    def blocked_goal_evidence(self,goal):
        msg=self.samples['/global_costmap/costmap_raw'][-1][1]
        grid=np.asarray(msg.data).reshape(msg.metadata.size_y,msg.metadata.size_x)
        origin=msg.metadata.origin.position;res=msg.metadata.resolution
        x,y=math.floor((goal[0]-origin.x)/res),math.floor((goal[1]-origin.y)/res)
        center=int(grid[y,x]) if 0<=x<msg.metadata.size_x and 0<=y<msg.metadata.size_y else 255
        shape=polygon(goal,self.course['footprint']);low=shape.min(axis=0);high=shape.max(axis=0);hits=[]
        for yy in range(max(0,math.floor((low[1]-origin.y)/res)),min(msg.metadata.size_y,math.ceil((high[1]-origin.y)/res)+1)):
            for xx in range(max(0,math.floor((low[0]-origin.x)/res)),min(msg.metadata.size_x,math.ceil((high[0]-origin.x)/res)+1)):
                if grid[yy,xx]==254 and intersects(shape,rectangle([origin.x+xx*res,origin.x+(xx+1)*res,origin.y+yy*res,origin.y+(yy+1)*res])):
                    hits.append([xx,yy])
        record={'goal':goal,'stamp':stamp_s(msg.header.stamp),'map_center_free':self.grid_free(goal),'center_cost':center,'lethal_footprint_cells':hits}
        self.e.detail('I5.center_free_footprint_blocked',record['map_center_free'] and center<254 and bool(hits),**record)

    def blocked_result(self,entry,marked=None,obstruction_epoch=None):
        metrics=bounded_planner_evidence(entry,self.bt_events,self.states,self.ros_errors,marked,obstruction_epoch)
        passed=metrics.pop('passed')
        self.e.detail(entry['label']+'.bounded_planner_failure',passed,**metrics)
        return metrics

    def send_goal(self,goal,label,require_free=True):
        if require_free:self.until(lambda:self.grid_free(goal),20,'observed free goal '+str(goal))
        self.until(self.action.server_is_ready,10,'NavigateToPose server')
        request=self.NavigateToPose.Goal();request.pose.header.frame_id='map'
        request.pose.header.stamp=self.node.get_clock().now().to_msg()
        request.pose.pose.position.x=float(goal[0]);request.pose.pose.position.y=float(goal[1])
        request.pose.pose.orientation.z=math.sin(goal[2]/2);request.pose.pose.orientation.w=math.cos(goal[2]/2)
        feedback=[]
        future=self.action.send_goal_async(request,feedback_callback=lambda msg:feedback.append({
            **self.now(),'distance':msg.feedback.distance_remaining,'recoveries':msg.feedback.number_of_recoveries}))
        self.until(future.done,8,'goal acceptance')
        handle=future.result()
        if not handle.accepted:raise AssertionError('NavigateToPose rejected '+label)
        ident=bytes(handle.goal_id.uuid).hex()
        entry={'id':ident,'label':label,'goal':list(goal),'start':self.now(),'feedback':feedback}
        self.goals.append(entry);self.current_goal=handle;self.current_future=handle.get_result_async();self.current_entry=entry
        self.event('goal_started',uuid=ident,target=goal,label=label)
        return self.current_future

    def finish_goal(self,expected=4,timeout=120,stop_trigger=None):
        future=self.current_future;entry=self.current_entry
        self.until(future.done,timeout,'goal result '+entry['label'])
        response=future.result();entry.update(status=response.status,error_code=response.result.error_code,error_msg=response.result.error_msg,end=self.now())
        self.event('goal_result',uuid=entry['id'],status=response.status,error_code=response.result.error_code,error_msg=response.result.error_msg)
        self.current_goal=None;self.current_future=None;self.current_entry=None
        self.e.detail('goal_'+entry['label'],response.status==expected,uuid=entry['id'],action_status=response.status,expected_action_status=expected,
                      error_code=response.result.error_code,error_msg=response.result.error_msg)
        self.wait_stop(entry['label'],stop_trigger)
        if expected==4:
            truth=compose(self.world_to_map,self.pose());goal=np.array(entry['goal'])
            entry['truth']=truth.tolist();entry['estimate']=self.map_pose().tolist()
            entry['position_error']=float(np.linalg.norm(truth[:2]-goal[:2]));entry['yaw_error']=abs(angle_delta(truth[2],goal[2]))
            self.e.detail('accuracy_'+entry['label'],entry['position_error']<=.15 and entry['yaw_error']<=.20,
                          position_m=entry['position_error'],yaw_rad=entry['yaw_error'],truth=entry['truth'],target=entry['goal'])
        self.until(lambda:self.guard_state.get('state') in ('ARMED_IDLE','FAULT_LATCHED','DISARMED','READY') and not self.guard_state.get('goal_id'),5,'goal terminal guard state')
        return entry

    def goto(self,goal,label):
        self.send_goal(goal,label);return self.finish_goal()

    def cancel(self):
        start=self.sim_time;handle=self.current_goal
        self.event('cancel_request', uuid=self.current_entry['id'])
        future=handle.cancel_goal_async();self.until(future.done,5,'cancel response')
        response=future.result();self.event('cancel_response',goals_canceling=len(response.goals_canceling))
        self.e.detail('cancel_accepted',bool(response.goals_canceling))
        entry=self.finish_goal(expected=5,timeout=10,stop_trigger=start)
        terminal=entry['end']['wall_time']
        late=late_cancel_commands(self.command_events,start,terminal)
        self.e.detail('cancel_no_late_nonzero',not late,late=late)
        return entry

    def in_region(self,region,min_speed=.1):
        if self.current_future is not None and self.current_future.done():
            response=self.current_future.result()
            self.current_entry.update(status=response.status,error_code=response.result.error_code,
                                      error_msg=response.result.error_msg,end=self.now())
            self.event('goal_ended_before_motion_event',uuid=self.current_entry['id'],status=response.status,
                       error_code=response.result.error_code,error_msg=response.result.error_msg)
            raise AssertionError('Goal ended before its motion event: '+str({k:self.current_entry[k] for k in ('label','status','error_code','error_msg')}))
        pose=self.pose('/odom');vx=self.samples['/odom'][-1][1].twist.twist.linear.x
        # Odom starts at the course origin; truth is deliberately not used to trigger controller actions.
        return region[0]<=pose[0]<=region[1] and region[2]<=pose[1]<=region[3] and vx>=min_speed

    def create_object(self,name):
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.boolean_pb2 import Boolean
        spec=self.course['objects'][name]
        request=EntityFactory(sdf=object_sdf(name,spec),name=name,allow_renaming=False)
        ok,response=self.gz.request('/world/p4_course/create',request,EntityFactory,Boolean,5000)
        if not ok or not response.data:raise RuntimeError('Gazebo object creation failed: '+name)
        self.active_objects[name]=list(spec['center']);self.object_intervals.append({'name':name,'created':self.sim_time,'removed':None});self.event('object_created',name=name,center=spec['center'])
        self.object_positions.append({'name':name,'time':self.sim_time,'center':list(spec['center'])})

    def remove_object(self,name):
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.boolean_pb2 import Boolean
        request=Entity(name=name,type=Entity.MODEL)
        ok,response=self.gz.request('/world/p4_course/remove',request,Entity,Boolean,5000)
        if not ok or not response.data:raise RuntimeError('Gazebo object removal failed: '+name)
        self.active_objects.pop(name,None)
        for interval in reversed(self.object_intervals):
            if interval['name']==name and interval['removed'] is None:interval['removed']=self.sim_time;break
        self.event('object_removed',name=name)

    def move_object(self,name,center):
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.boolean_pb2 import Boolean
        request=Pose(name=name);request.position.x=center[0];request.position.y=center[1]
        request.position.z=self.course['objects'][name]['size'][2]/2;request.orientation.w=1.
        ok,response=self.gz.request('/world/p4_course/set_pose',request,Pose,Boolean,1000)
        if not ok or not response.data:raise RuntimeError('Gazebo object move failed: '+name)
        self.active_objects[name]=list(center)
        self.object_positions.append({'name':name,'time':self.sim_time,'center':list(center)})

    def object_memory(self,name):
        return self.memory_events[-1].get(name,0) if self.memory_events else 0

    def outside_camera(self,name):
        from rclpy.time import Time
        from scipy.spatial.transform import Rotation
        msg=self.samples['/camera/depth/camera_info'][-1][1]
        tf=self.buffer.lookup_transform('camera_optical_frame','odom',Time()).transform
        center=compose(self.initial_odom,np.array([*self.course['objects'][name]['center'],0]))
        p=np.array([center[0],center[1],.05]);q=tf.rotation
        p=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(p)+np.array([tf.translation.x,tf.translation.y,tf.translation.z])
        if p[2]<=0:return True
        u,v=msg.k[0]*p[0]/p[2]+msg.k[2],msg.k[4]*p[1]/p[2]+msg.k[5]
        return not(0<=u<msg.width and 0<=v<msg.height and .12<=p[2]<=5.)

    def fault_event(self,name,goal,region,label,recover_goal=True):
        self.send_goal(goal,label)
        self.until(lambda:self.in_region(region),30,label+' trigger region and actual forward speed')
        old_memory=self.object_memory('low_b')
        self.e.detail(label+'_occupied_memory_precondition',old_memory>0,occupied_voxels=old_memory)
        last_stamp=self.sensor_stamp('/scan' if name=='scan' else '/camera/depth/image_rect_raw')
        start=self.sim_time;wall=time.monotonic()-self.start_wall
        self.call(self.fault_clients[name],self.SetBool.Request(data=True))
        self.event('sensor_fault_begin',sensor=name,last_stamp=last_stamp)
        self.until(lambda:self.guard_state.get('state')=='FAULT_LATCHED',5,'sensor fault latch')
        latch=self.sim_time
        self.until(lambda:any(x['topic']=='/cmd_vel' and x['sim_time']>=start and not any(x['velocity']) for x in self.command_events),2,'fault final zero')
        zero_event=next(x for x in self.command_events if x['topic']=='/cmd_vel' and x['sim_time']>=start and not any(x['velocity']))
        zero=zero_event['sim_time']
        sensor_topic='/scan' if name=='scan' else '/camera/depth/image_rect_raw'
        last_stamp=last_valid_sensor_stamp(self.sensor_observations,sensor_topic,last_stamp,zero_event)
        self.wait_stop(label,zero)
        self.until(lambda:self.current_future.done(),5,'fault goal terminal')
        response=self.current_future.result();self.current_entry.update(status=response.status,end=self.now(),error_code=response.result.error_code)
        self.current_goal=None;self.current_future=None;self.current_entry=None
        self.until(lambda:self.sim_time>=start+1.,5,'one-second sensor interruption')
        retained=self.object_memory('low_b')
        entry={'sensor':name,'begin':start,'last_sensor_stamp':last_stamp,'latch':latch,'zero':zero,
               'zero_age_s':zero-last_stamp,'terminal_status':response.status,'memory_before':old_memory,'memory_after':retained}
        self.metrics['faults'].append(entry)
        self.e.detail(label+'_fault_response',zero-last_stamp<=.40 and zero-latch<=.10 and response.status in (5,6)
                      and (not old_memory or retained>0),**entry)
        self.call(self.fault_clients[name],self.SetBool.Request(data=False));self.event('sensor_fault_end',sensor=name)
        self.until(lambda:self.guard_state.get('healthy'),5,'fresh healthy observations after '+name)
        self.wait_sim(1.05);self.stationary(label+'_recovery_no_restart',.3)
        self.e.detail(label+'_latch_remains',self.guard_state.get('state')=='FAULT_LATCHED',state=self.guard_state)
        if recover_goal:
            self.enable(True);self.stationary(label+'_enable_no_goal',1.)
            self.goto(goal,label+'_new_goal')

    def run_five_goals(self):
        self.initialize()
        targets=[[1,0,0],[5,0,math.pi/2],[5,4,math.pi/2],[0,4,-math.pi/2],[0,0,0]]
        for index,target in enumerate(targets,1):
            self.stage=f'G{index}'
            self.goto(target,self.stage)
        stops={s['label']:s for s in self.metrics['stops']}
        self.e.check('five_consecutive_goals',len(self.goals)==5 and all(g.get('status')==4 for g in self.goals)
                     and all(f'G{i}' in stops and stops[f'G{i}']['duration_s']<=1. for i in range(1,6))
                     and self.guard_state.get('fault_count')==0,one_execution=True,targets=targets,
                     goals=[{k:v for k,v in g.items() if k!='feedback'} for g in self.goals],stops=self.metrics['stops'])
        self.enable(False);self.stationary('normal_shutdown',.3)
        self.e.check('normal_shutdown',not self.guard_state.get('goal_id')
                     and self.guard_state.get('state') in ('READY','DISARMED'),guard=self.guard_state)
        self.final_checks({'V4-02','V4-03','V4-04'},physical_only=True,sensor_checks=False)

    def run(self,checks):
        wanted=set(checks)
        if self.e.baseline:
            baseline=json.loads(Path(self.e.baseline).read_text())
            if 'V4-06' not in wanted and baseline.get('checks',{}).get('V4-06',{}).get('status')=='PASS':
                self.reused_stages.add('I2')
            if baseline.get('details',{}).get('I5.bounded_planner_failure',{}).get('status')=='PASS':
                self.reused_stages.add('I5')
            if self.reused_stages:
                applicable=baseline.get('applicable_source_sha256',baseline.get('source_sha256'))
                self.e.detail('reused_completed_stages',applicable==package_source_hashes(),
                              stages=sorted(self.reused_stages),evidence=str(self.e.baseline))
        self.initialize()
        self.stage='I1';self.goto([1,0,0],'I1')
        self.stage='I2'
        if 'I2' not in self.reused_stages and wanted & {'V4-06','V4-05','V4-02','V4-03','V4-04','V4-11'}:
            self.send_goal([4,0,0],'I2');self.until(lambda:self.in_region([2,3,-.2,.2],0),30,'cancel in narrow corridor')
            cancel=self.cancel();self.metrics['cancel']=cancel
        self.stage='I3';self.goto([5,0,math.pi/2],'I3')
        self.stage='I4';self.goto([5,4,math.pi/2],'I4')
        if 'V4-08' in wanted:self.check_memory_marking('low_a')
        if 'I5' not in self.reused_stages and wanted & {'V4-05','V4-07'}:
            self.stage='I5'
            if not self.grid_free([9,1,0]):
                # Reveal the branch through its opening without a truth/map fixture override.
                self.goto([5,1,0],'I5_observe_entrance')
                self.goto([6.1,1,0],'I5_observe_branch')
            self.blocked_goal_evidence([9,1,0])
            self.send_goal([9,1,0],'I5');self.blocked_result(self.finish_goal(expected=6,timeout=30))
        if 'V4-07' in wanted:
            self.goto([5,4,0],'I6_approach')
            self.stage='I6';path_count=len(self.paths);self.send_goal([9,4,0],'I6')
            def fresh_path():
                if self.current_future.done():raise AssertionError('Gate preparation goal terminated before a valid path')
                return any(p['stamp']>=self.current_entry['start']['sim_time'] and
                           np.linalg.norm(np.subtract(p['poses'][-1][:2],[9,4]))<=.2 for p in self.paths[path_count:])
            self.until(fresh_path,12,'gate branch path')
            used=max((f['recoveries'] for f in self.current_entry['feedback']),default=0)
            if used:
                self.current_entry['role']='gate_path_preparation'
                previous=self.current_entry['id']
                self.event('gate_preparation_restarted',uuid=previous,used_recoveries=used)
                self.cancel()
                path_count=len(self.paths);self.send_goal([9,4,0],'I6')
                self.until(fresh_path,12,'fresh gate branch path after preparation')
                self.e.detail('I6.fresh_goal_uuid',self.current_entry['id']!=previous,
                              preparation_uuid=previous,test_uuid=self.current_entry['id'])
            used=max((f['recoveries'] for f in self.current_entry['feedback']),default=0)
            self.e.detail('I6.unused_recovery_budget',used==0,recoveries=used,uuid=self.current_entry['id'])
            self.e.detail('I6.gate_before_entry',self.map_pose()[0]<6,pose=self.map_pose())
            self.create_object('gate');created=self.sim_time;created_epoch=time.time()
            self.event('gate_recovery_boundary',created_sim_time=created,created_wall_epoch=created_epoch)
            self.until(lambda:any(e['sim_time']>=created and e['objects'].get('gate',{}).get('lethal',0)>0
                                  for e in self.costmap_events),5,'gate marked in costmap')
            marked=min(e['sim_time'] for e in self.costmap_events if e['sim_time']>=created and e['objects'].get('gate',{}).get('lethal',0)>0)
            failed=self.finish_goal(expected=6,timeout=30)
            duration=failed['end']['sim_time']-marked
            recoveries=max((f['recoveries'] for f in failed['feedback']),default=0)
            recovery=self.blocked_result(failed,marked,created_epoch)
            self.e.check('V4-07',duration<=30 and recoveries<=1,marked=marked,aborted=failed['end']['sim_time'],duration_s=duration,recoveries=recoveries,recovery=recovery)
            self.remove_object('gate');self.wait_sim(2.)
        self.stage='I7';self.goto([5,4,math.pi],'I7_face_away')
        self.e.detail('I7.A_outside_camera',self.outside_camera('low_a'))
        self.until(lambda:self.object_memory('low_a')>0,3,'A occupancy before out-of-view hold')
        start=len(self.memory_events);self.wait_sim(2.)
        outside=self.memory_events[start:]
        self.remove_object('low_a');removed=self.sim_time;start=len(self.memory_events)
        self.wait_sim(2.);unobserved=self.memory_events[start:]
        before_clear=self.memory_events[-1]['cleared']
        clear_begin=len(self.memory_events)
        self.goto([5,4,-math.pi/2],'I7_reobserve')
        self.until(lambda:self.object_memory('low_a')==0,10,'valid free-space deletion of A')
        cleared=next(e for e in self.memory_events[clear_begin:] if e['low_a']==0)
        self.metrics['memory']['A_clear']={'removed':removed,'cleared':cleared,'outside_frames':len(outside),'unobserved_frames':len(unobserved)}
        consecutive=[];eligible=None;previous=None
        for e in self.depth_evidence:
            if e['stamp']<removed or e['stamp']>cleared['observation_stamp']+1e-6:continue
            if previous is not None and e['stamp']<=previous:continue
            gap=e['stamp']-previous if previous is not None else 0.;previous=e['stamp']
            if gap>.3 or not e.get('low_a',{}).get('free',False):consecutive=[]
            if not e.get('low_a',{}).get('free',False):continue
            consecutive.append(e)
            if len(consecutive)>=5 and e['stamp']-consecutive[0]['stamp']>=.2:
                eligible=consecutive[:5];break
        deletion_delay=cleared['sim_time']-eligible[-1]['stamp'] if eligible else None
        self.e.check('V4-09',bool(outside) and all(e['low_a']>0 for e in outside) and bool(unobserved)
                     and all(e['low_a']>0 for e in unobserved) and eligible is not None
                     and deletion_delay is not None and 0<=deletion_delay<=1. and cleared['cleared']>before_clear,
                     outside_hold_s=2.,removed_unobserved_hold_s=2.,consecutive_free_depth_stamps=[e['stamp'] for e in eligible or []],
                     first_empty_memory_stamp=cleared['stamp'],first_empty_observation_stamp=cleared['observation_stamp'],
                     deletion_after_fifth_frame_s=deletion_delay,cleared_counter_increase=cleared['cleared']-before_clear)
        if wanted & {'V4-10','V4-12','V4-13','V4-14','V4-03','V4-04'}:
            self.goto([5,4,math.pi],'I8_face_west');self.stage='I8'
            self.send_goal([3,4,math.pi],'I8')
            self.until(lambda:self.in_region([4.7,4.9,3.8,4.2]),30,'crossing object trigger')
            shape=polygon(self.pose(),self.course['footprint']);box=rectangle(object_bounds(self.course['objects']['crossing']))
            clearance=separation(shape,box)
            self.e.detail('I8.initial_clearance',clearance>=.50,clearance_m=clearance)
            self.create_object('crossing');start=self.sim_time;spec=self.course['objects']['crossing']
            duration=(spec['end'][1]-spec['center'][1])/spec['velocity'][1]
            next_move=self.sim_time
            while self.sim_time<start+duration:
                if self.sim_time>=next_move:
                    delta=min(duration,self.sim_time-start)
                    center=[spec['center'][i]+delta*spec['velocity'][i] for i in (0,1)]
                    self.move_object('crossing',center);next_move=self.sim_time+.05
                self.pump()
            self.move_object('crossing',spec['end']);self.wait_sim(1.);self.remove_object('crossing')
            self.finish_goal()
            states=[msg.action_type for wall,msg in self.samples['/collision_monitor_state']]
            self.metrics['dynamic_event']={'start':start,'end':self.sim_time,'monitor_actions':sorted(set(states)),'clearance_at_insertion':clearance}
        if wanted & {'V4-10','V4-12'}:
            self.stage='I9';self.fault_event('depth',[2,4,math.pi],[2.5,2.7,3.8,4.2],'I9',recover_goal=False)
            self.stage='I10';self.enable(True);self.stationary('I10_enable_without_goal',1.);self.goto([2,4,math.pi],'I10')
            self.stage='I11';self.fault_event('scan',[1.5,4,math.pi],[1.8,1.9,3.8,4.2],'I11')
            self.e.check('V4-10',len(self.metrics['faults'])==2 and all(f['zero_age_s']<=.40 for f in self.metrics['faults']),faults=self.metrics['faults'])
        if wanted & {'V4-13','V4-14'}:
            self.stage='I12'
            self.graph_before=self.slam_graph()
            self.goto([0,4,-math.pi/2],'I12_corner');self.goto([0,0,0],'I12_return')
            self.until(lambda:self.samples['/slam_toolbox/graph_visualization'],5,'SLAM graph')
            self.check_slam_graph()
            self.stage='I13';self.enable(False);self.stationary('before_save',.3)
            last_slam=self.map_pose().copy();prefix=self.e.directory/'saved_map/course'
            prefix.parent.mkdir(exist_ok=True)
            self.cli('save_map',['ros2','run','omslam_mapping','save_map','--output',str(prefix),'--backend','simulation','--workspace',str(WORKSPACE)])
            paths=[Path(str(prefix)+suffix) for suffix in ('.yaml','.pgm','.posegraph','.data','.metadata.json')]
            self.e.detail('I13.map_graph_bundle',all(p.exists() and p.stat().st_size>0 for p in paths),
                          files={str(p):digest(p) for p in paths if p.exists()})
            metadata=json.loads(paths[-1].read_text())
            self.e.detail('I13.bundle_hashes',all(metadata['files'][suffix]['sha256']==digest(Path(str(prefix)+'.'+suffix))
                          for suffix in ('yaml','pgm','posegraph','data')))
            self.saved_grid=self.samples['/map'][-1][1]
            self.stage='I14';self.stop_process('navigation')
            self.until(lambda:not any(name in ('slam_toolbox','amcl','navigation_guard','bt_navigator') for name in self.node.get_node_names()),12,'prior backend/Nav2 graph removal')
            self.e.detail('I14.no_old_final_publisher',not self.node.get_publishers_info_by_topic('/cmd_vel'))
            self.samples['/map'].clear();self.tf_edges.pop(('map','odom'),None);self.guard_state={}
            self.start_navigation('amcl',Path(str(prefix)+'.yaml'),last_slam)
            self.until(lambda:self.samples['/map'] and ('map','odom') in self.tf_edges and self.guard_state.get('ready'),45,'AMCL new session ready')
            self.authority('amcl');self.stationary('AMCL_before_enable',.5)
            self.e.detail('I14.no_old_goal',not self.guard_state.get('goal_id'),guard=self.guard_state)
            self.stage='I15';self.enable(True);self.goto([.7,0,0],'I15')
            self.enable(False);self.stationary('normal_shutdown',.3)
            self.e.check('V4-14',True,saved_bundle=[str(p) for p in paths],AMCL_initial_pose=last_slam.tolist(),initial_pose_source='last SLAM estimate',world_reset=False)
        else:
            self.enable(False);self.stationary('normal_shutdown',.3)
        self.final_checks(wanted)

    def check_memory_marking(self,name):
        candidates=[e for e in self.cloud_events if e.get(name,0)>0]
        self.e.detail('I4.raw_low_obstacle',bool(candidates),observations=len(candidates))
        observed=candidates[0]['stamp'];metrics={}
        for topic in ('/global_costmap/costmap_raw','/local_costmap/costmap_raw'):
            events=[e for e in self.costmap_events if e['topic']==topic and e['sim_time']>=observed
                    and e['objects'].get(name,{}).get('lethal',0)>0]
            metrics[topic]={'marked_at':events[0]['sim_time'] if events else None,
                            'delay':events[0]['sim_time']-observed if events else None}
        memory=[e for e in self.memory_events if e.get(name,0)>0]
        self.e.check('V4-08',bool(memory) and all(v['delay'] is not None and v['delay']<=1. for v in metrics.values()),
                     observation_stamp=observed,costmaps=metrics,memory_first=memory[0] if memory else None)

    def slam_graph(self):
        from scipy.spatial import cKDTree
        markers=self.samples['/slam_toolbox/graph_visualization'][-1][1].markers
        nodes={};segments=[]
        for marker in markers:
            if marker.ns=='slam_toolbox':nodes[marker.id]=[marker.pose.position.x,marker.pose.position.y]
            elif marker.ns=='slam_toolbox_edges':segments += [([a.x,a.y],[b.x,b.y]) for a,b in zip(marker.points[::2],marker.points[1::2])]
        ids=sorted(nodes)
        if not ids:raise AssertionError('No SLAM graph nodes')
        tree=cKDTree([nodes[i] for i in ids]);edges=[]
        for a,b in segments:
            distance,index=tree.query([a,b])
            if max(distance)<1e-4:edges.append([ids[int(index[0])],ids[int(index[1])]])
        distance=np.concatenate(([0.],np.cumsum([np.linalg.norm(np.subtract(nodes[b],nodes[a])) for a,b in zip(ids,ids[1:])])))
        at=dict(zip(ids,distance))
        revisits=[[a,b] for a,b in edges if abs(at[a]-at[b])>6 and min(a,b)<=ids[max(1,len(ids)//4)] and max(a,b)>=ids[3*len(ids)//4]]
        return {'nodes':nodes,'edges':edges,'revisit_edges':revisits,'time':self.sim_time}

    def check_slam_graph(self):
        graph=self.slam_graph();nodes=graph['nodes'];edges=graph['edges'];revisits=graph['revisit_edges']
        graph['before_return']=self.graph_before
        (self.e.directory/'pose_graph.json').write_text(json.dumps(graph,indent=2)+'\n')
        transforms=[e for e in self.tf_observations if e['edge']==['map','odom'] and e['mode']=='slam']
        changes=[]
        for previous,current in zip(transforms,transforms[1:]):
            distance=float(np.linalg.norm(np.subtract(current['pose'][:2],previous['pose'][:2])))
            rotation=abs(angle_delta(current['pose'][2],previous['pose'][2]))
            if distance>1e-6 or rotation>1e-6:changes.append({'time':current['sim_time'],'distance':distance,'rotation':rotation})
        projections=[e for e in self.costmap_events if e['mode']=='slam' and e['topic']=='/global_costmap/costmap_raw' and 'memory_projection' in e]
        large=[c for c in changes if c['distance']>=.1 or c['rotation']>=.1]
        if large:
            response=[]
            for correction in large:
                states=[e for e in self.states if correction['time']<=e['sim_time']<=correction['time']+.2]
                latched=any(e['state']=='FAULT_LATCHED' for e in states)
                finals=[e for e in self.command_events if e['topic']=='/cmd_vel' and correction['time']<=e['sim_time']<=correction['time']+.2]
                response.append({'correction':correction,'latched':latched,'zero':any(not any(e['velocity']) for e in finals)})
            branch={'name':'large_correction_latched','response':response,'pass':all(e['latched'] and e['zero'] for e in response)}
        else:
            pair=None
            for before in projections:
                for after in reversed(projections):
                    if after['sim_time']<=before['sim_time']:continue
                    a,b=before['memory_projection'],after['memory_projection']
                    distance=float(np.linalg.norm(np.subtract(a['map_from_odom'][:2],b['map_from_odom'][:2])))
                    rotation=abs(angle_delta(a['map_from_odom'][2],b['map_from_odom'][2]))
                    if distance>=1e-4 or rotation>=1e-4:
                        pair={'before':before,'after':after,'translation_m':distance,'rotation_rad':rotation};break
                if pair:break
            aligned=pair is not None and all(pair[k]['memory_projection']['in_bounds']>0 and
                pair[k]['memory_projection']['lethal_fraction']>=.95 for k in ('before','after'))
            branch={'name':'small_correction_reprojected','pass':aligned,'pair':pair,'projection_samples':len(projections)}
        before_edges={tuple(sorted(e)) for e in (self.graph_before or {}).get('edges',[])}
        new_revisits=[e for e in revisits if tuple(sorted(e)) not in before_edges]
        updated=len({e['sha256'] for e in self.map_events if e['mode']=='slam'})>=2
        self.metrics['slam_correction']={'changes':changes,'branch':branch}
        self.e.check('V4-13',len(nodes)>25 and bool(new_revisits) and updated and branch['pass'],
                     nodes=len(nodes),edges=len(edges),early_late_revisit_edges=revisits,map_messages=self.counts['/map'],
                     new_revisit_edges=new_revisits,distinct_map_updates=updated,map_correction_branch=branch,
                     boundary_complement='Non-driving layer/guard tests verify former bounds cleanup and the 0.10 m/rad latch boundary')

    def cli(self,label,command):
        self.start(label,command);proc=self.processes[label]
        # Keep ownership during a bounded wait, including timeout cleanup.
        self.allowed_exit.add(label)
        try:
            self.until(lambda:proc.poll() is not None,40,label+' completed')
            code=proc.returncode
        finally:
            self.stop_process(label);self.allowed_exit.discard(label)
        if code:raise RuntimeError(label+' failed with '+str(code))

    def final_checks(self,wanted,physical_only=False,sensor_checks=True):
        successes=[g for g in self.goals if g.get('status')==4]
        if 'V4-02' in wanted:
            self.e.check('V4-02',bool(successes) and all(g.get('position_error',float('inf'))<=.15 and g.get('yaw_error',float('inf'))<=.20 for g in successes),
                         successful_goals=len(successes),max_position_error_m=max(g['position_error'] for g in successes),
                         max_yaw_error_rad=max(g['yaw_error'] for g in successes),goal_details='goals.json')
        collision_paths=[];footprint=np.asarray(self.course['footprint'])
        footprint=footprint+np.sign(footprint)*self.course['padding']
        marked={name:min((e['sim_time'] for e in self.costmap_events if e['topic']=='/global_costmap/costmap_raw'
                          and e['objects'].get(name,{}).get('lethal',0)>0),default=float('inf')) for name in ('low_a','low_b','gate','crossing')}
        # Historical plans are assessed with obstacles known at planning time.
        # An inserted obstacle triggers subsequent replanning; it cannot make an old plan retroactively invalid.
        for path in ([] if physical_only else self.paths):
            if self.world_to_map is None:continue
            obstacles=[(o['name'],rectangle(o['bounds'])) for o in self.course['static_boxes']]
            for name in ('low_a','low_b'):
                if path['time']>=marked[name] and not(name=='low_a' and path['stage'] in ('I8','I9','I10','I11','I12','I13','I14','I15')):
                    obstacles.append((name,rectangle(object_bounds(self.course['objects'][name]))))
            for interval in self.object_intervals:
                if interval['name']=='gate' and marked['gate']<=path['time'] and interval['created']<=path['time']<=(interval['removed'] or float('inf')):
                    obstacles.append(('gate',rectangle(object_bounds(self.course['objects']['gate']))))
                if interval['name']=='crossing' and marked['crossing']<=path['time'] and interval['created']<=path['time']<=(interval['removed'] or float('inf')):
                    positions=[e for e in self.object_positions if e['name']=='crossing' and e['time']<=path['time']]
                    if positions:obstacles.append(('crossing',rectangle(object_bounds(self.course['objects']['crossing'],positions[-1]['center']))))
            poses=[compose(inverse(self.world_to_map),p) for p in path['poses']]
            for a,b in zip(poses,poses[1:]):
                for pose in interpolate(a,b):
                    shape=polygon(pose,footprint)
                    hits=[name for name,box in obstacles if intersects(shape,box)]
                    if hits:collision_paths.append({'stage':path['stage'],'time':path['time'],'pose':pose.tolist(),'obstacles':hits});break
                if collision_paths and collision_paths[-1]['time']==path['time']:break
        if 'V4-03' in wanted:
            self.e.check('V4-03',len(self.trajectory)>100 and not self.collision
                         and (physical_only or (len(self.paths)>5 and not collision_paths)),footprint_overlaps=len(self.collision),
                         minimum_footprint_clearance_m=self.minimum_clearance,planned_paths_evaluated=not physical_only,
                         path_collision_count=None if physical_only else len(collision_paths),
                         path_collisions=collision_paths[:20],trajectory_samples=len(self.trajectory),
                         interpretation='Raw physical envelope against all active obstacles throughout the five-goal drive' if physical_only else
                         'Raw physical envelope against all active obstacles for motion; padded Nav2 footprint against static and already marked obstacles for each plan timestamp')
        if 'V4-04' in wanted:
            stages = command_limit_evidence(self.command_events, self.drive_geometry, self.drive_limits)
            self.e.check('V4-04', all(stage['passed'] for stage in stages.values()), stages=stages,
                         max_rpm=stages['/sim/cmd_vel'].get('max_rpm'), geometry=self.drive_geometry,
                         command_limits=self.drive_limits)
        validation = self.validation
        validation_path = self.validation_manifest
        contracts = validation.get('checks', {})
        if 'V4-05' in wanted:
            corridor=[p for p in self.trajectory if 1.5<=p['world'][0]<=3.5 and abs(p['world'][1])<.4]
            crossed=bool(corridor) and min(p['world'][0] for p in corridor)<1.6 and max(p['world'][0] for p in corridor)>3.4
            self.e.check('V4-05',crossed and contracts.get('P0.7.actual_plugins',{}).get('status')=='PASS',
                         corridor_samples=len(corridor),actual_plugin_fixture=contracts.get('P0.7.actual_plugins'),validation=str(validation_path))
        if 'V4-06' in wanted:
            self.e.check('V4-06',bool(self.metrics.get('cancel')) and self.metrics['cancel']['status']==5
                         and any(g['label']=='I3' and g.get('status')==4 for g in self.goals),
                         cancel_uuid=self.metrics.get('cancel',{}).get('id'),subsequent_uuid=next((g['id'] for g in self.goals if g['label']=='I3'),None),
                         stops=self.metrics['stops'])
        if 'V4-11' in wanted:
            probe=validation.get('command_chain',{})
            source={e['stamp'] for e in self.command_events if e['topic']=='/cmd_vel/requested'}
            final=[e for e in self.command_events if e['topic']=='/cmd_vel' and any(e['velocity'])]
            matching=all(e['stamp'] in source and e['frame']=='base_footprint' for e in final)
            self.e.check('V4-11',bool(final) and matching and probe.get('status')=='PASS',
                         nonzero_finals=len(final),all_original_stamps_preserved=matching,isolated_chain_evidence=probe)
        if 'V4-12' in wanted:
            tests=validation.get('tests',{})
            preserved=len(self.metrics['faults'])==2 and all(e['memory_before']>0 and e['memory_after']>0 for e in self.metrics['faults'])
            self.e.check('V4-12',preserved and tests.get('status')=='PASS' and tests.get('map_correction_gate') is True,
                         fault_memory=self.metrics['faults'],boundary_tests=tests)
        if sensor_checks:self.sensor_summary()

    def sensor_summary(self):
        metrics={}
        for topic in ('/scan','/imu/raw','/imu/data','/camera/depth/image_rect_raw','/odom','/wheel/odometry'):
            values=self.timing[topic]
            ages=np.array([v[3] for v in values]);stamps=np.array([v[2] for v in values]);gaps=np.diff(stamps)
            metrics[topic]={'count':len(values),'age_p50_s':float(np.median(ages)),'age_p99_s':float(np.percentile(ages,99)),
                            'age_max_s':float(np.max(ages)),'max_stamp_gap_s':float(np.max(gaps)) if len(gaps) else None,
                            'monotonic_stamps':bool(np.all(gaps>=0)),'frames':sorted(set(v[4] for v in values))}
        scan=[e for e in self.sensor_observations if e['topic']=='/scan']
        imu=[e for e in self.sensor_observations if e['topic']=='/imu/data']
        depth=[e for e in self.sensor_observations if e['topic']=='/camera/depth/image_rect_raw']
        self.e.detail('sensors_same_drive',all(v['count']>10 and v['monotonic_stamps'] for v in metrics.values())
                      and all(e['frame']=='laser_frame' and e['valid_count']>20 for e in scan)
                      and all(e['orientation_valid'] and .99<=e['quaternion_norm']<=1.01 for e in imu)
                      and all(e['encoding'] in ('32FC1','16UC1') and e['frame']=='camera_optical_frame' and e['valid_fraction']>.05 for e in depth),
                      topics=metrics,clock_reset=self.clock_reset)
        self.metrics['sensors']=metrics
        continuity={}
        for label,records in [('odom',self.odom_observations),('odom_to_base',[e for e in self.tf_observations if e['edge']==['odom','base_footprint']])]:
            jumps=[];steps=[]
            for a,b in zip(records,records[1:]):
                dt=b['stamp']-a['stamp'];distance=float(np.linalg.norm(np.subtract(b['pose'][:2],a['pose'][:2])))
                rotation=abs(angle_delta(b['pose'][2],a['pose'][2]))
                if dt<0 or distance>.03+.6*max(dt,0) or rotation>.04+1.5*max(dt,0):
                    jumps.append({'before':a,'after':b,'dt':dt,'distance':distance,'rotation':rotation})
                steps.append([dt,distance,rotation])
            continuity[label]={'samples':len(records),'jumps':jumps[:10],'max_step_m':max((e[1] for e in steps),default=None)}
        self.e.detail('odom_tf_same_drive_continuity',not self.clock_reset and all(v['samples']>100 and not v['jumps'] for v in continuity.values())
                      and all(e['frame']=='odom' and e['child']=='base_footprint' for e in self.odom_observations),series=continuity)
        truth_stamps=np.array([e['stamp'] for e in self.trajectory]);turns=[]
        for observation in imu:
            if 'base_wz' not in observation:continue
            index=int(np.searchsorted(truth_stamps,observation['stamp']))
            candidates=[i for i in (index-1,index) if 0<=i<len(self.trajectory)]
            if not candidates:continue
            nearest=min(candidates,key=lambda i:abs(truth_stamps[i]-observation['stamp']));truth=self.trajectory[nearest]
            if abs(truth['stamp']-observation['stamp'])<=.05 and abs(truth['velocity'][1])>.10:
                turns.append({'stamp':observation['stamp'],'imu_wz':observation['base_wz'],'truth_wz':truth['velocity'][1]})
        sign_fraction=sum(t['imu_wz']*t['truth_wz']>0 for t in turns)/len(turns) if turns else 0.
        self.e.detail('imu_yaw_sign_same_drive',len(turns)>20 and sign_fraction>=.98,turn_samples=len(turns),sign_agreement=sign_fraction)
        self.metrics['real_time_factor']={'samples':len(self.gz_stats),'p50':float(np.median([s[2] for s in self.gz_stats])) if self.gz_stats else None}
        self.metrics['ros_warnings_errors']=self.ros_errors

    def plot(self):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        fig,ax=plt.subplots(figsize=(10,9))
        if hasattr(self,'saved_grid'):
            grid=self.saved_grid;cells=np.asarray(grid.data).reshape(grid.info.height,grid.info.width)
            image=np.where(cells<0,.65,1.-cells/100.)
            x,y,res=grid.info.origin.position.x,grid.info.origin.position.y,grid.info.resolution
            ax.imshow(image,origin='lower',cmap='gray',vmin=0,vmax=1,extent=[x,x+grid.info.width*res,y,y+grid.info.height*res])
        for box in self.course['static_boxes']:
            x0,x1,y0,y1=box['bounds'];ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,edgecolor='#666666',linewidth=.5))
        for mode,color in [('slam','#bf5b16'),('amcl','#1f69b3')]:
            data=[p for p in self.trajectory if p['mode']==mode and p['truth_map'] is not None]
            if data:
                values=np.asarray([p['truth_map'] for p in data]);ax.plot(values[:,0],values[:,1],color=color,label=mode+' truth (fixed alignment)',linewidth=1.5)
        for g in self.goals:
            if g.get('status')==4:ax.plot(*g['goal'][:2],'o',color='#188b61',markersize=3)
        title='P4: five consecutive goals (online SLAM)' if self.e.data.get('scenario')=='five-goals' else 'P4 recorded SLAM / saved-map navigation'
        ax.set(xlabel='Map x [m]',ylabel='Map y [m]',title=title);ax.set_aspect('equal');ax.legend();fig.tight_layout()
        fig.savefig(self.e.directory/'map_and_trajectory.png',dpi=150);plt.close(fig)

    def snapshot(self):
        for filename,contents in [('commands.json',self.commands),('metrics.json',self.metrics),('goals.json',self.goals),
                                  ('trajectory.json',self.trajectory),('plans.json',self.paths),('sensor_timing.json',self.timing),
                                  ('memory_observations.json',self.memory_events),('costmap_observations.json',self.costmap_events),('map_observations.json',self.map_events),
                                  ('object_positions.json',self.object_positions),('object_intervals.json',self.object_intervals),
                                  ('dwb_evaluations.json',self.dwb_evaluations),
                                  ('failure_costmaps.json',self.costmap_snapshots),
                                  ('depth_target_evidence.json',self.depth_evidence),('command_observations.json',self.command_events),('tf_observations.json',self.tf_observations),('odom_observations.json',self.odom_observations),('bt_events.json',self.bt_events),('guard_states.json',self.states),('sensor_observations.json',self.sensor_observations)]:
            (self.e.directory/filename).write_text(json.dumps(contents,indent=2,default=jsonable)+'\n')
        self.e.data.update(sample_counts=dict(self.counts),sim_duration_s=self.sim_time,wall_duration_s=time.monotonic()-self.start_wall,
                           final_guard_state=self.guard_state,actual_movement=any(abs(p['velocity'][0])>.01 or abs(p['velocity'][1])>.02 for p in self.trajectory))
        self.e.save()
        if self.trajectory:self.plot()

    def close(self):
        self.stopping=True
        if self.node is not None and 'navigation' in self.processes and self.processes['navigation'].poll() is None:
            try:self.enable(False)
            except Exception as exc:self.e.data['shutdown_disable_error']=str(exc)
            # Drain the terminal ROS logs and pending costmap replies only after
            # disabling motion, before tearing down their publishers/services.
            try:self.pump(.3)
            except Exception as exc:self.e.data['shutdown_observation_error']=str(exc)
        for label in list(self.processes):
            if label!='simulator':self.stop_process(label)
        self.stop_process('simulator')
        if self.writer is not None:self.writer=None
        if self.odom_executor is not None:self.odom_executor.shutdown()
        if self.odom_node is not None:self.odom_node.destroy_node()
        if self.executor is not None:self.executor.shutdown()
        if self.node is not None:self.node.destroy_node()
        if self.context is not None and self.context.ok():self.rclpy.shutdown(context=self.context)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=['integrated','five-goals'],default='integrated')
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--checks')
    parser.add_argument('--baseline-results',type=Path)
    parser.add_argument('--build-manifest', type=Path, default=DEFAULT_BUILD)
    parser.add_argument('--validation-manifest', type=Path, default=DEFAULT_VALIDATION)
    parser.add_argument('--timeout',type=float,default=2400)
    parser.add_argument('--domain',type=int,default=176)
    args=parser.parse_args()
    available=FIVE_GOAL_CHECKS if args.scenario=='five-goals' else ALL_CHECKS
    checks=args.checks.split(',') if args.checks else available
    if set(checks)-set(available):parser.error('Unknown acceptance IDs')
    if args.scenario=='five-goals' and (args.checks or args.baseline_results):
        parser.error('The five-goal scenario completes all five goals in one execution without a baseline')
    if set(checks)!=set(available) and not args.baseline_results:parser.error('Partial verification requires --baseline-results')
    os.environ['ROS_DOMAIN_ID']=str(args.domain);os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS',None)
    os.environ['GZ_PARTITION']='omslam_p4_'+str(os.getpid());os.environ['OMSLAM_GZ_PARTITION']=os.environ['GZ_PARTITION']
    result=Evidence(args.output,checks,args.baseline_results,args.scenario);run=NavigationRun(result,args.timeout)
    run.build_manifest = args.build_manifest.resolve()
    run.validation_manifest = args.validation_manifest.resolve()
    run.validation = {}
    try:
        build = verified_build(run.build_manifest)
        from ament_index_python.packages import get_package_prefix
        prefixes = {pkg: Path(get_package_prefix(pkg)).resolve() for pkg in build['packages']}
        mismatches = {pkg: str(path) for pkg, path in prefixes.items() if path != WORKSPACE/'install'/pkg}
        if mismatches:
            raise ValueError(f'Source install/setup.bash from {WORKSPACE}; unexpected package prefixes: {mismatches}')
        result.detail('I0.latest_built_sources', True, build_manifest=str(run.build_manifest))
        result.data['source_sha256'] = build['source_sha256']
        required = set(checks) & {'V4-05', 'V4-11', 'V4-12'}
        if required:
            validation = json.loads(run.validation_manifest.read_text())
            needed = {'V4-05': 'P0.7.actual_plugins', 'V4-11': 'command_chain', 'V4-12': 'map_correction_gate'}
            if (validation.get('build_manifest_sha256') != digest(run.build_manifest)
                    or validation.get('source_sha256') != build['source_sha256'] or validation.get('status') != 'PASS'
                    or not all(validation['checks'].get(needed[k], {}).get('status') == 'PASS' for k in required)):
                raise ValueError(f'Run verify_nav_contracts.py --integrated for this build: {run.validation_manifest}')
            run.validation = validation
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        result.data.update(status='FAIL', error={'message':str(exc)}, actual_movement=False)
        result.save()
        result.events.close()
        print(f'Preflight failed before starting simulation: {exc}', flush=True)
        return 1
    ledger=WORKSPACE/'reports/sim-navigation/drive_ledger.json'
    ledger.parent.mkdir(parents=True, exist_ok=True)
    entries=json.loads(ledger.read_text()) if ledger.exists() else []
    entry={'run':str(result.directory),'pid':os.getpid(),'started_at':result.data['started_at'],'status':'RUNNING',
           'scenario':args.scenario,'checks':checks,'baseline':str(args.baseline_results) if args.baseline_results else None}
    entries.append(entry);ledger.write_text(json.dumps(entries,indent=2)+'\n')
    try:
        if args.scenario=='five-goals':run.run_five_goals()
        else:run.run(checks)
    except Exception as exc:
        result.data['error']={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
        print(traceback.format_exc(),flush=True)
    finally:
        try:run.close()
        except Exception as exc:result.data['cleanup_error']=str(exc)
        run.snapshot()
    required_pass=all(result.data['checks'][key]['status']=='PASS' for key in checks)
    result.data['status']='PASS' if required_pass and 'error' not in result.data and 'cleanup_error' not in result.data else 'FAIL'
    if result.data['status']=='PASS' and set(checks)!=set(available):result.data['status']='SELECTED_CHECKS_PASS'
    result.data['completed_at']=datetime.now().astimezone().isoformat();result.save()
    entry.update(status=result.data['status'],actual_movement=result.data['actual_movement'],completed_at=result.data['completed_at'])
    ledger.write_text(json.dumps(entries,indent=2)+'\n')
    rows=['# '+result.data['scope'], '', 'Status: '+result.data['status'],'','| Check | Result |','| --- | --- |']
    rows += [f"| {key} | {value['status']} |" for key,value in result.data['checks'].items()]
    if 'error' in result.data:rows+=['',result.data['error']['message']]
    (result.directory/'summary.md').write_text('\n'.join(rows)+'\n')
    result.events.close();print(result.data['status']+': '+str(result.directory/'results.json'),flush=True)
    return 0 if required_pass and result.data['status'] in ('PASS','SELECTED_CHECKS_PASS') else 1

if __name__=='__main__':raise SystemExit(main())
