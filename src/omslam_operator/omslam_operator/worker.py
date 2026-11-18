"""ROS transport owned by one background thread; never refreshes UI intent."""
import copy
import json
import math
import os
import socket
import threading
import time
import uuid
from types import SimpleNamespace
from collections import deque

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped, Polygon, Point32
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import Image, Imu, JointState, LaserScan, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from omm_perception_msgs.msg import MemoryStatus, PerceptionHealth
from omslam_operator_msgs.msg import OperatorState, ManualCommand, OwnerHeartbeat, MotorTelemetry, KeepoutZones
from omslam_operator_msgs.srv import Control, SetSpeedLimit, MapCommand, SetKeepoutZones
from omslam_operator_msgs.action import OperatorNavigate
from PySide6.QtGui import QImage
from .geometry import depth_array, depth_rgb, quaternion_rpy, quaternion_product, rotate_vector


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def plain(message):
    return json.loads(json.dumps(message_to_ordereddict(message)))


class Shared:
    def __init__(self, path_debug_directory=''):
        self.lock=threading.RLock()
        self.path_debug_directory=path_debug_directory
        self.data={'topics':{},'diagnostics':{},'events':[], 'operator':{},'navigation':{},
                   'velocity':{},'joints':{},'imu':{},'motor':{},'memory':{},'cameras':{},
                   'pose':None,'path':[],'layers':{},'scan':[], 'trace':[], 'history':[],
                   'goal_feedback':{},'goal_result':{}, 'control_response':{},'map_response':{},
                   'ros_time':0.,'host':socket.gethostname(),'domain':os.getenv('ROS_DOMAIN_ID','0')}
        self.images={}
        self.history_enabled=True
        self.map_layers={}
        self.map_revisions={}
        self.depth=None
        self.controls=deque(maxlen=32)
        self.latest_manual=None
        self.latest_heartbeat=None
        self.stop=threading.Event()
        self.node=None
        self.goal_handle=None
        self.goal_active=False
        self.path_blocked=False
        self.path_start_ns=0

    def touch_map(self,key):
        """Caller holds lock; revisions cross the UI boundary, dense arrays do not."""
        self.map_revisions[key]=self.map_revisions.get(key,0)+1
        return self.map_revisions[key]

    def event(self,message):
        with self.lock:
            events=self.data['events']
            if events and events[0]['message']==message:
                events[0]['count']+=1
                return
            events.insert(0,{'time':time.strftime('%H:%M:%S'),'message':str(message),'count':1})
            del events[200:]


class RosWorker(threading.Thread):
    def __init__(self, shared, use_sim_time=True, namespace=''):
        super().__init__(daemon=True,name='operator-ros')
        self.shared=shared
        self.use_sim_time=use_sim_time
        self.namespace=namespace

    def run(self):
        executor=None
        try:
            self.node=ConsoleNode(self.shared,self.use_sim_time,self.namespace)
            self.shared.node=self.node
            executor=SingleThreadedExecutor(context=self.node.context)
            executor.add_node(self.node)
            while rclpy.ok() and not self.shared.stop.is_set():
                # rclpy.spin_once(node) adds/removes the node on every call,
                # invalidating the ready-callback iterator for every message.
                executor.spin_once(timeout_sec=.02)
                # A continuously ready ROS graph can immediately reacquire the
                # GIL after Qt releases it in a native call. Yield between
                # callbacks so the UI thread can sample fresh operator intent.
                # This never manufactures or repeats a heartbeat in the worker.
                time.sleep(0)
        except Exception as exc:
            self.shared.event('ROS 연결 오류: '+str(exc))
        finally:
            self.shared.node=None
            if executor is not None:executor.shutdown()
            if hasattr(self,'node'):self.node.destroy_node()


class ConsoleNode(Node):
    SCAN_DISPLAY_TIMEOUT = .30
    CLOUD_DISPLAY_TIMEOUT = .30

    def __init__(self, shared, use_sim_time, namespace):
        super().__init__('operator_console',namespace=namespace,
                         parameter_overrides=[Parameter('use_sim_time',value=use_sim_time)])
        self.s=shared
        self.tf=Buffer(cache_time=Duration(seconds=10.),node=self)
        self.listener=TransformListener(self.tf,self,spin_thread=False)
        self.last_graph=0.
        self.last_history=0.
        self.frame_visible='camera'
        self.tf_errors={}
        self.last_revision={}
        self.last_event_signature=''
        self.pending_samples={}
        self.last_ros_time=None
        self.sample_received=None
        self.pending_scans=deque(maxlen=4)
        self.displayed_scan=None
        self.latest_scan_stamp=None
        self.pending_clouds={}
        self.displayed_clouds={}
        self.latest_cloud_stamps={}
        transient=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.path_debug_pub=None
        if self.s.path_debug_directory:
            self.path_debug_pub=self.create_publisher(String,self.topic('/operator/path_debug/events'),
                QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.sub(String,'/operator/path_debug/status',self.path_debug_status,transient)
        self.sub(OperatorState,'/operator/state',self.operator,transient)
        self.sub(KeepoutZones,'/operator/keepouts',lambda m:self.assign('keepouts',plain(m)),transient)
        self.sub(String,'/navigation/state',self.navigation,transient)
        self.sub(DiagnosticArray,'/diagnostics',self.diagnostics,10)
        self.sub(MotorTelemetry,'/base/motor_state',lambda m:self.assign('motor',plain(m)),10)
        self.sub(OccupancyGrid,'/map',lambda m:self.grid('map',m),transient)
        self.sub(PoseWithCovarianceStamped,'/amcl_pose',lambda m:self.assign('amcl_pose_seen',True),10)
        self.sub(Path,'/plan',self.path,10)
        for topic in ('/wheel/odometry','/odom'):
            self.sub(Odometry,topic,lambda m,t=topic:self.velocity(t,m.twist.twist),sensor)
        self.sub(JointState,'/joint_states',self.joints,sensor)
        self.sub(Imu,'/imu/data',self.imu,sensor)
        self.sub(LaserScan,'/scan',self.scan,sensor)
        for key,topic in [('rgb','/camera/color/image_raw'),('depth','/camera/depth/image_rect_raw')]:
            self.sub(Image,topic,lambda m,k=key:self.camera(k,m),sensor)
        for key,topic in [('rgb','/camera/color/camera_info'),('depth','/camera/depth/camera_info')]:
            self.sub(CameraInfo,topic,lambda m,k=key:self.info(k,m),sensor)
        for key,topic in [('global','/global_costmap/costmap_raw'),('local','/local_costmap/costmap_raw')]:
            self.sub(Costmap,topic,lambda m,k=key:self.grid(k,m),transient)
        self.sub(MemoryStatus,'/perception/memory_status',lambda m:self.assign('memory',plain(m)),transient)
        self.sub(PointCloud2,'/perception/memory_preview',lambda m:self.cloud('memory_points',m),transient)
        self.sub(PerceptionHealth,'/perception/health',lambda m:self.assign('health',plain(m)),transient)
        for key,topic in [('obstacles','/perception/obstacles'),('points','/camera/points')]:
            self.sub(PointCloud2,topic,lambda m,k=key:self.cloud(k,m),sensor)
        for topic in ('/cmd_vel/requested','/cmd_vel/smoothed','/cmd_vel/monitored','/cmd_vel'):
            self.sub(TwistStamped,topic,lambda m,t=topic:self.velocity(t,m.twist),10)
        self.manual_pub=self.create_publisher(ManualCommand,self.topic('/operator/manual'),1)
        self.heartbeat_pub=self.create_publisher(OwnerHeartbeat,self.topic('/operator/heartbeat'),1)
        self.control_client=self.create_client(Control,self.topic('/operator/control'))
        self.stop_client=self.create_client(Trigger,self.topic('/operator/stop'))
        self.speed_client=self.create_client(SetSpeedLimit,self.topic('/operator/speed_limit'))
        self.maps_client=self.create_client(MapCommand,self.topic('/operator/maps'))
        self.keepouts_client=self.create_client(SetKeepoutZones,self.topic('/operator/set_keepouts'))
        self.navigate=ActionClient(self,OperatorNavigate,self.topic('/operator/navigate'))
        self.create_timer(.02,self.pump,clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))

    def topic(self,name):
        ns=self.get_namespace().rstrip('/')
        return ns+name

    def path_debug_status(self,message):
        status=json.loads(message.data)
        if status.get('directory')==self.s.path_debug_directory:
            self.assign('path_debug',dict(status,_received=time.monotonic()))

    def debug_event(self,event,request_id='',**payload):
        """Publish only explicit diagnostic fields, never command capabilities."""
        publisher=getattr(self,'path_debug_pub',None)
        if publisher is None:return
        try:
            value=dict(payload,event=event,request_id=request_id,event_id=uuid.uuid4().hex,
                       ros_stamp_ns=self.get_clock().now().nanoseconds,
                       directory=self.s.path_debug_directory)
            publisher.publish(String(data=json.dumps(value,ensure_ascii=False,allow_nan=False)))
        except Exception as exc:
            self.s.event('경로 기록 이벤트 전송 오류: '+str(exc))

    def sub(self,typ,topic,callback,qos):
        def receive(message):
            now=time.monotonic()
            header=getattr(message,'header',None)
            stamp=getattr(message,'observation_stamp',None) or getattr(header,'stamp',None) or getattr(message,'stamp',None)
            sample=(message,callback,now,stamp_seconds(stamp) if stamp else None,
                    getattr(header,'frame_id',''),typ.__name__)
            # A sensor/state stream can reach DDS before its matching /clock.
            # Keep the last coherent value visible until clock catches up, and
            # never renew the original receipt deadline while waiting.
            ros=self.flush_pending(topic)
            # Host diagnostics use wall time even in a simulated ROS session;
            # their UI freshness is deliberately based on steady receipt time.
            if topic!='/diagnostics' and sample[3] is not None and sample[3]>ros:
                self.pending_samples[topic]=sample
            else:
                self.deliver_sample(topic,sample)
        return self.create_subscription(typ,self.topic(topic),receive,qos)

    def flush_pending(self,topic=None):
        ros=self.get_clock().now().nanoseconds*1e-9
        if self.last_ros_time is not None and ros<self.last_ros_time:
            # Also handle rewinds smaller than tf2's default one-second jump
            # threshold. Old dynamic transforms must not cross clock epochs.
            self.tf.clear()
            self.pending_samples.clear()
            self.pending_scans.clear()
            self.displayed_scan=None
            self.latest_scan_stamp=None
            self.assign('scan',[])
            self.assign('scan_info',{'display_status':'clock_reset','display_stamp':None})
            self.pending_clouds.clear()
            self.displayed_clouds.clear()
            self.latest_cloud_stamps.clear()
            for key in ('memory_points','obstacles','points'):self.assign(key,[])
            self.assign('cloud_info',{})
            with self.s.lock:self.s.data['topics'].clear()
        self.last_ros_time=ros
        for name in ([topic] if topic is not None else list(self.pending_samples)):
            sample=self.pending_samples.get(name)
            if sample is None:continue
            timeout=.3 if name=='/operator/state' else .8
            if time.monotonic()-sample[2]>=timeout:
                self.pending_samples.pop(name,None)
            elif sample[3]<=ros:
                self.pending_samples.pop(name,None)
                self.deliver_sample(name,sample)
        return ros

    def deliver_sample(self,topic,sample):
        message,callback,received,stamp,frame,type_name=sample
        with self.s.lock:
            prev=self.s.data['topics'].get(topic,{})
            interval=received-prev.get('received',received)
            self.sample_received=received
            try:callback(message)
            except Exception as exc:self.s.event(topic+' 표시 오류: '+str(exc))
            finally:self.sample_received=None
            self.s.data['topics'][topic]={'received':received,'stamp':stamp,'frame':frame,
                'hz':.8*prev.get('hz',0)+.2/interval if interval>0 else 0.,
                'type':type_name,'count':prev.get('count',0)+1}

    def assign(self,key,value):
        if key=='goal_result' and value:
            value=dict(value,_received=time.monotonic())
        with self.s.lock:
            self.s.data[key]=value
            if key=='goal_result' and value:
                # /plan has no action ID. Keep a completed action's late DDS
                # samples hidden until another goal starts in this console.
                self.s.path_blocked=True
                self.s.data['path']=[]
                self.s.touch_map('path')
            if key in ('path','scan','memory_points','obstacles'):
                self.s.touch_map(key)

    def operator(self,msg):
        state=plain(msg)
        self.assign('operator',state)
        sig=str((state.get('mode'),state.get('phase'),state.get('reason')))
        if sig!=self.last_event_signature:
            self.s.event('제어 상태: '+str(state.get('mode',''))+' / '+str(state.get('reason','')))
            self.last_event_signature=sig

    def navigation(self,msg):
        self.assign('navigation',json.loads(msg.data))

    def diagnostics(self,msg):
        with self.s.lock:
            for row in msg.status:
                self.s.data['diagnostics'][row.name]={'name':row.name,'level':int.from_bytes(row.level,'little') if isinstance(row.level,(bytes,bytearray)) else int(row.level),'message':row.message,'hardware_id':row.hardware_id,
                    'values':[{ 'key':v.key,'value':v.value} for v in row.values], 'received':time.monotonic()}

    def velocity(self,topic,twist):
        with self.s.lock:self.s.data['velocity'][topic]={'v':twist.linear.x,'w':twist.angular.z}

    def joints(self,msg):
        rows={}
        for i,name in enumerate(msg.name):
            velocity=msg.velocity[i] if i<len(msg.velocity) else None
            rows[name]={'position':msg.position[i] if i<len(msg.position) else None,
                        'velocity':velocity,'rpm':velocity*30/math.pi if velocity is not None else None}
        self.assign('joints',rows)

    def imu(self,msg):
        rpy=quaternion_rpy(msg.orientation) if msg.orientation_covariance[0]!=-1 else None
        sensor={'rpy':list(rpy) if rpy else [],'valid':bool(rpy),
            'frame':msg.header.frame_id,'gyro':[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],
            'accel':[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],
            'accel_valid':bool(msg.linear_acceleration_covariance[0]!=-1),
            'covariance':[float(value) for value in msg.orientation_covariance],'covariance_frame':msg.header.frame_id,
            'quaternion':[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w],
            'gyro_covariance':[float(value) for value in msg.angular_velocity_covariance],
            'accel_covariance':[float(value) for value in msg.linear_acceleration_covariance]}
        self.assign('imu',sensor)
        base=dict(sensor,frame='base_footprint',valid=False,rpy=[],gyro=[],accel=[])
        try:
            t=self.tf.lookup_transform('base_footprint',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
            q=t.rotation
            if quaternion_rpy(q) is None:raise ValueError('invalid mounting orientation')
            mount=(q.x,q.y,q.z,q.w)
            base['gyro']=list(rotate_vector(mount,sensor['gyro']))
            if sensor['accel_valid']:base['accel']=list(rotate_vector(mount,sensor['accel']))
            if rpy:
                o=msg.orientation
                body=quaternion_product((o.x,o.y,o.z,o.w),(-q.x,-q.y,-q.z,q.w))
                body_rpy=quaternion_rpy(SimpleNamespace(**dict(zip(('x','y','z','w'),body))))
                base.update(rpy=list(body_rpy),valid=True)
        except Exception:
            base['frame']='base_footprint (장착 TF 대기)'
        self.assign('imu_base',base)

    def transform(self,frame,stamp):
        if frame=='map':return (0.,0.,0.)
        tr=self.tf.lookup_transform('map',frame,Time.from_msg(stamp)).transform
        rpy=quaternion_rpy(tr.rotation)
        if not rpy:raise ValueError('잘못된 TF 자세')
        return tr.translation.x,tr.translation.y,rpy[2]

    def project(self,points,frame,stamp):
        ox,oy,yaw=self.transform(frame,stamp)
        c,s=math.cos(yaw),math.sin(yaw)
        return [[ox+c*x-s*y,oy+s*x+c*y] for x,y in points]

    def grid(self,key,msg):
        meta=msg.info if key=='map' else msg.metadata
        width=meta.width if key=='map' else meta.size_x
        height=meta.height if key=='map' else meta.size_y
        if not width or not height:return
        # Retain one owned raw snapshot. The visible renderer creates its indexed
        # image lazily; hidden costmaps never allocate or recolor an image here.
        cells=np.array(msg.data,dtype=np.int8 if key=='map' else np.uint8).reshape(height,width)
        cells.setflags(write=False)
        yaw=(quaternion_rpy(meta.origin.orientation) or (0,0,0))[2]
        origin=[meta.origin.position.x,meta.origin.position.y,yaw]
        try:
            x,y,t=self.transform(msg.header.frame_id,msg.header.stamp)
            ox,oy=origin[:2]
            origin=[x+math.cos(t)*ox-math.sin(t)*oy,y+math.sin(t)*ox+math.cos(t)*oy,t+yaw]
        except Exception:
            with self.s.lock:self.s.data['layers'][key]={'error':'TF 대기','available':False}
            return
        with self.s.lock:
            revision=self.s.touch_map(key)
            self.s.map_layers[key]={'origin':origin,'resolution':meta.resolution,
                                    'width':width,'height':height,'received':time.monotonic(),
                                    'cells':cells,'revision':revision}
            self.s.data['layers'][key]={'available':True,'width':width,'height':height,
                'resolution':meta.resolution,'frame':msg.header.frame_id,'stamp':stamp_seconds(msg.header.stamp)}

    def path(self,msg):
        stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        with self.s.lock:
            if self.s.path_blocked or stamp_ns<self.s.path_start_ns:return
        try:points=self.project([(p.pose.position.x,p.pose.position.y) for p in msg.poses],msg.header.frame_id,msg.header.stamp)
        except Exception:points=[]
        self.assign('path',points)

    def scan(self,msg):
        stamp=stamp_seconds(msg.header.stamp)
        # TF reserves time zero for "latest", not an acquisition timestamp.
        if stamp<=0:return
        if self.latest_scan_stamp is not None and stamp<=self.latest_scan_stamp:
            return
        self.latest_scan_stamp=stamp
        received=self.sample_received if self.sample_received is not None else time.monotonic()
        self.assign('scan_info',{'frame':msg.header.frame_id,'stamp':stamp,
            'range_min':msg.range_min,'range_max':msg.range_max,'angle_min':msg.angle_min,
            'angle_max':msg.angle_max,'angle_increment':msg.angle_increment,'samples':len(msg.ranges)})
        points=[(r*math.cos(msg.angle_min+i*msg.angle_increment),r*math.sin(msg.angle_min+i*msg.angle_increment))
                for i,r in enumerate(msg.ranges) if i%2==0 and math.isfinite(r) and msg.range_min<=r<=msg.range_max]
        # DDS can deliver /scan before the TF for that exact acquisition time.
        # Keep a bounded backlog so a newer scan cannot continually replace an
        # older one just before its TF arrives. Never use a latest-time TF here.
        self.pending_scans.append((points,msg.header.frame_id,msg.header.stamp,received,stamp))
        self.flush_scans()

    def retry_projection(self,samples,displayed,timeout):
        """Project fresh stamped points without blanking on transient TF waits."""
        now=time.monotonic()
        ros=self.get_clock().now().nanoseconds*1e-9
        def fresh(received,stamp):
            return (0<=now-received<timeout and -1e-6<=ros-stamp<timeout)
        pending=deque(maxlen=samples.maxlen)
        candidate=None
        error=''
        for sample in samples:
            points,frame,header_stamp,received,stamp=sample
            if not fresh(received,stamp):continue
            try:
                # An explicit empty observation clears the overlay even when
                # TF lags: there are no coordinates that need transforming.
                projected=self.project(points,frame,header_stamp) if points else []
            except (TransformException,ValueError) as exc:
                pending.append(sample)
                error=str(exc)
            else:
                candidate=(projected,received,stamp)
        update=None
        if candidate is not None:
            points,received,stamp=candidate
            displayed=(received,stamp)
            update=points
            # A late transform must never replace a newer displayed frame.
            pending=deque((sample for sample in pending if sample[4]>stamp),maxlen=pending.maxlen)
        if displayed is not None and not fresh(*displayed):
            displayed=None
            update=[]
        return pending,displayed,update,error

    def flush_scans(self):
        self.pending_scans,self.displayed_scan,update,error=self.retry_projection(
            self.pending_scans,self.displayed_scan,self.SCAN_DISPLAY_TIMEOUT)
        if update is not None:self.assign('scan',update)
        valid=self.displayed_scan is not None
        with self.s.lock:
            info=self.s.data.get('scan_info')
            if info is not None:
                info=dict(info,display_status='tf_wait' if self.pending_scans else ('live' if valid else 'stale'),
                          display_stamp=self.displayed_scan[1] if valid else None,
                          tf_error=error if self.pending_scans else '')
                self.s.data['scan_info']=info

    def cloud(self,key,msg):
        # Full cloud conversion only while its detail view is visible.
        if key=='points' and self.frame_visible!='perception':return
        stamp=stamp_seconds(msg.header.stamp)
        if stamp<=0 or stamp<=self.latest_cloud_stamps.get(key,-1.):return
        received=self.sample_received if self.sample_received is not None else time.monotonic()
        arr=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True)
        if len(arr)>1500:
            arr=arr[np.linspace(0,len(arr)-1,1500,dtype=int)]
        points=[(float(p[0]),float(p[1])) for p in arr]
        self.latest_cloud_stamps[key]=stamp
        with self.s.lock:
            info=dict(self.s.data.get('cloud_info',{}))
            info[key]={'frame':msg.header.frame_id,'stamp':stamp,'samples':len(points)}
            self.s.data['cloud_info']=info
        self.pending_clouds.setdefault(key,deque(maxlen=4)).append(
            (points,msg.header.frame_id,msg.header.stamp,received,stamp))
        self.flush_clouds(key)

    def flush_clouds(self,key=None):
        for name in ([key] if key is not None else list(self.pending_clouds)):
            pending,displayed,update,error=self.retry_projection(
                self.pending_clouds[name],self.displayed_clouds.get(name),self.CLOUD_DISPLAY_TIMEOUT)
            self.pending_clouds[name]=pending
            self.displayed_clouds[name]=displayed
            if update is not None:self.assign(name,update)
            with self.s.lock:
                info=dict(self.s.data.get('cloud_info',{}))
                info[name]=dict(info.get(name,{}),
                    display_status='tf_wait' if pending else ('live' if displayed is not None else 'stale'),
                    display_stamp=displayed[1] if displayed is not None else None,
                    tf_error=error if pending else '')
                self.s.data['cloud_info']=info

    def camera(self,key,msg):
        # Both live previews are visible on the simplified map page.
        if self.frame_visible not in ('camera','driving'):return
        if key=='depth':
            array=depth_array(msg);rgb=depth_rgb(array)
            with self.s.lock:
                self.s.depth=array
                center=float(array[array.shape[0]//2,array.shape[1]//2])
                self.s.data['depth_info']={'unit':'m','valid_pixels':int(np.isfinite(array).sum()),
                    'pixels':int(array.size),'center_m':center if math.isfinite(center) else None}
        else:
            channels=4 if msg.encoding.lower() in ('rgba8','bgra8') else 3
            if msg.encoding.lower() not in ('rgb8','bgr8','rgba8','bgra8','r8g8b8'):
                raise ValueError('지원하지 않는 색상 encoding '+msg.encoding)
            if msg.step<msg.width*channels or len(msg.data)<msg.step*msg.height:raise ValueError('잘못된 영상 크기')
            arr=np.ndarray((msg.height,msg.width,channels),dtype=np.uint8,buffer=bytes(msg.data),
                           strides=(msg.step,channels,1))[:,:,:3]
            rgb=np.ascontiguousarray(arr[:,:,::-1] if msg.encoding.lower().startswith('bgr') else arr)
        image=QImage(rgb.data,msg.width,msg.height,msg.width*3,QImage.Format_RGB888).copy()
        with self.s.lock:
            self.s.images[key]=image
            rev=self.s.data['cameras'].get(key,{}).get('revision',0)+1
            self.s.data['cameras'][key]={'width':msg.width,'height':msg.height,'encoding':msg.encoding,
                'frame':msg.header.frame_id,'revision':rev,
                'received':self.sample_received if self.sample_received is not None else time.monotonic(),
                'stamp':stamp_seconds(msg.header.stamp)}

    def info(self,key,msg):
        with self.s.lock:self.s.data.setdefault('intrinsics',{})[key]={'k':[float(value) for value in msg.k],'d':[float(value) for value in msg.d],
            'r':[float(value) for value in msg.r],'p':[float(value) for value in msg.p],
            'distortion_model':msg.distortion_model,'frame':msg.header.frame_id,'width':msg.width,'height':msg.height}

    def request(self,client,request,key='response',metadata=None):
        if not client.service_is_ready():
            message='서비스 준비 안 됨: '+client.srv_name
            self.assign(key,dict(metadata or {},success=False,message=message,_received=time.monotonic()))
            self.s.event(message);return
        future=client.call_async(request)
        def complete(f):
            try:
                data=plain(f.result());data['_received']=time.monotonic();data.update(metadata or {});self.assign(key,data)
                self.s.event(data.get('message','요청 응답'))
            except Exception as exc:
                message='요청 실패: '+str(exc)
                self.assign(key,dict(metadata or {},success=False,message=message,_received=time.monotonic()))
                self.s.event(message)
        future.add_done_callback(complete)

    def dispatch(self,kind,data):
        if kind=='control':
            self.debug_event('control_requested',command=data.get('command',''),mode=data.get('mode',''))
            req=Control.Request()
            for k,v in data.items():setattr(req,k,v)
            self.request(self.control_client,req,'control_response',{'_command':req.command,'_mode':req.mode,
                         '_request_id':req.request_id})
        elif kind=='stop':
            self.debug_event('stop_requested')
            self.request(self.stop_client,Trigger.Request())
        elif kind=='speed':
            req=SetSpeedLimit.Request()
            for k,v in data.items():setattr(req,k,v)
            self.request(self.speed_client,req,'speed_response',{'_request_id':req.request_id})
        elif kind=='maps':
            req=MapCommand.Request()
            for k,v in data.items():
                if not k.startswith('_'):setattr(req,k,v)
            self.request(self.maps_client,req,'map_response',{'_request_id':data.get('_request_id'),
                         '_operation':req.operation})
        elif kind=='keepouts':
            req=SetKeepoutZones.Request(token=data['token'],epoch=data['epoch'],
                                      expected_revision=data['expected_revision'])
            req.polygons=[Polygon(points=[Point32(x=float(x),y=float(y),z=0.) for x,y in points])
                          for points in data['polygons']]
            self.request(self.keepouts_client,req,'keepout_response',{'_request_id':data['_request_id']})
        elif kind=='cancel':
            self.debug_event('cancel_requested')
            if self.s.goal_handle:self.s.goal_handle.cancel_goal_async()
        elif kind=='navigate':self.start_goal(data)
        elif kind=='view':self.frame_visible=data

    def start_goal(self,data):
        request_id=uuid.uuid4().hex
        self.debug_event('goal_requested',request_id,x=data['x'],y=data['y'],yaw=data['yaw'],
                         frame_id='map',stamp={'sec':data['stamp'].sec,'nanosec':data['stamp'].nanosec})
        def finish(value):
            self.debug_event('goal_result',request_id,result=value)
            self.assign('goal_result',value)
        if not self.navigate.server_is_ready():
            self.s.event('목표 서버 준비 안 됨')
            finish({'success':False,'message':'목표 서버 준비 안 됨','status':6})
            return
        if self.s.goal_active:
            self.s.event('기존 목표를 먼저 중지하세요')
            self.debug_event('goal_result',request_id,result={'success':False,'message':'기존 목표 활성','status':6})
            return
        goal=OperatorNavigate.Goal()
        goal.token=data['token'];goal.epoch=data['epoch'];goal.pose.header.frame_id='map'
        goal.pose.header.stamp=data['stamp'];goal.pose.pose.position.x=data['x'];goal.pose.pose.position.y=data['y']
        goal.pose.pose.orientation.z=math.sin(data['yaw']/2);goal.pose.pose.orientation.w=math.cos(data['yaw']/2)
        self.s.goal_active=True
        with self.s.lock:
            self.s.path_start_ns=self.get_clock().now().nanoseconds
            self.s.path_blocked=False
            self.assign('path',[])
        self.assign('goal_result',{})
        try:
            future=self.navigate.send_goal_async(goal,feedback_callback=lambda f:self.assign('goal_feedback',plain(f.feedback)))
        except Exception as exc:
            self.s.goal_active=False
            message='목표 전송 실패: '+str(exc)
            self.s.event(message)
            finish({'success':False,'message':message,'status':6})
            return
        def accepted(f):
            try:
                handle=f.result();self.s.goal_handle=handle
                self.debug_event('goal_response',request_id,accepted=handle.accepted,
                                 action_goal_id=bytes(handle.goal_id.uuid).hex())
                if not handle.accepted:
                    self.s.goal_active=False;finish({'success':False,'message':'목표 거부','status':6});return
                self.s.event('목표 수락됨')
                def result(r):
                    try:
                        data=plain(r.result().result)
                        finish(data);self.s.event(data.get('message','목표 종료'))
                    except Exception as exc:
                        message='목표 결과 오류: '+str(exc)
                        self.s.event(message)
                        finish({'success':False,'message':message,'status':6})
                    self.s.goal_active=False;self.s.goal_handle=None
                handle.get_result_async().add_done_callback(result)
            except Exception as exc:
                self.s.goal_active=False
                message='목표 전송 실패: '+str(exc)
                self.s.event(message)
                finish({'success':False,'message':message,'status':6})
        future.add_done_callback(accepted)

    def pump(self):
        self.flush_pending()
        self.flush_scans()
        self.flush_clouds()
        with self.s.lock:
            hb,self.s.latest_heartbeat=self.s.latest_heartbeat,None
            manual,self.s.latest_manual=self.s.latest_manual,None
            commands=list(self.s.controls);self.s.controls.clear()
        if hb:self.heartbeat_pub.publish(hb)
        if manual:self.manual_pub.publish(manual)
        for kind,data in commands:
            try:self.dispatch(kind,data)
            except Exception as exc:self.s.event('조작 실패: '+str(exc))
        now=time.monotonic();ros=self.get_clock().now()
        with self.s.lock:self.s.data['ros_time']=ros.nanoseconds*1e-9
        if now-self.last_history>.1:
            self.last_history=now
            try:
                tr=self.tf.lookup_transform('map','base_footprint',Time()).transform
                stamped=self.tf.lookup_transform('map','base_footprint',Time())
                valid=abs(ros.nanoseconds*1e-9-stamp_seconds(stamped.header.stamp))<.5
                rpy=quaternion_rpy(tr.rotation)
                pose=[tr.translation.x,tr.translation.y,rpy[2]] if valid and rpy else None
            except Exception:pose=None
            with self.s.lock:
                self.s.data['pose']=pose
                self.s.data['pose_received']=now if pose else 0.
                if pose and (not self.s.data['trace'] or self.s.data['trace'][-1]!=pose[:2]):
                    self.s.data['trace'].append(pose[:2]);del self.s.data['trace'][:-1200]
                    self.s.touch_map('trace')
                if self.s.history_enabled:
                    imu=self.s.data['imu'].get('gyro',[0,0,0])
                    joints=self.s.data['joints']
                    self.s.data['history'].append({'t':now,'velocity':copy.deepcopy(self.s.data['velocity']),
                        'gyro':imu,'gyro_base':self.s.data.get('imu_base',{}).get('gyro',[]),'rpm':[joints.get(k,{}).get('rpm',0) for k in ('left_wheel_joint','right_wheel_joint')]})
                    del self.s.data['history'][:-600]
        if now-self.last_graph>2.:
            self.last_graph=now
            names=self.get_node_names_and_namespaces()
            self.assign('nodes',[ns.rstrip('/')+'/'+name for name,ns in names])
            self.assign('services',{'control':self.control_client.service_is_ready(),
                'maps':self.maps_client.service_is_ready(),'navigate':self.navigate.server_is_ready()})
