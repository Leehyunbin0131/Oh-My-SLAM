"""Main-thread presentation and input sampling. The ROS worker never repeats intent."""
import json
import math
import socket
import time
import uuid
from pathlib import Path
from PySide6.QtCore import QObject, Property, Signal, Slot, QTimer, Qt
from omslam_operator_msgs.msg import ManualCommand, OwnerHeartbeat
from .geometry import world_grid
from omslam_navigation.keepouts import grid_goal_error, pose_allowed
from .presentation import presentation_snapshot


def number(value, digits=2):
    return '—' if value is None or not math.isfinite(float(value)) else f'{value:.{digits}f}'


class ConsoleModel(QObject):
    changed=Signal()
    def __init__(self,shared,initial_page=0):
        super().__init__()
        self.s=shared;self._data={};self._page=initial_page;self._sensor=0;self._imu_frame=0
        self.token='';self.epoch=0;self.owner=socket.gethostname()+'-'+uuid.uuid4().hex[:8]
        self.sequence=0;self.deadman=False;self.manual_v=0.;self.manual_w=0.;self.active=True
        self._goal={'x':0.,'y':0.,'yaw':0.,'valid':False,'reason':'지도에서 위치와 방향을 지정하세요'}
        self.queue=[];self.queue_running=False;self.queue_waiting=False;self.queue_result=None;self.queue_index=None
        self.response_seen=None;self.result_seen=None;self._map_path=''
        self._last_state_owner=None;self.pending_mode=None;self.guard_session=None;self._control_state={}
        self.timer=QTimer(self);self.timer.setTimerType(Qt.PreciseTimer);self.timer.setInterval(50);self.timer.timeout.connect(self.tick);self.timer.start()
        self.display_timer=QTimer(self);self.display_timer.setInterval(100);self.display_timer.timeout.connect(self.refresh);self.display_timer.start()
        self.command('view','driving' if initial_page==0 else ('camera' if initial_page==1 else 'status'))
        self.refresh()

    @Property('QVariantMap',notify=changed)
    def data(self):return self._data
    @Property('QVariantMap',notify=changed)
    def goal(self):return self._goal
    @Property('QVariantList',notify=changed)
    def goals(self):return self.queue
    @Property(int,notify=changed)
    def page(self):return self._page
    @Property(int,notify=changed)
    def sensorPage(self):return self._sensor
    @Property(int,notify=changed)
    def imuFrame(self):return self._imu_frame
    @Slot(int)
    def setImuFrame(self,value):self._imu_frame=value;self.refresh()
    @Property(bool,notify=changed)
    def ownsControl(self):return bool(self.token)
    @Property(bool,notify=changed)
    def queueRunning(self):return self.queue_running
    @Property(bool,notify=changed)
    def queueBusy(self):return self.queue_running or self.queue_waiting
    @Property(str,notify=changed)
    def mapPath(self):return self._map_path

    def command(self,kind,data):
        with self.s.lock:self.s.controls.append((kind,data))

    @Slot(int)
    def setPage(self,value):
        self.clearInput();self._page=value;self.command('view',('camera','imu','perception')[self._sensor] if value==1 else ('driving' if value==0 else 'status'));self.changed.emit()

    @Slot(int)
    def setSensorPage(self,value):
        self._sensor=value;self.command('view',('camera','imu','perception')[value]);self.changed.emit()

    @Slot(bool)
    def setActive(self,value):
        self.active=value
        if not value:self.clearInput()

    @Slot(float,float,bool)
    def manual(self,v,w,pressed):
        op=self._control_state
        if not self.active or not self.token or op.get('mode')!='MANUAL' or op.get('phase')!='ACTIVE':
            pressed=False
        if v<0 and not op.get('reverse_allowed',False):v=0.
        self.deadman=bool(pressed)
        self.manual_v=max(-1.,min(1.,v));self.manual_w=max(-1.,min(1.,w))
        self.sample_input()

    @Slot()
    def clearInput(self):
        self.deadman=False;self.manual_v=self.manual_w=0.;self.sample_input()

    def now(self):
        node=self.s.node
        return node.get_clock().now().to_msg() if node else None

    def sample_input(self):
        if not self.token:return
        stamp=self.now()
        if stamp is None:return
        op=self._control_state
        self.sequence+=1
        msg=ManualCommand();msg.token=self.token;msg.epoch=self.epoch;msg.sequence=self.sequence
        msg.command.header.stamp=stamp;msg.command.header.frame_id='base_footprint'
        msg.deadman=self.deadman and self.active and self._page==0 and op.get('mode')=='MANUAL'
        if msg.deadman:
            msg.command.twist.linear.x=self.manual_v*float(op.get('linear_limit',.1))
            msg.command.twist.angular.z=self.manual_w*float(op.get('angular_limit',.3))
        with self.s.lock:self.s.latest_manual=msg

    @Slot()
    def acquireControl(self):
        """Acquire an observing owner session without enabling either motion mode."""
        if self.token:
            return
        self.pending_mode=None
        self.command('control',{'command':'acquire','owner_id':self.owner,'token':'','epoch':0,
                               'request_id':uuid.uuid4().hex,'mode':'DISABLED'})

    @Slot(str)
    def control(self,mode):
        self.clearInput();self._halt_queue('모드 전환 요청')
        command='release' if mode=='DISABLED' else ('set_mode' if self.token else 'acquire')
        self.pending_mode=mode if command=='acquire' else None
        self.command('control',{'command':command,'owner_id':self.owner,'token':self.token,'epoch':self.epoch,
                               'request_id':uuid.uuid4().hex,'mode':'DISABLED' if command=='acquire' else mode})

    @Slot()
    def stop(self):
        self.clearInput();self._halt_queue('정지 요청')
        with self.s.lock:
            self.s.drop_queued_requests()
            self.s.cancel_unsent_goal()
            self.s.controls.clear();self.s.controls.appendleft(('stop',{}))
        self.s.event('정지·구동해제 요청')

    @Slot(float,float,float)
    def speed(self,linear,angular,percent):
        if not self.token:return
        self.command('speed',{'token':self.token,'epoch':self.epoch,'request_id':uuid.uuid4().hex,
            'linear_limit':float(linear),'angular_limit':float(angular),'speed_percent':float(percent)})

    @Slot(float,float,float)
    def setGoal(self,x,y,yaw):
        valid=all(math.isfinite(v) for v in (x,y,yaw));reason=''
        if valid:
            with self.s.lock:grid=self.s.map_layers.get('map')
            if not grid:valid=False;reason='지도를 기다리는 중입니다'
            else:
                u, v = world_grid(x, y, grid['resolution'], grid['origin'])
                i, j = math.floor(u), math.floor(v)
                if not (0 <= i < grid['width'] and 0 <= j < grid['height']):
                    reason = '지도 밖의 목표입니다'
                elif grid['cells'][j, i] < 0:
                    reason = '관측되지 않은 목표입니다'
                elif grid['cells'][j, i] >= 65:
                    reason = '점유된 목표입니다'
                reason = reason or grid_goal_error(x, y, yaw, grid['origin'], grid['resolution'],
                                         grid['width'], grid['height'], grid['cells'].ravel())
                if not reason and not pose_allowed(getattr(self, '_keepout_polygons', []), x, y, yaw):
                    reason = '차체가 진입 금지 구역과 겹칩니다'
                valid = not reason
        self._goal={'x':float(x),'y':float(y),'yaw':float(yaw),'valid':valid,'reason':reason or '위치·도착 방향 확인 후 이동 시작'}
        self.changed.emit()

    @Slot()
    def addGoal(self):
        if self._goal['valid'] and not self.queueBusy:
            self.queue.append(dict(self._goal,status='대기'));self.changed.emit()

    @Slot(int)
    def removeGoal(self,index):
        if not self.queueBusy and 0<=index<len(self.queue):self.queue.pop(index);self.changed.emit()

    @Slot()
    def navigate(self):
        if self._goal['valid']:self.sendGoal(self._goal)

    def sendGoal(self,goal):
        op=self._control_state
        if not self.token or op.get('mode')!='AUTO' or op.get('phase')!='ACTIVE':
            self.s.event('자율 모드를 활성화한 뒤 목표를 전송하세요');return False
        if self.s.goal_active:self.s.event('기존 목표를 먼저 중지하세요');return False
        stamp=self.now()
        if stamp is None:return False
        # Clear the previous terminal result before this request becomes visible to
        # the worker; a delayed send must not complete the new queue row with old data.
        with self.s.lock:
            self.s.data['goal_result']={}
            self.s.goal_request = uuid.uuid4().hex
            self.s.goal_phase = 'queued'
            self.s.controls.append(('navigate',{'token':self.token,'epoch':self.epoch,'stamp':stamp,
                'request_id':self.s.goal_request,'x':goal['x'],'y':goal['y'],'yaw':goal['yaw']}))
        self.result_seen=None;return True

    @Slot()
    def runQueue(self):
        if not self.queue or self.queueBusy:return
        self.queue_running=True;self.queue_waiting=False;self.queue_result=None;self.queue_index=None
        for item in self.queue:item['status']='대기'
        self.nextGoal()

    def nextGoal(self):
        if not self.queue_running:return
        for index,goal in enumerate(self.queue):
            if goal['status']=='대기':
                if self.sendGoal(goal):goal['status']='이동 중';self.queue_waiting=True;self.queue_index=index
                else:self.queue_running=False
                return
        self.queue_running=False

    @Slot()
    def cancel(self):
        self._halt_queue('취소 요청')
        with self.s.lock:
            self.s.cancel_request = self.s.goal_request
            self.s.cancel_unsent_goal()
            self.s.controls = type(self.s.controls)(
                ((k, d) for k, d in self.s.controls if k != 'navigate'), maxlen=self.s.controls.maxlen)
            if self.s.goal_phase != 'terminal':
                self.command('cancel',{'request_id':self.s.goal_request})
        self.s.event('현재 목표 취소 요청')

    def _halt_queue(self,reason):
        # Stop permission to advance, but retain the in-flight result/stop observer.
        self.queue_running=False
        if self.queue_waiting and self.queue_index is not None:
            self.queue[self.queue_index]['status']=reason+' · 정지 확인 중'

    def _queue_stopped(self,op):
        if not op.get('stopped',False):return False
        now=time.monotonic()
        with self.s.lock:
            topics=self.s.data.get('topics',{})
            if now-topics.get('/operator/state',{}).get('received',0)>.3:return False
            ros=self.s.data.get('ros_time',0.)
            velocities=self.s.data.get('velocity',{})
            for topic,timeout in (('/odom',.2),('/wheel/odometry',.4)):
                observation=topics.get(topic,{})
                stamp=observation.get('stamp')
                if (now-observation.get('received',0)>timeout or stamp is None
                        or not 0.<=ros-stamp<=timeout):return False
                value=velocities.get(topic,{})
                v,w=value.get('v'),value.get('w')
                if (v is None or w is None or not math.isfinite(v) or not math.isfinite(w)
                        or abs(v)>.01 or abs(w)>.02):return False
        return True

    @Slot(str)
    def setMapPath(self,path):self._map_path=path;self.changed.emit()

    @Slot(str)
    def mapOperation(self,operation):
        goal=self._goal
        self.clearInput()
        self.command('maps',{'operation':operation,'path':self._map_path,'x':goal['x'],'y':goal['y'],'yaw':goal['yaw'],
            'session_id':self.owner,'token':self.token,'epoch':self.epoch})

    @Slot(float,float,result=str)
    def depthAt(self,x,y):
        with self.s.lock:
            arr=self.s.depth
            if arr is None:return '깊이 영상 미수신'
            row=min(arr.shape[0]-1,max(0,int(y*arr.shape[0])));col=min(arr.shape[1]-1,max(0,int(x*arr.shape[1])))
            v=float(arr[row,col])
        return f'픽셀 {col}, {row} · '+(f'{v:.3f} m' if math.isfinite(v) else '유효 깊이 없음')

    @Slot(str)
    def exportDiagnostics(self,path):
        try:
            p=Path(path).expanduser();p.parent.mkdir(parents=True,exist_ok=True)
            with p.open('x',encoding='utf-8') as f:json.dump(self._data,f,ensure_ascii=False,indent=2)
            self.s.event('진단 저장: '+str(p))
        except Exception as exc:self.s.event('진단 저장 실패: '+str(exc))

    def tick(self):
        # This timer is the UI liveness source. It must never perform QML conversion,
        # plot construction, snapshot serialization, or screen rendering.
        with self.s.lock:
            response=self.s.data.get('control_response',{})
            op=self.s.data.get('operator',{})
            state_received=self.s.data.get('topics',{}).get('/operator/state',{}).get('received',0)
            result=self.s.data.get('goal_result',{})
        self._sync_control(response,op,state_received)
        self._control_state=op
        if self.token:
            stamp=self.now()
            if stamp:
                self.sequence+=1
                hb=OwnerHeartbeat();hb.header.stamp=stamp;hb.token=self.token;hb.epoch=self.epoch;hb.sequence=self.sequence
                with self.s.lock:self.s.latest_heartbeat=hb
            if op.get('mode')=='MANUAL':self.sample_input()
        if self.queue_waiting and result and not self.s.goal_active:
            success=bool(result.get('success',False))
            if not success:self.queue_running=False
            row=self.queue[self.queue_index] if self.queue_index is not None else None
            if self._queue_stopped(op):
                if row is not None:
                    row['status']='완료' if success else ('취소됨' if result.get('status')==5 else '실패')
                self.queue_waiting=False;self.queue_index=None
                if success and self.queue_running:self.nextGoal()
            elif row is not None:
                row['status']=('도착' if success else '목표 종료')+' · 정지 확인 중'

    def _sync_control(self,response,op,state_received):
        serial=repr(response)
        if serial!=self.response_seen and response:
            self.response_seen=serial
            if response.get('success'):
                self.token=response.get('token','');self.epoch=response.get('epoch',0)
                if response.get('_command')=='acquire' and self.pending_mode:
                    mode=self.pending_mode;self.pending_mode=None
                    self.command('control',{'command':'set_mode','owner_id':self.owner,'token':self.token,
                        'epoch':self.epoch,'request_id':uuid.uuid4().hex,'mode':mode})
        session=op.get('session_id')
        if session and self.guard_session and session!=self.guard_session:
            self.token='';self.deadman=False;self.queue_running=False;self.queue_waiting=False;self.queue_index=None;self.queue.clear();self.pending_mode=None
            self.s.event('제어 서버가 다시 시작되었습니다. 관측 모드로 연결합니다')
        if session:self.guard_session=session
        if self.token and op.get('owner_id') not in ('',self.owner) and op.get('phase')!='TRANSITION':
            self.token='';self.deadman=False
        # Only clear a confirmed lost session after a fresh owner state supersedes its response.
        if self.token and op.get('owner_id')=='' and op.get('phase') in ('BLOCKED','READY'):
            received=state_received
            if received>response.get('_received',float('inf')):self.token='';self.deadman=False

    def refresh(self):
        with self.s.lock:
            d=presentation_snapshot(self.s.data,self._page,self._sensor,self._imu_frame)
        op=d.get('operator',{})
        now=time.monotonic();ros=d.get('ros_time',0)
        rows=[]
        for topic,item in d['topics'].items():
            age=max(now-item['received'], max(0,ros-item['stamp']) if item.get('stamp') and topic!='/diagnostics' else 0)
            item['age']=age;item['status']='정상' if age<(.8 if topic not in ('/map','/diagnostics') else 3.) else '지연'
            rows.append({'name':topic,'detail':f"{item['hz']:.1f} Hz  ·  {age:.2f} s  ·  {item['frame'] or '—'}",
                         'status':item['status'],'type':item['type']})
        d['topicRows']=rows if self._page==3 else []
        d['diagnosticRows']=list(d['diagnostics'].values())
        d['operatorConnected']=now-d['topics'].get('/operator/state',{}).get('received',0)<1.
        d['connected']=d['operatorConnected'] or any(item.get('age',999)<1. for topic,item in d['topics'].items() if topic in ('/odom','/wheel/odometry','/imu/data','/scan','/camera/color/image_raw'))
        d['ownerSelf']=self.owner;d['hasToken']=bool(self.token)
        d['goalActive']=self.s.goal_active
        d['actualV']=number(d['velocity'].get('/odom',{}).get('v'))
        d['actualW']=number(d['velocity'].get('/odom',{}).get('w'))
        d['operatorReason']=op.get('reason','센서 관측 연결됨 · 제어 서버 미연결' if d['connected'] else '연결 대상의 ROS 데이터를 기다리는 중입니다')
        d['modeText']={'MANUAL':'수동','AUTO':'자율','DISABLED':'관측'}.get(op.get('mode'),'관측')
        d['motorRows']=self.motorRows(d) if self._page==2 else []
        self._data=d;self.changed.emit()

    def motorRows(self,d):
        motor=d.get('motor',{});valid=set(motor.get('valid_fields',[]));j=d.get('joints',{})
        rows=[]
        fields=[('목표 RPM','target_rpm'),('MCU 측정 RPM','measured_rpm'),('엔코더 차분 RPM','encoder_rpm'),
                ('바퀴 각도 rad','position_rad'),('바퀴 각속도 rad/s','velocity_rad_s'),('원시 count','raw_counts'),
                ('누적 count','counts'),('count 변화','delta_counts'),('A/B 상태','ab'),('invalid 전이','invalid_transitions'),
                ('PWM','pwm'),('정지 수렴','settled'),('출력 제한','limited'),('위치 오차','position_error'),('바퀴 상태','wheel_states')]
        for title,key in fields:
            values=motor.get(key,[]) if key in valid else []
            if not values and key in ('position_rad','velocity_rad_s','encoder_rpm'):
                field={'position_rad':'position','velocity_rad_s':'velocity','encoder_rpm':'rpm'}[key]
                values=[j.get(name,{}).get(field) for name in ('left_wheel_joint','right_wheel_joint')]
            def cell(index):
                value=values[index] if index<len(values) else None
                if value is None:return '미제공'
                if isinstance(value,float):return number(value,3)
                return str(value)
            rows.append({'name':title,'left':cell(0),'right':cell(1)})
        for key in ('backend','source','mode','fault_reason','sequence','applied_sequence','target_age_ms','ack_status','feedback_age_s'):
            value=motor.get(key) if key in valid or key in ('backend','source','feedback_age_s') else None
            rows.append({'name':key,'left':str(value) if value is not None else '미제공','right':''})
        for item in motor.get('counters',[]):rows.append({'name':item['key'],'left':item['value'],'right':''})
        return rows

    @Slot()
    def shutdown(self):
        if self.token:self.stop()
