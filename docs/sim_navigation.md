# 시뮬레이션 자율주행

[README](../README.md) · [지도 작성·위치추정](sim_mapping.md) · [인수 기준](acceptance.md) · [P4 구현·검증 명세](p4_implementation_plan.md)

`omslam_navigation`은 온라인 SLAM 또는 저장 지도에서 State Lattice 경로를 DWB로 추종합니다. 라이다와 RGB-D 장애물 기억을 비용지도에 반영하고, Collision Monitor와 navigation guard가 관측·명령·실행 상태에 따라 구동을 차단합니다. 이 실행 경로의 대상은 Gazebo 시뮬레이션입니다.

시뮬레이션 최고 속도는 바퀴별 **100 RPM**, 직진 **0.712 m/s**이며 Nav2·velocity smoother·navigation guard에 같은 선속도 상한을 적용합니다. 회전 상한은 0.60 rad/s입니다. 아래 기존 주행 결과는 해당 시험 당시 설정의 근거이며, 최고 속도 변경 후 모든 장애물 코스를 다시 검증했다는 의미는 아닙니다.

## 한 명령으로 실행

직접 RViz에서 목표를 지정하려면 workspace의 실행 스크립트를 사용합니다.

```bash
cd ~/MyRos/Oh_My_SLAM
./nav.sh
```

기본값은 검증된 `p4_course`입니다. Gazebo·온라인 SLAM·Nav2를 시작하고, 준비 상태가 안정된 뒤 구동을 활성화하여 P4용 RViz를 엽니다. **Nav2 Goal**로 관측된 빈 공간의 위치와 도착 방향을 지정합니다. 도착·정지 후 다음 목표를 지정합니다. 스크립트는 목표를 자동 전송하지 않습니다.

`Ctrl+C` 또는 RViz 창 닫기로 종료하면 구동을 해제하고 측정 정지를 확인한 뒤 자신이 시작한 프로세스를 정리합니다. 실행 중 fault는 자동으로 재활성화하지 않고 이유를 출력하며 종료합니다. 같은 ROS domain에 기존 시뮬레이션·구동이 있으면 중복 실행을 거부합니다. 기존 수동 launch는 먼저 해당 터미널에서 종료하세요.

공항 world는 `./nav.sh --world airport`로 선택합니다. 기본 ROS domain은 `86`이며 `OMSLAM_NAV_DOMAIN_ID`로 바꿀 수 있습니다. 로그와 시작·종료 결과는 `reports/navigation/<실행시각>/`에 저장합니다. 데스크톱 터미널에서 실행하며, 설치본이 없다면 아래 빌드 절차를 먼저 따릅니다.

## 구성과 입력

| 역할 | 구성 |
| --- | --- |
| 전역 계획 | `nav2_smac_planner::SmacPlannerLattice`, diff·0.05 m·16 heading·112 primitives |
| 지역 제어 | `dwb_core::DWBLocalPlanner`, 전진·제자리 회전, `ObstacleFootprint` 검사 |
| 도착 판정 | `StoppedGoalChecker`, 위치 0.10 m·yaw 0.15 rad 이내와 측정 속도 0.01 m/s·0.02 rad/s 이하를 함께 확인 |
| 비용지도 | 전역 static→lidar→camera memory→inflation, 지역 lidar→camera memory→inflation |
| 인지 | depth·CameraInfo·유효 IMU 자세로 바닥 분리, `odom` 기준 장애물 기억 |
| 명령 | `/cmd_vel/nav`→guard→`/cmd_vel/requested`→smoother→`/cmd_vel/smoothed`→monitor→`/cmd_vel/monitored`→guard→`/cmd_vel`→sim base |
| 위치추정 | EKF의 `odom → base_footprint`, SLAM 또는 AMCL 하나의 `map → odom` |
| 구동 권한 | `/navigation/enable` 서비스, 최신 관측·서버 준비·base 활성화 후 새 목표 필요 |

공통 설정은 `config/nav2_common.yaml`, 시뮬레이션 속도·시간 값은 `config/sim_profile.yaml`, 인지는 `config/perception.yaml`에 있습니다. launch가 설치된 설정을 병합하고 lattice·BT의 절대 설치 경로를 넣습니다. `nav2_params_file`은 이 기본 설정에 병합할 override이며 플러그인·기하·시간 계약을 깨는 값은 시작 시 거부합니다. SLAM/AMCL 설정은 별도의 `backend_params_file`로 전달합니다.

목표 접근과 방향 정렬의 위치 기준은 `controller_server.goal_checker.xy_goal_tolerance` 한 곳에서 설정합니다. launch가 같은 값을 `GridBased.xy_goal_tolerance`와 `FollowPath.xy_goal_tolerance`에 전달하고, `goal_checker.trans_stopped_velocity`도 DWB에 전달합니다. 기본 위치 범위는 10 cm이며 이 안에서는 전역계획기가 원래 목표 좌표·방향을 유지한 짧은 마무리 경로를 반환합니다. 현재 위치의 제자리 회전과 목표까지의 연결 구간 전체를 footprint로 검사하고, 충돌하면 계획 실패를 반환합니다. 범위 밖은 기존 Lattice 탐색을 사용하되 경로 끝점을 원래 목표에 맞추고 짧은 연결 구간을 검사합니다. 연결 길이는 위치 허용오차와 격자 한 셀의 대각선 길이 중 큰 값으로 제한합니다. `GridBased.tolerance`는 A* 탐색의 fallback 허용값으로 별도 의미를 가집니다.

위치 범위에 진입해도 즉시 성공하지 않습니다. DWB가 감속·방향 정렬을 수행하고 `StoppedGoalChecker`가 방향 오차와 측정 속도를 확인한 뒤 완료합니다. 이 기준을 변경하려면 도착 판정 설정을 수정하고 세션을 다시 시작합니다. GUI에서 목표가 완료·취소·실패하면 진행 경로 표시를 지우고 이전 경로의 지연 수신을 무시합니다.

전역 계획기는 공식 Nav2 1.3.13 패키지에 연속 좌표 충돌 검사 패치를 적용한 workspace 설치본을 사용합니다. 격자 경계를 넘을 때 footprint 검사 위치가 셀 중심으로 이동하지 않도록 하며, 실제 장애물·unknown 거부 기준은 유지합니다. 원본 커밋·파일 해시·패치는 [기술 기준](baseline.md#개발-pc-설치-버전)과 패키지의 `provenance.json`에 기록합니다.

TF 발행 책임은 고정된 launch 구성과 실제 endpoint 목록을 함께 확인합니다. `/tf`는 `ekf_filter_node`, `robot_state_publisher`, 선택한 `slam_toolbox` 또는 `amcl` 노드가 각각 하나씩 있어야 하며, `/tf_static`은 `robot_state_publisher`만 허용합니다. 한 노드가 여러 broadcaster endpoint를 소유하는 경우와 같은 이름의 노드가 중복 실행된 경우를 구분합니다. 중복 노드·추가 발행 노드가 있으면 구동 준비를 차단합니다. Jazzy의 Python 수신 정보에는 메시지별 sender GID가 없으므로 이를 개별 TF edge의 발행자 추적으로 표시하지 않습니다. edge의 실제 수신 공백·관측 시각 변환·map 보정은 별도로 감시합니다.

## 빌드와 실행 환경

기존 workspace overlay를 source하지 않은 셸에서 의존 패키지까지 빌드합니다.

```bash
cd ~/MyRos/Oh_My_SLAM
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to omslam_navigation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

빌드가 성공한 뒤 새 셸에서 다음 환경을 준비합니다. 명령을 보내는 다른 터미널에도 같은 domain을 설정합니다.

```bash
cd ~/MyRos/Oh_My_SLAM
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=86
export GZ_PARTITION=omslam-navigation
ros2 pkg prefix nav2_smac_planner
```

마지막 명령의 경로는 현재 workspace의 `install/nav2_smac_planner`여야 합니다. `/opt/ros/jazzy`가 출력되면 새 설치본 환경이 적용되지 않은 상태입니다.

## 온라인 SLAM으로 시작

공항의 기본 위치에서 시작합니다. 이미 같은 domain에서 공항 또는 로봇을 실행 중이라면 중복 실행하지 않고 그 실행을 종료하거나 `start_sim:=false`로 재사용합니다.

```bash
ros2 launch omslam_navigation slam_navigation.launch.py
```

작은 인수 코스는 다음과 같이 선택합니다. 이 코스도 동일한 로봇·센서·EKF·구동 경로를 사용합니다.

```bash
ros2 launch omslam_navigation slam_navigation.launch.py \
  world:=p4_course spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0 \
  gui:=false pedestrians:=false
```

온라인 모드에서는 현재 지도에서 관측된 자유 공간의 목표를 사용합니다. 초기 map 좌표와 Gazebo world 좌표를 혼용하지 않습니다. 공항의 `spawn_x=-108`은 world에서 로봇이 생성되는 위치이며, SLAM의 시작 map 위치와 별개입니다.

## 저장 지도로 시작

`omslam_mapping`의 `save_map`으로 저장한 시뮬레이션 map YAML과 metadata를 사용합니다. 지도 이미지·metadata의 해시와 frame/backend가 일치해야 합니다. `map`에는 절대 경로를 지정합니다.

```bash
ros2 launch omslam_navigation navigation.launch.py \
  map:=/absolute/path/to/saved_map.yaml \
  initial_pose:=true initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
```

위 초기 위치는 예시이며 해당 지도에서의 실제 초기 위치를 넣어야 합니다. `initial_pose:=false`이면 RViz의 **2D Pose Estimate** 또는 `/initialpose`로 AMCL 초기 위치를 설정한 뒤 준비 상태를 확인합니다. Gazebo 생성 위치를 바꾸려면 `spawn_x/y/yaw`를 별도로 지정합니다.

두 navigation launch가 backend를 포함하므로 별도의 mapping/localization launch를 동시에 띄우지 않습니다. backend 교체는 아래 정지 절차 후 기존 navigation launch가 종료되고 해당 publisher가 사라진 상태에서 수행합니다. simulator를 유지하려면 simulator를 별도 실행하고 두 navigation 모드에 `start_sim:=false`를 사용합니다.

## 활성화·목표·정지

상태를 먼저 확인합니다.

```bash
ros2 topic echo /navigation/state std_msgs/msg/String --once \
  --qos-durability transient_local
ros2 service call /navigation/enable std_srvs/srv/SetBool '{data: true}'
```

`READY`는 관측·TF·서버가 준비됐다는 뜻이며 구동 권한은 없습니다. enable은 기존 목표가 종료되고 정상 관측이 1초 동안 유지되며 fresh wheel과 base 활성화까지 확인한 뒤 성공합니다. 성공 후 상태는 `ARMED_IDLE`입니다. 실패 응답의 원인과 `missing` 목록을 먼저 해결합니다.

P4용 RViz 설정을 사용합니다. 다음 명령을 같은 ROS domain의 터미널에서 실행합니다.

```bash
rviz2 -d ~/MyRos/Oh_My_SLAM/install/omslam_navigation/share/omslam_navigation/rviz/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

이 설정에는 Navigation 2 패널과 Nav2 Goal 도구가 있습니다. 현재 실행하지 않는 경로 smoothing 서버를 조회하는 Selector와 docking 패널은 포함하지 않습니다. `velocity_smoother`는 명령 속도를 평활하는 별도 구성요소입니다.

이후 RViz의 **Nav2 Goal** 또는 `NavigateToPose` action으로 새 목표를 보냅니다. 다음은 작은 코스에서 시작점 앞 1 m로 이동하는 예입니다.

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}' \
  --feedback
```

목표는 한 번에 하나만 보냅니다. 결과가 terminal이고 guard가 `ARMED_IDLE`이며 차체가 정지한 뒤 다음 목표를 보냅니다. 실행 중 목표 교체나 동시에 들어온 다른 UUID는 차단·취소합니다.

정지하고 구동 권한을 해제하려면 다음 서비스를 사용합니다. 요청 즉시 zero를 발행하고 기존 목표 취소와 base 비활성화를 수행합니다.

```bash
ros2 service call /navigation/enable std_srvs/srv/SetBool '{data: false}'
```

정상 종료·모드 전환 전에는 위 서비스의 성공과 실제 정지를 확인한 뒤 launch를 종료합니다. 실패 응답이면 fault 상태를 유지하며 종료/전환을 성공으로 취급하지 않습니다.

## 장애와 복구

| 상태·원인 | 구동 결과와 복구 |
| --- | --- |
| scan·depth·인지 health가 0.30 s 이상 오래됨 | zero·목표 취소·base 비활성화, 데이터 복구만으로 재출발하지 않음 |
| EKF odom·wheel·TF·lifecycle 이상 | 동일한 fault latch, `missing`과 관련 센서/서버 상태 확인 |
| 원본 이동 명령 만료 | 300 ms 수명 이후 구동 차단, 반복된 평활 출력으로 수명 연장 불가 |
| 장애물에 의한 monitor stop | zero 우선; 새 유효 원본·관측 조건에서 같은 목표의 제어 가능 |
| 제한된 BT 대기 | 정지→2초 대기→재계획 1회, 계속 불가능하면 action 실패 |
| 큰 map 보정 | 연속 표본 간 0.10 m 또는 0.10 rad 이상이면 latch, 같은 세션에서 재활성화 불가 |
| 시뮬레이션 pause·시각 역행 | 과거 명령 폐기, 새 관측·활성화·새 목표 요구 |

일반 관측 장애는 원인 해결 후 기존 목표가 terminal이고 정상 관측이 1초 동안 유지된 것을 확인하여 enable을 다시 요청하고 **새 목표**를 보냅니다. 큰 map 보정은 정지 상태에서 navigation 모드를 종료하고 새 세션의 위치·관측을 확인한 뒤 재활성화합니다. 이전 장애물 기억을 정확히 재구축했다고 간주하지 않습니다.

`queue is full` 같은 경고는 발생 노드·scan timestamp·TF 대기와 함께 확인합니다. 큐·timeout을 무작정 늘리지 않습니다. guard는 도착 순서가 다른 정상 메시지를 작은 대기 버퍼에서 맞추되, 원본 관측의 freshness 기준을 연장하지 않습니다.

## 검증과 결과

현재 P4 인수는 **한 실행에서 목표 5곳에 연속 정상 도착**하면 종료합니다. 최신 빌드와 변경에 직접 관련된 회귀 검사를 통과한 뒤 실행합니다.

이 기준의 검증 결과는 **5/5 연속 도착, 충돌 0회**입니다. 최대 위치 오차는 0.1111 m, 최대 yaw 오차는 0.0100 rad였고 모든 목표에서 정지까지 0.412 s 이내였습니다. 실제 차체의 최소 장애물 여유는 0.1299 m이며, 최대 바퀴 명령은 48.31 RPM으로 95 RPM 한도 안입니다. 마지막 목표 뒤 구동 해제와 프로세스 종료를 확인했습니다.

```bash
/usr/bin/python3 tools/verify_sim_navigation.py \
  --scenario five-goals --output reports/sim-navigation/p4-five-goals
```

같은 코스에서 `(1,0,0)` → `(5,0,π/2)` → `(5,4,π/2)` → `(0,4,−π/2)` → `(0,0,0)`을 순서대로 요청합니다. 좌표는 시작 시 고정한 기준의 `(x m, y m, yaw rad)`입니다. 각 목표는 action 성공 후 측정 정지와 truth 기준 위치 오차 ≤0.15 m·yaw 오차 ≤0.20 rad를 확인합니다. 센서·TF·인지·명령·충돌 관측을 같은 실행에 유지하며, 5개 통과 뒤 구동 권한을 해제하고 실행을 정리합니다.

기존 `integrated` 모드는 전체 기능·실패 주입 검증을 위한 선택 도구입니다. 그 전체 항목을 5목표 인수 뒤 추가로 수행할 필요는 없습니다. 비주행 계약 도구 `tools/verify_nav_contracts.py`는 실제 설치 플러그인의 충돌 거부와 명령·메모리 경계 조건의 근거를 제공합니다.

각 결과 디렉터리의 `manifest.json`, `results.json`, `metrics.json`, `events.jsonl`, 로그와 경로 그림이 판정 근거입니다. 완료 결과를 덮어쓰지 않고 새 output 경로를 사용합니다. 실패 기록을 보존하고, 이전 실행의 성공을 합쳐 연속 5회 성공으로 세지 않습니다.

## 장애 후 명시적 복구

`nav.sh`는 실행 중 일반 `FAULT_LATCHED`가 발생해도 Gazebo·Nav2·RViz를 유지합니다. 원인을 해소하고 최신 정지·관측 및 이전 목표 종료 조건이 충족된 뒤, 터미널에 표시된 `/navigation/enable` 호출로 재활성화하고 새 목표를 지정합니다. 자동 재출발이나 이전 목표 재개는 하지 않습니다. 큰 `map→odom` 보정처럼 `restart_required=true`인 경우에는 종료 후 내비게이션 세션을 재초기화합니다. 준비 시간 초과와 실제 자식 프로세스 종료는 계속 오류로 처리합니다.

전역·지역 inflation 반경은 `0.55 m`이며, padding을 포함한 외접 반경 약 `0.4965 m`와 한 셀(`0.05 m`) 여유를 포함합니다. GUI의 전체 `costmap_raw` 표시를 위해 `always_send_full_costmap=true`를 유지합니다. 계산 bounds 축소와 메시지 발행 방식은 별개의 계약입니다.
