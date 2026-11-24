# 시뮬레이션 자율주행

[README](../README.md) · [통합 설계](architecture.md) · [지도 작성·위치추정](sim_mapping.md) · [인수 기준](acceptance.md)

`omslam_navigation`은 온라인 SLAM이나 저장 지도 위에서 State Lattice로 전역 경로를 만들고 DWB로 따라가는 패키지입니다. 라이다 스캔과 RGB-D 낮은 장애물 기억을 비용지도에 올리고, `navigation_guard`가 센서 관측과 명령, 실행 상태를 보고 이상이 생기면 구동을 차단합니다. 이 문서는 Gazebo 시뮬레이션 실행 경로를 기준으로 정리했습니다.

Gazebo 주행 범위는 바퀴별 **3–70 RPM**입니다. 상한 기준 직진은 약 **0.4985 m/s**, 회전은 **0.50 rad/s**이고, 두 바퀴 중 큰 쪽이 **3.0 RPM** 미만인 작은 명령은 끌어올리지 않고 0으로 끊습니다. GUI는 **35 RPM**으로 시작하며 **3–70 RPM** 범위 안에서 0.1 RPM 간격 막대나 숫자 입력으로 바꿀 수 있습니다.

## 한 명령으로 실행

RViz에서 직접 목표를 찍어 보고 싶을 때는 워크스페이스 루트의 스크립트를 쓰면 편합니다.

```bash
cd ~/MyRos/Oh-My-SLAM
./nav.sh
```

기본 월드는 검증용 `p4_course`입니다. Gazebo와 온라인 SLAM, Nav2를 차례로 띄우고 준비 상태가 안정되면 구동을 열어 P4용 RViz를 실행합니다. RViz의 **Nav2 Goal** 도구로 관측된 빈 공간에 목표 위치와 도착 방향을 찍으면 이동을 시작합니다. 도착하고 완전히 멈춘 뒤에 다음 목표를 주면 되며, 스크립트가 목표를 자동으로 보내지는 않습니다.

`Ctrl+C`를 누르거나 RViz 창을 닫으면 구동을 해제하고 측정 정지를 확인한 뒤 자기가 띄운 프로세스를 정리합니다. 도중에 fault가 나면 자동으로 다시 켜지 않고 원인을 출력한 채 종료합니다. 같은 ROS domain에 이미 시뮬레이션이나 구동 노드가 떠 있으면 중복 실행을 막으므로, 기존 수동 launch는 먼저 닫아 두어야 합니다.

공항 월드에서 돌리려면 `./nav.sh --world airport`를 실행합니다. 기본 ROS domain은 `86`이고 `OMSLAM_NAV_DOMAIN_ID`로 바꿀 수 있습니다. 로그와 시작·종료 기록은 `reports/navigation/<실행시각>/`에 남습니다. 아직 빌드하지 않았다면 아래 빌드 절차를 먼저 진행하면 됩니다.

## 구성과 입력

| 역할 | 구성 |
| --- | --- |
| 전역 계획 | `nav2_smac_planner::SmacPlannerLattice`, diff, 0.05 m, 16 heading, 112 primitives |
| 지역 제어 | `dwb_core::DWBLocalPlanner`, 전진과 제자리 회전, `ObstacleFootprint` 검사 |
| 도착 판정 | `StoppedGoalChecker`, 위치 0.05 m·yaw 0.15 rad 이내와 측정 속도 0.01 m/s·0.02 rad/s 이하 동시 확인 |
| 비용지도 | 전역 static→lidar→camera memory→inflation, 지역 lidar→camera memory→inflation |
| 인지 | depth, CameraInfo, 유효 IMU 자세로 바닥 분리, `odom` 기준 장애물 기억 |
| 명령 체인 | `/cmd_vel/nav`→guard→`/cmd_vel/requested`→smoother→`/cmd_vel/smoothed`→guard→`/cmd_vel`→sim base |
| 위치추정 | EKF의 `odom → base_footprint`, SLAM 또는 AMCL 하나의 `map → odom` |
| 구동 권한 | `/navigation/enable` 서비스, 최신 관측·서버 준비·base 활성화 후 새 목표 필요 |

공통 설정은 `config/nav2_common.yaml`, 시뮬레이션 속도와 시간 값은 `config/sim_profile.yaml`, 인지 설정은 `config/perception.yaml`에 들어 있습니다. launch가 설치된 설정을 합치고 lattice와 BT의 절대 설치 경로를 채워 넣습니다. `nav2_params_file`로 일부 값을 덮어쓸 수는 있지만, 플러그인이나 기하·시간 계약을 깨는 값은 시작 시점에 거부합니다. SLAM과 AMCL 설정은 `backend_params_file`로 따로 넘깁니다.

목표 접근과 방향 정렬의 위치 기준은 `controller_server.goal_checker.xy_goal_tolerance` 한곳에서 관리합니다. launch가 같은 값을 DWB(`FollowPath.xy_goal_tolerance`)로 전달하고, `goal_checker.trans_stopped_velocity`도 함께 넘겨줍니다. 기본 위치 허용 범위는 5 cm입니다. 공식 Jazzy Smac Lattice에는 종단 위치 허용값 파라미터가 없기 때문에 `GridBased.xy_goal_tolerance`는 중첩 표기든 점 표기든 시작할 때 거부합니다. `GridBased.tolerance`(4 cm)는 A* 탐색 fallback 허용값이라 별개입니다.

BT는 로봇이 목표에서 `bt_navigator.goal_reached_tol`(1.5 m)보다 멀리 있을 때 1 Hz로 전역 경로를 다시 짭니다. 1.5 m 안으로 들어오면 `IsPathValid`가 유효한 동안 경로를 새로 짜지 않고 기존 경로를 끝까지 따라갑니다. 목표가 바뀌거나 경로가 막혔을 때만 다시 계획합니다. 이 반경은 재계획 여부만 결정하며 실제 도착 판정은 goal checker 몫입니다. 복구 동작은 정지 후 2초 대기하고 저장된 경로를 지운 뒤 딱 한 번만 새로 계획합니다.

위치 허용 범위 안에 들어갔다고 바로 성공 처리하지 않습니다. DWB가 감속하고 방향을 맞추는 동안에도 매 주기 목표 거리를 다시 잽니다. 방향을 돌리다가 중심이 5 cm 밖으로 밀리면 다시 접근하고, 위치·방향·측정 속도가 한꺼번에 기준 안에 들어와야 완료합니다. 이 기준을 바꾸려면 도착 판정 설정을 고치고 세션을 다시 띄워야 합니다. GUI에서는 목표가 끝나거나 취소·실패하면 진행 경로 표시를 바로 지우고 뒤늦게 온 이전 경로는 무시합니다.

콘솔에서 3 RPM처럼 낮은 속도를 지정하면 DWB가 실제로 움직일 수 있는 이동 후보와 도착 제동 후보를 같이 만들어 평가하고 예측 시간도 늘립니다. 진행 감시기도 승인된 속도로 5초 동안 갈 수 있는 거리와 회전량을 반영하고, 10초 동안 진전이 없으면 실패로 처리합니다.

전역 계획기는 공식 ROS 2 Jazzy 설치본 `nav2_smac_planner`(`1.3.14`, `/opt/ros/jazzy`)를 씁니다. 설치 버전 기준은 [기술 기준](baseline.md#개발-pc-설치-버전)에, 움직임 집합 원본과 해시는 `omslam_navigation/config/lattice/provenance.json`에 적어 두었습니다.

TF 발행 책임은 고정된 launch 구성과 실제 ROS endpoint 목록을 함께 대조해 확인합니다. `/tf`에는 `ekf_filter_node`, `robot_state_publisher`, 그리고 선택한 `slam_toolbox`나 `amcl` 노드가 각각 하나씩 있어야 하고, `/tf_static`은 `robot_state_publisher`만 허용합니다. 한 노드가 broadcaster를 여러 개 여는 경우와 같은 이름의 노드가 중복 실행된 경우를 구분하며, 중복이나 추가 발행 노드가 보이면 구동 준비를 막습니다. Jazzy Python 수신 정보에는 메시지별 sender GID가 없어서 개별 TF edge 발행자 추적으로 표시하지 않고, 대신 edge 수신 공백과 관측 시각 변환, map 보정을 따로 감시합니다.

## 빌드와 실행 환경

다른 workspace overlay가 source되지 않은 깨끗한 셸에서 의존 패키지까지 함께 빌드합니다.

```bash
cd ~/MyRos/Oh-My-SLAM
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 tools/build_navigation.py
```

빌드 스크립트는 소스와 설치 파일의 SHA-256을 남기고 성공한 빌드 정보를 `reports/sim-navigation/latest_build.json`에 저장합니다. 빌드가 끝나면 새 셸을 열어 환경을 잡습니다. 명령을 보내는 다른 터미널에도 같은 domain을 맞춰야 합니다.

```bash
cd ~/MyRos/Oh-My-SLAM
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=86
export GZ_PARTITION=omslam-navigation
ros2 pkg prefix omslam_navigation
ros2 pkg prefix nav2_smac_planner
```

`omslam_navigation`은 현재 워크스페이스의 `install/omslam_navigation`을, `nav2_smac_planner`는 공식 Jazzy 설치본 `/opt/ros/jazzy`를 가리키고 있어야 합니다.

## 온라인 SLAM으로 시작

공항 기본 위치에서 온라인 SLAM 자율주행을 띄웁니다. 같은 domain에 이미 공항이나 로봇 시뮬레이션이 떠 있으면 중복으로 띄우지 말고 기존 실행을 끄거나 `start_sim:=false`를 붙여 재사용합니다.

```bash
ros2 launch omslam_navigation slam_navigation.launch.py
```

작은 검증 코스 `p4_course`로 실행할 때는 아래처럼 옵션을 넘깁니다. 로봇과 센서, EKF, 구동 경로는 공항 월드와 같습니다.

```bash
ros2 launch omslam_navigation slam_navigation.launch.py \
  world:=p4_course spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0 \
  gui:=false pedestrians:=false
```

온라인 모드에서는 현재 지도에서 이미 관측된 자유 공간 안으로 목표를 주어야 합니다. 초기 map 좌표와 Gazebo world 좌표를 헷갈리지 않도록 주의합니다. 공항의 `spawn_x=-108`은 world에서 로봇이 생성되는 좌표이고, SLAM 시작 map 원점과는 다릅니다.

## 저장 지도로 시작

`omslam_mapping`의 `save_map`으로 저장한 시뮬레이션 map YAML과 메타데이터를 씁니다. 지도 이미지와 메타데이터의 해시, frame, backend가 일치해야 하며 `map`에는 절대 경로를 넘깁니다.

```bash
ros2 launch omslam_navigation navigation.launch.py \
  map:=/absolute/path/to/saved_map.yaml \
  initial_pose:=true initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
```

위 초기 좌표는 예시이므로 실제 저장 지도에서 로봇이 있는 위치로 바꿔 넣어야 합니다. `initial_pose:=false`로 띄웠다면 RViz의 **2D Pose Estimate**나 `/initialpose`로 AMCL 초기 위치를 잡은 뒤 준비 상태를 확인합니다. Gazebo 생성 위치를 바꾸려면 `spawn_x/y/yaw`를 따로 지정합니다.

두 navigation launch 안에 SLAM/AMCL 백엔드가 이미 들어 있으므로 별도의 mapping이나 localization launch를 같이 띄우지 않습니다. 백엔드를 바꿀 때는 아래 정지 절차로 기존 navigation launch를 완전히 끄고 관련 publisher가 사라진 뒤 새로 띄웁니다. 시뮬레이터만 계속 살려 두고 싶으면 시뮬레이터를 따로 띄운 상태에서 `start_sim:=false`를 씁니다.

## 활성화·목표·정지

먼저 상태를 확인하고 구동 권한을 엽니다.

```bash
ros2 topic echo /navigation/state std_msgs/msg/String --once \
  --qos-durability transient_local
ros2 service call /navigation/enable std_srvs/srv/SetBool '{data: true}'
```

`READY`는 센서와 TF, 서버가 준비됐다는 뜻이지 아직 구동 권한이 열린 상태는 아닙니다. `enable`을 부르면 이전 목표가 끝나 있고 정상 관측이 1초 동안 유지되며 새 wheel 피드백과 base 활성화까지 확인된 뒤에 성공합니다. 성공하면 상태가 `ARMED_IDLE`로 바뀝니다. 실패하면 응답의 원인과 `missing` 목록부터 해결합니다.

P4용 RViz 화면은 같은 ROS domain의 터미널에서 아래처럼 엽니다.

```bash
rviz2 -d ~/MyRos/Oh-My-SLAM/install/omslam_navigation/share/omslam_navigation/rviz/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

이 RViz 설정에는 Navigation 2 패널과 Nav2 Goal 도구가 들어 있습니다. 쓰지 않는 경로 smoothing 서버를 조회하는 Selector와 docking 패널은 뺐습니다. `velocity_smoother`는 속도 명령을 평활하는 별도 노드입니다.

준비가 끝나면 RViz의 **Nav2 Goal**이나 `NavigateToPose` 액션으로 목표를 보냅니다. 아래는 작은 코스에서 시작점 앞 1 m로 보내는 예시입니다.

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}' \
  --feedback
```

목표는 한 번에 하나씩만 보냅니다. 이전 목표가 끝나고 guard가 `ARMED_IDLE`로 돌아와 차체가 완전히 멈춘 뒤 다음 목표를 보냅니다. 주행 중에 새 목표를 덮어쓰거나 다른 UUID로 동시에 요청하면 차단하고 취소합니다.

멈추고 구동 권한을 풀 때는 아래 서비스를 부릅니다. 호출하자마자 0 속도를 보내고 진행 중 목표를 취소한 뒤 base를 비활성화합니다.

```bash
ros2 service call /navigation/enable std_srvs/srv/SetBool '{data: false}'
```

종료하거나 모드를 바꾸기 전에는 이 서비스가 성공하고 로봇이 실제로 멈췄는지 확인한 뒤 launch를 내립니다. 서비스가 실패하면 fault 상태를 유지하며 정상 종료나 전환으로 보지 않습니다.

## 장애와 복구

| 상태와 원인 | 구동 결과와 복구 방법 |
| --- | --- |
| scan, depth, 인지 health가 0.30 s 이상 오래됨 | 0 속도, 목표 취소, base 비활성화; 데이터가 돌아와도 자동 재출발 없음 |
| EKF odom, wheel, TF, lifecycle 이상 | 동일하게 fault latch; `missing`과 해당 센서·서버 상태 확인 |
| 원본 이동 명령 만료 | 300 ms 지나면 구동 차단, 평활 출력을 반복해도 수명 연장 불가 |
| 명시적 정지나 목표 취소 | 0 속도 우선 처리; 유효한 새 관측과 새 목표로 다시 제어 가능 |
| 제한된 BT 대기 | 정지 → 2초 대기 → 재계획 1회, 계속 막혀 있으면 action 실패 |
| 큰 map 보정 | 연속 표본 사이 0.10 m 또는 0.10 rad 이상 튀면 latch, 같은 세션 재활성화 불가 |
| 시뮬레이션 pause나 시각 역행 | 과거 명령 폐기, 새 관측·활성화·새 목표 필요 |

일반 관측 장애는 원인을 해결하고 이전 목표가 끝난 상태에서 정상 관측이 1초 이상 이어지면 다시 `enable`을 호출한 뒤 **새 목표**를 보냅니다. 큰 map 보정이 일어난 경우에는 정지 상태에서 navigation 모드를 끄고 새 세션에서 위치와 관측을 다시 확인한 뒤 켭니다. 이전 장애물 기억이 자동으로 정확히 옮겨졌다고 가정하지 않습니다.

`queue is full` 같은 경고가 뜨면 큐 크기나 timeout을 무작정 늘리지 말고 발생 노드와 scan timestamp, TF 대기부터 확인합니다. guard는 순서가 살짝 어긋나 도착한 정상 메시지를 작은 대기 버퍼로 맞추되, 원본 관측의 freshness 기준은 늘리지 않습니다.

## 검증과 결과

현재 P4 인수 기준은 **한 번의 실행에서 목표 5곳에 연속으로 정상 도착하는 것**입니다. 최신 빌드와 수정에 직접 걸린 회귀 검사를 먼저 통과한 뒤 실행합니다.

기본 P4 인수 기준(35 RPM, 설치본 `c175e1f`)에서 **목표 5곳 연속 도착과 충돌 0회**(최대 위치 오차 0.1451 m, 최대 방향 오차 0.1414 rad, 정지 유지 확인 0.324초 이내, 장애물 최소 여유 0.1112 m)와 정상 구동 해제·프로세스 종료를 확인했습니다. 이후 추가된 `3–70 RPM` 범위, `3.0 RPM` 최소 구동 하한, `5 cm` 도착 판정은 단위·DWB 프로브 회귀와 헤드리스 Gazebo `floor-arrival` 시험으로 함께 검증합니다.

```bash
/usr/bin/python3 tools/verify_sim_navigation.py \
  --scenario five-goals --output reports/sim-navigation/p4-five-goals
```

같은 코스에서 `(1, 0, 0)` → `(5, 0, π/2)` → `(5, 4, π/2)` → `(0, 4, −π/2)` → `(0, 0, 0)` 순서로 보냅니다. 좌표는 시작 시점 기준의 `(x m, y m, yaw rad)`입니다. 목표마다 action 성공 뒤 측정 정지와 truth 기준 위치 오차 0.15 m 이하, yaw 오차 0.20 rad 이하를 확인합니다. 센서·TF·인지·명령·충돌 관측도 같은 실행에서 계속 지켜보고, 5개를 다 통과하면 구동 권한을 풀고 프로세스를 정리합니다.

전체 기능과 실패 주입까지 확인하는 `integrated` 모드는 같은 빌드의 플러그인·명령 체인·지도 보정 검사부터 먼저 돌립니다. 출력 폴더는 실행마다 새로 지정합니다.

```bash
/usr/bin/python3 tools/verify_nav_contracts.py --integrated --output reports/sim-navigation/p4-contracts
/usr/bin/python3 tools/verify_sim_navigation.py --scenario integrated --output reports/sim-navigation/p4-integrated
```

특정 빌드 기록을 지정하려면 두 검증기의 `--build-manifest`와 주행 검증기의 `--validation-manifest`에 파일 경로를 넘기면 됩니다. 주행 검증기는 시뮬레이터를 띄우기 전에 소스와 설치 파일 지문, 계약 결과를 먼저 확인합니다. 속도 검사는 단계별 선속도와 각속도를 기록하고, 최종 Gazebo 입력은 실행 중인 베이스의 바퀴 반경·윤거·RPM 한도로 판정합니다.

각 출력 디렉터리의 `manifest.json`, `results.json`, `metrics.json`, `events.jsonl`, 로그, 경로 그림이 판정 근거가 됩니다. 이전 완료 결과를 덮어쓰지 않고 새 경로를 쓰며, 실패 기록도 남겨 둡니다. 이전 실행의 성공을 합쳐서 연속 5회 성공으로 세지 않습니다.

## 장애 이후 명시적 복구

`nav.sh`는 주행 중 일반 `FAULT_LATCHED`가 걸려도 Gazebo와 Nav2, RViz를 바로 닫지 않고 살려 둡니다. 원인을 해결하고 최신 정지·관측과 이전 목표 종료가 확인되면 터미널에 안내된 `/navigation/enable` 명령으로 다시 열고 새 목표를 보내면 됩니다. 자동으로 다시 출발하거나 이전 목표를 이어서 달리지 않습니다. 큰 `map → odom` 보정처럼 `restart_required=true`가 뜬 경우에는 종료 후 세션을 새로 띄워야 합니다. 준비 시간 초과나 실제 자식 프로세스 비정상 종료는 그대로 오류로 처리합니다.

전역과 지역 inflation 반경은 `0.55 m`로 잡았습니다. padding을 포함한 차체 외접 반경 약 `0.4965 m`에 한 셀(`0.05 m`) 여유를 더한 값입니다. GUI에서 전체 `costmap_raw`를 보여 주기 위해 `always_send_full_costmap=true`를 유지합니다. 계산 bounds를 줄이는 것과 전체 costmap을 발행하는 것은 별개의 설정입니다.
