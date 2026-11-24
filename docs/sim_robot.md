# 공항 로봇과 센서 시뮬레이션

[README](../README.md) · [공항 환경](airport_world.md) · [구동·EKF](base_ekf.md)

`omslam_sim`은 공항 월드에 CAD 기반 차동구동 로봇을 올리고, 가상 라이다·IMU·RGB-D 센서와 바퀴 오도메트리를 ROS 2에 연결합니다. 속도 명령은 원본 시각을 검사한 뒤 Gazebo로 넘기고, EKF가 바퀴 측정값과 IMU 각속도를 합쳐 로컬 오도메트리를 만듭니다.

## 모델과 시뮬레이션 범위

`omslam_description`은 `dependencies.repos`에 고정해 둔 `Oh-My-MobileRobot`의 원본 base URDF와 메시를 그대로 유지하면서 시뮬레이션에 필요한 관성, 마찰, 구동 플러그인, 가상 센서를 덧붙였습니다. 원본 모델의 CERN-OHL-P-2.0 라이선스와 파일별 출처·해시도 함께 들어 있습니다.

바퀴 반경 0.068 m, 윤거 0.480 m, 센서 장착 위치는 모두 모델 초기값입니다. 질량, 관성, 마찰, 센서 잡음도 시뮬레이션용 설정일 뿐 실물 보정 결과가 아닙니다. Gazebo DiffDrive는 차동구동 움직임을 흉내 내는 역할이며, Arduino Mega의 1 kHz PID나 USB 통신 지연, 실제 바닥 제동 특성까지 재현한 것은 아닙니다.

## 데이터와 좌표계

- 전진이 차체 +X, 좌회전이 +yaw입니다. wheel joint 회전 부호와 측정 odom이 실제 Gazebo 이동 방향과 맞는지 확인합니다.
- `/wheel/odometry`는 Gazebo가 측정한 바퀴 이동량이며 EKF 입력으로 들어갑니다. 입력받은 속도 명령을 측정값인 것처럼 되돌려 보내지 않습니다.
- `/imu/raw`는 `imu_link` 기준 가상 센서 출력이며, 실물과 같은 IMU 처리·EKF 경로를 탑니다.
- `/scan`은 `laser_frame`, RGB-D 영상은 카메라 optical frame 기준입니다. Optical frame은 Z 전방, X 우측, Y 하방입니다.
- `/joint_states`는 `robot_state_publisher`가 바퀴 관절 TF를 만드는 데 쓰고, 고정 센서 TF도 로봇 모델에서 같이 나옵니다.
- `odom → base_footprint` TF는 EKF만 발행합니다. Gazebo 자체의 odom TF는 ROS `/tf`로 넘기지 않습니다.
- `/sim/ground_truth`는 검증 도구에서만 씁니다. EKF, SLAM, 주행 노드는 이 값을 위치 입력으로 쓰지 않습니다.

관련된 ROS 노드는 전부 `use_sim_time=true`로 맞추고, 센서 stamp와 TF는 `/clock` 기준을 따릅니다. ROS 시각과는 별도로 monotonic 워치독이 명령 입력이 끊기는지 감시합니다. 시뮬레이션을 일시정지하거나 시각을 초기화하면 이전 이동 목표를 버리고 새 명령을 기다립니다.

## 검증 내용

로봇·센서 정의와 명령 가드 단위 검사 6개, 최신 설치본 통합 검사 25개가 통과했습니다. 0.16 m/s 직진 명령 구간에서 Gazebo 모델 0.294 m, wheel odometry 0.294 m, EKF 0.287 m 전진이 관측됐고, 좌회전할 때 모델·바퀴·IMU·EKF의 yaw 부호가 모두 양수로 일치했습니다. 명시적 정지, 명령 만료, pause/resume 뒤 정지 유지와 새 명령으로 재출발하는 동작도 확인했습니다.

짧은 주행 한 번으로 전진·좌회전·정지, 원본 명령 만료와 pause/resume, 바퀴와 EKF 이동량, IMU 부호, TF 단독 발행자, 센서 frame·시각·QoS, RGB-D 데이터를 함께 확인합니다. 이미 통과한 공항 가구·보행자 밀도 검사는 다시 돌리지 않습니다.

이 문서의 검증 범위는 시뮬레이션 하단 구동과 센서 연결까지입니다. 실물 구동과 장착 보정, SLAM 지도 품질, 자율 목표 도착은 각 단계에서 따로 검증합니다.

## 실행

```bash
cd ~/MyRos/Oh-My-SLAM
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to omslam_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
./sim.sh
```

`sim.sh`는 ROS domain 86과 전용 Gazebo partition에서 설치된 `sim.launch.py`를 띄웁니다. 기본 생성 위치는 T1의 `(-108, 0, 0.015)`이고 GUI와 보행자 32명이 함께 켜집니다. `gui:=false`로 화면 없이 서버만 띄우거나 `pedestrians:=false`로 보행자를 끌 수 있습니다. 종료할 때는 `Ctrl+C`를 누르면 됩니다.

현재 설치된 Gazebo의 `libgz-sim8` 내부 스레드에서는 종료할 때 segmentation fault가 간혹 발생합니다. 주행·센서 검사를 통과한 뒤나 기본 정지 상태에서 종료할 때도 재현됐지만, 실행한 자식 프로세스는 전부 정상 정리됩니다. 주행 중 센서·명령 검증 결과와는 별개인 알려진 종료 문제입니다.

센서 토픽은 바로 나오지만 `/sim_base`는 처음에 미구성 상태로 대기합니다. 같은 domain에서 `configure`를 호출하고 wheel feedback이 들어오면 `activate`로 엽니다. 명령은 `/cmd_vel/requested`와 `/cmd_vel`에 같은 원본 stamp와 `base_footprint` frame을 넣은 `TwistStamped`로 보냅니다. 활성화 전이나 pause 중에 들어온 명령은 재개 뒤 다시 쓰지 않습니다. 기본 실행에는 자율주행 노드나 실제 하드웨어 접근이 포함되지 않습니다.

| 토픽 | 센서 데이터 |
| --- | --- |
| `/scan` | 가상 라이다 LaserScan |
| `/imu/raw`, `/imu/data` | 가상 IMU 원본과 보정 출력 |
| `/camera/color/image_raw` | RGB 영상 |
| `/camera/depth/image_rect_raw` | 깊이 영상 |
| `/camera/depth/camera_info`, `/camera/color/camera_info` | 광학 모델과 내부 파라미터 |
| `/wheel/odometry`, `/odom` | 바퀴 측정과 EKF 결과 |
| `/joint_states` | 시뮬레이션 바퀴 관절 측정 |

`tools/verify_sim_robot.py`를 쓰면 격리된 환경에서 위 통합 검증을 한 번에 돌릴 수 있습니다. 최신 설치본을 source한 셸에서 `--help`로 옵션을 확인하면 됩니다. 로봇 없이 공항 환경만 띄우고 싶을 때는 `ros2 launch omslam_sim airport.launch.py`를 실행합니다.
