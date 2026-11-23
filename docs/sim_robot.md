# 공항 로봇·센서 시뮬레이션

[README](../README.md) · [공항 환경](airport_world.md) · [구동·EKF](base_ekf.md)

`omslam_sim`은 공항 월드에 CAD 기반 차동구동 로봇을 생성하고 가상 라이다·IMU·RGB-D와 wheel odometry를 ROS 2로 연결합니다. 목표 속도 명령은 원본 시각을 검사한 뒤 Gazebo로 전달하고, EKF는 바퀴 측정과 IMU로 로컬 오도메트리를 생성합니다.

## 모델과 물리 범위

`omslam_description`은 `dependencies.repos`에 고정된 `Oh-My-MobileRobot`의 원본 base URDF·참조 mesh를 보존하고 시뮬레이션용 관성·마찰·구동·센서를 추가합니다. 원본 모델의 CERN-OHL-P-2.0 라이선스와 파일별 출처·해시를 함께 제공합니다.

바퀴 반경 0.068 m, 윤거 0.480 m, 센서 위치는 모델 초기값입니다. 질량·관성·마찰과 센서 잡음은 시뮬레이션 설정이며 실물 보정 결과가 아닙니다. Gazebo DiffDrive는 차동구동을 모사하며 Mega의 1 kHz PID·USB 전송·실제 제동 특성을 재현했다고 판정하지 않습니다.

## 데이터와 좌표계

- 전진은 차체 +X, 좌회전은 +yaw입니다. wheel joint의 회전 부호와 측정 odom을 실제 Gazebo 이동과 비교합니다.
- `/wheel/odometry`는 Gazebo wheel 측정이며 EKF 입력으로 사용합니다. 원본 속도 명령을 측정값으로 바꿔 발행하지 않습니다.
- `/imu/raw`는 `imu_link`의 가상 센서 출력입니다. 기존 IMU 처리·EKF 경로를 사용합니다.
- `/scan`은 `laser_frame`, RGB-D 데이터는 카메라 optical frame 기준입니다. Optical frame은 Z 전방·X 우측·Y 하방입니다.
- `/joint_states`는 robot_state_publisher가 관절 TF를 만드는 입력입니다. 고정 센서 TF도 로봇 모델에서 제공합니다.
- EKF만 `odom → base_footprint`를 발행합니다. Gazebo의 odom TF는 ROS `/tf`에 연결하지 않습니다.
- `/sim/ground_truth`는 검증 전용입니다. EKF·SLAM·주행 노드는 이 값을 위치 입력으로 사용하지 않습니다.

모든 관련 ROS 노드는 `use_sim_time=true`이며, 센서 stamp와 TF는 `/clock` 기준입니다. ROS 시각과 별도로 monotonic watchdog이 명령 입력 중단을 감시합니다. pause 또는 시각 초기화 이후에는 이전 이동 목표를 재생하지 않고 새로운 명령을 요구합니다.

## 검증 범위

로봇·센서 정의와 명령 가드 단위 검사 6개, 최신 설치본의 통합 검사 25개가 통과했습니다. 0.16 m/s 명령 구간에서 Gazebo 모델 0.294 m, wheel odometry 0.294 m, EKF 0.287 m 전진을 관측했고, 좌회전 시 모델·바퀴·IMU·EKF의 양의 yaw 부호가 일치했습니다. 명시적 정지·명령 만료와 pause/resume 이후 정지 유지, 새 명령에 의한 재출발도 확인했습니다.

최신 설치본의 짧은 주행에서 전진·좌회전·정지, 원본 명령 만료와 pause/resume, 바퀴·EKF 이동량, IMU 부호, TF 발행자, 센서 frame·시각·QoS 및 RGB-D 데이터를 확인합니다. 통과한 공항 가구·보행자 밀도 검사를 반복하지 않습니다.

이 단계는 시뮬레이션 하단과 센서 연결 범위입니다. 실물 구동·장착 보정, SLAM 지도 품질, 자율 목표 도착은 각 해당 단계에서 검증합니다.

## 실행

```bash
cd ~/MyRos/Oh-My-SLAM
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to omslam_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
./sim.sh
```

`sim.sh`는 domain 86과 전용 Gazebo partition에서 설치된 `sim.launch.py`를 실행합니다. 기본 스폰은 T1의 `(-108, 0, 0.015)`이며 GUI·32명 보행자가 켜집니다. `gui:=false`로 서버만 실행하거나 `pedestrians:=false`로 보행자를 끌 수 있습니다. 실행을 종료할 때는 Ctrl+C를 사용합니다.

현재 설치된 Gazebo의 `libgz-sim8` 내부 스레드에서 종료 시 segmentation fault가 관측됩니다. 주행·센서 검사 통과 후와 기본 정지 상태 종료에서 재현되었으며, 소유 프로세스는 모두 정리됐습니다. 운전 중 센서·명령 검증 결과와 구분하는 알려진 종료 제한입니다.

센서는 실행되지만 `/sim_base`는 기본 미구성 상태입니다. 같은 domain에서 configure 후 wheel feedback이 준비되면 activate합니다. 명령 입력은 `/cmd_vel/requested`와 `/cmd_vel`에 동일한 원본 stamp·`base_footprint` frame의 `TwistStamped`를 제공합니다. 활성화 전·pause 중의 명령을 재개 후 재사용하지 않습니다. 기본 실행에는 자율주행과 실제 장치 접근이 없습니다.

| 토픽 | 센서 데이터 |
| --- | --- |
| `/scan` | 가상 라이다 LaserScan |
| `/imu/raw`, `/imu/data` | 가상 IMU 원본과 검증·보정 출력 |
| `/camera/color/image_raw` | RGB 영상 |
| `/camera/depth/image_rect_raw` | 깊이 영상 |
| `/camera/depth/camera_info`, `/camera/color/camera_info` | 광학 모델·내부 파라미터 |
| `/wheel/odometry`, `/odom` | 바퀴 측정과 EKF 결과 |
| `/joint_states` | 실제 시뮬레이션 관절 측정 |

`tools/verify_sim_robot.py`는 격리 실행에서 요청 범위의 통합 검증을 수행합니다. 최신 설치본을 source한 셸에서 `--help`로 실행 인수를 확인합니다. 로봇을 제외한 공항 환경은 `ros2 launch omslam_sim airport.launch.py`로 실행할 수 있습니다.
