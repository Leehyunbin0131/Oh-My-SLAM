# 기술 기준과 호환성

[README](../README.md) · [통합 설계](architecture.md) · [구현 계획](roadmap.md) · [검증 기준](acceptance.md)

기준 환경인 **Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic**의 패키지 버전과 재사용하는 하위 제어·로봇 모델 커밋, 차체 기하 초기값, 기존 자산 이관 범위를 정리한 문서입니다.

## 시스템 기준

| 구성 요소 | 기준 | 적용 조건 |
| --- | --- | --- |
| 메가/Orin 공통 제어 | `Oh-My-Control@5d5d65b6ba208383d3dede73ffb8a108cf786a76` | `c703f13`에 원본 시각 API를 추가한 로컬 커밋. vendor에 동일 모듈 포함, 실물 연결은 별도 검증 |
| 실물 컴퓨터 | Jetson Orin, aarch64, Ubuntu 24.04.5 LTS, ROS 2 Jazzy | ARM64에서 별도 빌드 |
| 기존 실물 구성 | `robot_mapping_ws`, 수동 mapping과 저장 지도 navigation | 새 bridge와 odom/TF 계약으로 이관 |
| 기존 시뮬레이션 | Jazzy + Harmonic, NavFn + MPPI, 보로노이 선호 cost와 깊이 장애물 기억 | 새 공통 기준은 State Lattice + DWB와 표준 비용지도 |
| 모델 저장소 | `Oh-My-MobileRobot@f85154b53ea57b80e8a8ff6e4d6d8d38a8a0c35b` | description과 CAD/mesh 출처·라이선스 유지 |
| 소스 구성 | 기존 sim은 독립 Git 이력 없음 | 이관 파일의 SHA256과 NOTICE로 출처 고정 |

저수준 모터 제어를 실물에서 확인했다고 해서 ROS bridge나 오도메트리, SLAM, Nav2 검증이 끝난 것은 아닙니다. 실물 센서 드라이버의 정확한 설치 버전과 토픽, frame, 시각·QoS 설정은 P5 배치 단계에서 확정합니다.

## 개발 PC 설치 버전

개발 PC에 설치된 기준 버전은 아래와 같습니다. Orin 쪽 버전은 장비별 배치 manifest에 따로 고정합니다.

| 패키지 | 기준 버전 |
| --- | --- |
| `ros_gz_bridge` | 1.0.25 |
| `nav2_bringup`, `nav2_smac_planner`, `nav2_controller`, `dwb_core`, `dwb_plugins`, `dwb_critics`, `nav2_velocity_smoother` (Jazzy 설치본) | 1.3.14 |
| `slam_toolbox` | 2.8.5 |
| `robot_localization` | 3.8.4 |
| `diff_drive_controller` | 4.42.1 |

전역 계획기 `nav2_smac_planner`와 속도 평활기 `nav2_velocity_smoother`는 모두 공식 ROS 2 Jazzy 설치본(`/opt/ros/jazzy`, 1.3.14)을 그대로 씁니다.

- 전역 계획은 `nav2_smac_planner::SmacPlannerLattice`에 `omslam_navigation`에 넣어 둔 `diff` 움직임 집합(`5 cm`, 16방향, 회전 반경 `0.5 m`)을 연결해 씁니다.
- 지역 제어는 `dwb_core::DWBLocalPlanner`에 `omslam_navigation`의 커스텀 trajectory generator, footprint·turn·oscillation critic, `SpeedAwareProgressChecker`를 붙여 씁니다.
- 도착 판정은 `StoppedGoalChecker`(`stateful=false`, XY `0.05 m`, yaw `0.15 rad`, 정지 속도 `0.01 m/s`와 `0.02 rad/s`)가 매 주기 위치, 방향, 측정 정지를 같이 확인합니다.
- 속도 명령은 `/cmd_vel/nav` → `navigation_guard` → `/cmd_vel/requested` → `velocity_smoother` → `/cmd_vel/smoothed` → `navigation_guard` → `/cmd_vel` → base 순서로 흐르며, 원본 timestamp 유지와 300 ms 명령 수명 검사를 끝까지 지킵니다.

공식 문서의 최신 Jazzy 브랜치와 내 PC에 실제로 깔린 바이너리가 100% 같다고 가정하지 않습니다. 특히 stamped velocity 옵션이나 smoother timestamp 보존 옵션은 설치된 바이너리의 선언 파라미터와 실제 메시지로 확인합니다.

## 제어 계층 규격

| 항목 | 값과 의미 |
| --- | --- |
| 프로토콜 | Mega v8, `signed_pwm_v1`, OMR1 CRC/sequence 프레임 |
| USB 시리얼 | 115200 bps |
| MCU 속도 제어 | 1 kHz |
| 호스트 송신 / 상태 수신 | 최대 50 Hz / 약 20 Hz |
| command lease | 100–300 ms, 원본 생성 시각 기준 유효기간 유지 |
| 피드백 중단 | 호스트 400 ms 기준 |
| 바퀴 상한 | 실물 운용 3–95 RPM / MCU 보호 100 RPM / 시뮬레이션 3–70 RPM |
| 정상 0 속도 | 제동과 HOLD |
| `STOP` | 제어와 출력 해제 |

참고 소스: [제어 소스 고정 커밋](https://github.com/Leehyunbin0131/Oh-My-Control/tree/c703f130c14388b4b7845a25b900d726f8c9a71d) (`tools/wheel_drive.py`, `motor_link.py`, `motor_pid.py`, `drive_calibration.py`, `docs/control_rebuild.md`).

## 모델 값과 실측값의 구분

| 항목 | 현재 모델 값 | 적용 방식 |
| --- | --- | --- |
| 바퀴 반경 | 0.068 m | 시뮬레이션 초기값; 실물 유효값은 바닥 주행으로 보정 |
| 좌우 구동축 간격 | 0.480 m | 실물 유효 윤거와 구분 |
| `base_footprint → base_link` | `(0, 0, 0.101)` m | 원본 모델 구조 유지 |
| wheel joints | `left_wheel_joint`, `right_wheel_joint` | 위치 `(0, ±0.24, −0.033)` m, 축 +Y |
| 기존 sim footprint | 앞 +0.09 / 뒤 −0.39 / 좌우 ±0.26 m | 원점은 구동축 기준; 실물 외곽 확인 후 갱신 |
| 기존 padding | 0.02 m | 초기값; 실제 외곽과 제동·관측 오차를 보고 조정 |
| lidar | `base_link → lidar_link=(-0.14,0,0.143)` m, 이후 `laser_frame` z=0.0303 m | CAD 기준 위치이며 실물 장착 TF 확인 필요 |
| camera | `base_link → camera_link=(0.06505,0,0.1035)` m | optical transform 포함; 실물 높이와 각도 대조 |

기준 xacro 파일의 SHA256은 `5290010ef531becc2b4c5e3db8f64dff8692b4000129714ee13cbbf37a389009`입니다. 원본 저장소와 기존 시뮬레이션의 모델 내용은 같고, 버전 표기에서는 가져온 시점의 commit과 현재 원본 저장소 HEAD를 구분해 적습니다.

## 기존 자산의 재사용과 변경

| 기존 자산 | 새 저장소에서의 처리 |
| --- | --- |
| Mega 펌웨어와 공통 통신·보정 | 버전을 고정해 재사용. 원본 시각을 넘기는 호환 API 확장을 vendor에 반영 |
| 원본 description과 mesh | 출처와 라이선스 유지, 필요한 경로만 설치 |
| Gazebo 모델, 센서, 월드 | 공통 ROS 계약과 시뮬레이션 3–70 RPM 범위 적용 |
| `omm_perception`, `omm_perception_msgs` | 바닥 분리와 낮은 장애물 기억 기능 이관 |
| `omm_costmap_layers` | camera memory 레이어 이관. 보로노이 전용 레이어는 초기 구성에서 제외 |
| State Lattice | 새 기본 전역 계획기. 차동구동 움직임 집합과 footprint, costmap 해상도를 공통 계약에 맞춰 고정 |
| 기존 MPPI critic | 초기 DWB 구성 이관 대상에서 제외 |
| 기존 실물 bridge | 통신·정지·원본 수명 계약을 새 base에 맞춰 구현. 확인 없이 그대로 복사하지 않음 |
| ICP odometry | 새 기본 구성은 실측 wheel odom + IMU. 기존 ICP TF와 동시에 발행하지 않음 |
| 기존 실물 지도와 보정 | 원본 보존, 장비별 로컬 설정으로 관리. 새 구성 통과 근거로 자동 승계하지 않음 |

## 레거시 구성과의 차이

| 구분 | 기존 구성의 특성 | 새 구성에서 바꾼 점 |
| --- | --- | --- |
| 실물 controller | 운용 문서는 NavFn + RPP, 보관 검증 일부는 NavFn + MPPI | State Lattice + DWB로 통일. controller 설정을 고정하고 다른 구성 결과를 섞지 않음 |
| 실물 odom | 보관 구성은 `icp_odometry`, 약 12 Hz | 실측 wheel odom + EKF로 전환, 기존 TF 발행 비활성화 |
| sim odom | Gazebo가 odom과 odom TF를 함께 발행 | EKF를 켤 때 Gazebo TF 제거 |
| sim 명령 | `Twist`를 guard에서 주기적으로 재발행 | stamped 원본 수명과 공통 명령 계약으로 변경 |
| 실물 입력 timeout | 기존 bridge는 계속 0 RPM/HOLD 유지 | 정상 0 명령과 입력 단절을 구분하고, 단절 시 `STOP` |
| 기존 인수 근거 | 일부 실행은 `motor:=false`이거나 경로 생성 직후 취소 | 새 실물 목표 도착 검증 근거로 쓰지 않음 |

자세한 명령과 TF 계약은 [통합 설계](architecture.md), 단계별 판정 기준은 [검증 기준](acceptance.md)에 정리되어 있습니다.

## 참고한 공식 자료

설계 판단은 이 로봇의 하드웨어 제약을 바탕으로 내렸고, ROS 인터페이스와 동작 규칙은 아래 공식 자료를 참고했습니다.

1. [Nav2 Jazzy: Gazebo 구성 순서](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/gazebo/) — TF, 모델, odom, 센서, mapping, navigation 연결 순서
2. [Nav2 Jazzy: robot_localization](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization/) — wheel과 IMU 결합, odom TF 단독 발행
3. [Nav2 Jazzy smoother 소스](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_velocity_smoother/src/velocity_smoother.cpp) — stamp 유지 방식과 timeout 뒤 메시지의 동작 확인
4. [SLAM Toolbox Jazzy README](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/README.md) — scan/odom 입력, map 저장과 pose graph 직렬화의 차이
5. [REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst), [REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst) — 좌표축과 SI 단위, map/odom/base 프레임 책임
6. [ros2_control Jazzy diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) — 대안 검토 참고(현재 기준 경로에서는 미사용)
