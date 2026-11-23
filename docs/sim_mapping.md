# 시뮬레이션 지도 작성과 저장 지도 위치추정

[README](../README.md) · [로봇·센서](sim_robot.md) · [단계별 계획](roadmap.md) · [검증 기준](acceptance.md)

`omslam_mapping`은 로봇의 라이다와 EKF 오도메트리로 지도를 작성하고, 저장한 지도에서 AMCL로 위치를 추정합니다. 시뮬레이터와 로봇·센서는 `omslam_sim`이 실행하고, 이 패키지는 그 위에 지도 작성 또는 저장 지도 위치추정 중 한 모드를 연결합니다.

## 입력과 TF 소유권

| 항목 | 지도 작성 | 저장 지도 위치추정 |
| --- | --- | --- |
| 라이다 | `/scan`, `laser_frame` | 동일 |
| 로컬 위치 | EKF의 `odom → base_footprint` | 동일 |
| 지도 발행 | `slam_toolbox`의 `/map` | `map_server`의 `/map` |
| 전역 위치 보정 | `slam_toolbox`의 `map → odom` | AMCL의 `map → odom` |
| 시각 | `/clock`, `use_sim_time=true` | 동일 |

지도 모드는 로봇·EKF를 다시 생성하지 않습니다. 모드를 전환할 때 기존 지도 모드를 종료한 후 새 모드를 실행합니다. 검증용 `/sim/ground_truth`는 지도 작성·위치추정 입력으로 사용하지 않습니다.

## 실행

빌드는 시뮬레이터와 지도 패키지를 함께 대상으로 합니다.

```bash
cd ~/MyRos/Oh-My-SLAM
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to omslam_sim omslam_mapping \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
./sim.sh pedestrians:=false
```

지도 명령을 실행할 별도 터미널에는 같은 workspace와 ROS domain을 설정합니다. 아래 환경 설정은 각 새 터미널에 적용합니다.

```bash
cd ~/MyRos/Oh-My-SLAM
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=86
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

지도 작성 모드를 실행합니다.

```bash
ros2 launch omslam_mapping mapping.launch.py use_sim_time:=true
```

로봇은 기본 미구성·정지 상태입니다. 주행을 시작하려면 `/sim_base`를 configure한 뒤 최신 wheel feedback이 들어오는 상태에서 activate합니다. 명령은 [기존 stamped 입력 계약](sim_robot.md)에 맞춰 제공합니다. 지도 launch 자체가 로봇을 움직이지는 않습니다.

```bash
ros2 lifecycle set /sim_base configure
ros2 lifecycle set /sim_base activate
```

정지한 상태에서 다른 터미널로 지도와 그래프를 저장합니다. 출력 prefix는 존재하지 않는 새 경로를 사용합니다.

```bash
ros2 run omslam_mapping save_map \
  --output "$PWD/maps/airport_p3/map" --backend simulation --workspace "$PWD"
```

지도 작성 launch를 Ctrl+C로 종료하고 저장 지도 위치추정으로 전환합니다. 로봇·센서·EKF는 계속 실행합니다.

```bash
ros2 launch omslam_mapping localization.launch.py \
  map:="$PWD/maps/airport_p3/map.yaml" use_sim_time:=true
```

AMCL의 초기 자세는 `/initialpose`로 지정합니다. 초기 위치를 알고 있다면 `initial_pose:=true x:=... y:=... yaw:=...` 인수로 지정할 수 있습니다. 좌표는 Gazebo world 좌표가 아닌 저장된 `map` 프레임이며 yaw 단위는 rad입니다.

저장 그래프로 지도 작성을 이어갈 때는 AMCL 모드를 종료하고 mapping 모드를 다시 실행한 후 그래프를 불러옵니다.

```bash
ros2 run omslam_mapping load_graph \
  --input "$PWD/maps/airport_p3/map" --match pose --x 0 --y 0 --yaw 0
```

위 `0, 0, 0`은 저장 지도에서 로봇의 현재 자세로 바꿉니다. 첫 저장 노드 위치에서 시작할 때는 `--match first`를 사용합니다. Occupancy YAML을 pose graph 입력으로 전달하지 않습니다.

## 저장 파일

Occupancy 지도는 지도 이미지와 YAML로 저장하며 `map_server`·AMCL에서 사용합니다. SLAM pose graph는 스캔과 그래프 상태를 저장하며 `slam_toolbox`에서 지도 작성을 이어갈 때 사용합니다. 두 파일 형식은 용도가 다릅니다.

저장 묶음에는 backend, frame 이름, 소스·의존성 버전, 설정·기하 식별 정보를 포함합니다. 시뮬레이션에서 생성한 지도와 실내 실측 지도는 구분해서 보관합니다. 저장 파일은 Git에서 제외된 `maps/` 또는 실행 결과 폴더에 둡니다.

## 검증 범위

**P3 구현과 시뮬레이션 검증을 완료했습니다.** 최신 6개 패키지 빌드, 모드 잠금 단위 검사 1개와 핵심 통합 검사 11개가 통과했습니다. 0.05 m 해상도의 실제 SLAM 지도·그래프 저장과 새 프로세스의 그래프 복원을 확인했습니다. 약 10 m 폐루프의 초기·후기 스캔 연결은 저장 그래프의 실제 constraint로 확인했고, AMCL은 0.430 m·0.25 rad 초기 오차에서 지도 안의 고정 3 m 관측 주행 후 위치 0.0359 m·yaw 0.00133 rad 오차로 수렴했습니다. 최종 스캔과 지도 장애물의 거리 p95는 0.05 m였습니다.

초기 자세를 지정해도 즉시 충분히 수렴한다고 가정하지 않습니다. 짧은 0.6 m 관측에서는 위치 오차 0.222 m가 남았으므로, 유효한 라이다 관측과 이동 후 위치추정 결과를 확인합니다. 이 결과는 공항 일부 구간의 정적 시뮬레이션에 대한 것이며 전체 공항 지도의 완성을 뜻하지 않습니다.

공항 일부 구간의 짧은 폐루프 주행에서 실제 라이다로 지도와 pose graph를 생성합니다. 저장 파일의 재로딩, SLAM 종료 후 AMCL 활성화, 초기 자세 이후 위치 수렴, `map → odom`의 단일 발행자와 시각을 확인합니다. AMCL 결과는 같은 기록의 시뮬레이션 truth와 비교하며 위치 오차 0.15 m 이하, yaw 오차 0.20 rad 이하를 기준으로 합니다.

정적 공항으로 지도 흐름을 검증합니다. RGB-D 낮은 장애물 기억·자율 목표 주행은 P4, 실물 지도 작성은 해당 실물 단계에서 다룹니다.
