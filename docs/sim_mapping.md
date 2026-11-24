# 시뮬레이션 지도 작성과 저장 지도 위치추정

[README](../README.md) · [로봇·센서](sim_robot.md) · [단계별 계획](roadmap.md) · [검증 기준](acceptance.md)

`omslam_mapping`은 라이다 스캔과 EKF 오도메트리로 지도를 만들고, 저장해 둔 지도 위에서 AMCL로 로봇 위치를 찾는 패키지입니다. 시뮬레이터와 로봇·센서는 `omslam_sim`이 띄우고, `omslam_mapping`은 그 위에 지도 작성과 저장 지도 위치추정 중 한 모드를 올려 연결합니다.

## 입력과 TF 소유권

| 항목 | 지도 작성 | 저장 지도 위치추정 |
| --- | --- | --- |
| 라이다 | `/scan`, `laser_frame` | 동일 |
| 로컬 위치 | EKF의 `odom → base_footprint` | 동일 |
| 지도 발행 | `slam_toolbox`의 `/map` | `map_server`의 `/map` |
| 전역 위치 보정 | `slam_toolbox`의 `map → odom` | AMCL의 `map → odom` |
| 시각 | `/clock`, `use_sim_time=true` | 동일 |

지도 모드를 띄운다고 로봇이나 EKF를 새로 만들지 않습니다. 모드를 바꿀 때는 켜져 있던 지도 모드를 먼저 끄고 새 모드를 올립니다. 검증용 `/sim/ground_truth`는 지도 작성이나 위치추정 입력으로 쓰지 않습니다.

## 실행 방법

시뮬레이터와 지도 패키지를 함께 빌드하고 공항 시뮬레이션을 띄웁니다.

```bash
cd ~/MyRos/Oh-My-SLAM
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to omslam_sim omslam_mapping \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
./sim.sh pedestrians:=false
```

지도 명령을 칠 새 터미널에는 같은 workspace와 ROS domain을 맞춰 두어야 합니다.

```bash
cd ~/MyRos/Oh-My-SLAM
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=86
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

먼저 지도 작성 모드를 켭니다.

```bash
ros2 launch omslam_mapping mapping.launch.py use_sim_time:=true
```

처음에는 로봇이 미구성·정지 상태에 있습니다. 움직이려면 `/sim_base`를 `configure`한 뒤 최신 wheel feedback이 들어오는 상태에서 `activate`합니다. 속도 명령은 [기존 stamped 입력 계약](sim_robot.md)에 맞춰 보내면 되고, 지도 launch 자체는 로봇을 움직이지 않습니다.

```bash
ros2 lifecycle set /sim_base configure
ros2 lifecycle set /sim_base activate
```

지도를 다 그렸으면 로봇을 세워 두고 다른 터미널에서 지도와 그래프를 저장합니다. 출력 prefix는 기존 파일과 겹치지 않는 새 경로로 지정합니다.

```bash
ros2 run omslam_mapping save_map \
  --output "$PWD/maps/airport_p3/map" --backend simulation --workspace "$PWD"
```

지도 작성 launch를 `Ctrl+C`로 끄고 저장 지도 위치추정으로 넘어갑니다. 로봇과 센서, EKF는 그대로 켜 두면 됩니다.

```bash
ros2 launch omslam_mapping localization.launch.py \
  map:="$PWD/maps/airport_p3/map.yaml" use_sim_time:=true
```

AMCL 초기 자세는 `/initialpose`로 보내거나, 처음 위치를 알고 있으면 `initial_pose:=true x:=... y:=... yaw:=...` 인수로 바로 넘길 수 있습니다. 이때 좌표는 Gazebo world 기준이 아니라 저장된 `map` 프레임 기준이고 yaw 단위는 rad입니다.

저장해 둔 그래프에서 지도 작성을 이어가고 싶을 때는 AMCL을 끄고 mapping 모드를 다시 켠 뒤 그래프를 불러옵니다.

```bash
ros2 run omslam_mapping load_graph \
  --input "$PWD/maps/airport_p3/map" --match pose --x 0 --y 0 --yaw 0
```

위 `0, 0, 0`은 저장 지도 안에서 로봇이 현재 놓여 있는 자세로 바꿔 넣습니다. 처음 저장된 노드 위치에서 시작할 때는 `--match first` 옵션을 씁니다. Occupancy YAML 파일을 pose graph 입력으로 넘기지 않도록 주의합니다.

## 저장 파일 구성

Occupancy 지도는 이미지 파일과 YAML로 저장되며 `map_server`와 AMCL에서 읽습니다. SLAM pose graph는 스캔과 그래프 상태를 담고 있어 `slam_toolbox`에서 지도를 이어 그릴 때 씁니다. 두 파일은 역할이 다릅니다.

저장 묶음에는 backend, frame 이름, 소스·의존성 버전, 설정과 기하 식별값을 함께 남깁니다. 시뮬레이션 지도와 실물 실내 지도는 섞이지 않게 나눠 보관하고, 저장 파일은 Git에서 제외된 `maps/`나 실행 결과 폴더에 둡니다.

## 검증 결과

**P3 구현과 시뮬레이션 검증을 마쳤습니다.** 최신 6개 패키지 빌드와 모드 잠금 단위 검사 1개, 핵심 통합 검사 11개가 모두 통과했습니다. 0.05 m 해상도로 실제 SLAM 지도와 그래프를 저장하고 새 프로세스에서 복원되는 것까지 확인했습니다. 약 10 m 폐루프 주행에서 처음과 끝 스캔이 저장 그래프의 실제 constraint로 연결됐고, AMCL은 0.430 m·0.25 rad 초기 오차에서 시작해 지도 안 3 m 주행 뒤 위치 오차 0.0359 m, yaw 오차 0.00133 rad까지 수렴했습니다. 최종 스캔과 지도 장애물 거리의 p95는 0.05 m였습니다.

초기 자세를 넣어 줬다고 바로 충분히 수렴한다고 보지는 않습니다. 짧은 0.6 m 이동에서는 위치 오차가 0.222 m 남아 있었기 때문에, 충분한 라이다 관측과 이동 뒤에 수렴 여부를 확인해야 합니다. 이 결과는 공항 일부 구간의 정적 시뮬레이션 기준이며 공항 전체 지도를 완성했다는 뜻은 아닙니다.

정적 공항 일부 구간의 짧은 폐루프 주행으로 지도와 pose graph 생성, 재로딩, SLAM 종료 뒤 AMCL 전환, 초기 자세 이후 수렴, `map → odom` 단독 발행과 시각을 확인했습니다. AMCL 오차는 같은 기록의 시뮬레이션 truth와 비교해 위치 0.15 m 이하, yaw 0.20 rad 이하를 기준으로 잡았습니다.

RGB-D 낮은 장애물 기억과 자율 목표 주행은 P4 문서에서, 실물 지도 작성은 실물 단계에서 따로 다룹니다.
