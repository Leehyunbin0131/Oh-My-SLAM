# 공항 실내 시뮬레이션 환경

[README](../README.md) · [통합 설계](architecture.md) · [구현 계획](roadmap.md)

`omslam_sim`은 Gazebo Harmonic에서 실행하는 **가상 하늘 국제공항(Hanul International)** 환경을 제공합니다. 인천공항의 넓은 대합실·탑승구·상업시설·실내 정원을 참고한 단일 층의 2터미널 테스트 공간입니다. T1·T2와 연결 동선은 시뮬레이션을 위해 설계한 가상 배치입니다.

이 패키지의 제공 범위는 정적 환경, 걷는 보행자, ROS 시각 연결 및 기하 기반 참고 지도입니다. 로봇 모델·센서·EKF를 포함하는 실행은 [로봇 시뮬레이션](sim_robot.md)을 따릅니다. SLAM·State Lattice+DWB 주행 구성은 후속 단계에서 연결합니다.

![공항 평면도](../src/omslam_sim/maps/airport_floorplan.png)

## 공간 구성

| 항목 | 구성 |
| --- | --- |
| 외곽 범위 | X: −122~38 m, Y: −62~62 m; 160×124 m |
| 실내 바닥 | 약 16,992 m², 단일 층·여러 연결 루프 |
| 터미널 | T1·T2, 각각 약 68×92 m 라운지와 남측 서비스 구역 |
| 주 동선 | 가구 배치 사이 중앙 통로 약 6.7 m, 교차 통로 약 4.3 m; 배치 기준에서 각각 6.4 m·3.8 m의 통행 공간 확보 |
| 환승 갤러리 | 좌석군으로 양쪽을 채우고 중앙 약 3.6 m 통행 공간 확보 |
| 보조 동선 | 2.8 m·3.6 m 서비스 복도, 1.4 m 병목과 90° 코너 |
| 탑승 대기석 | 2,440석(305개 벤치군); 카페·라운지 좌석은 별도 |
| 카페·라운지 | 카페 탁자 37개·스툴 148개, 커피 탁자 25개, 라운지 의자 52개 |
| 추가 시설 | 매대 54개, 낮은 칸막이 7개, 키오스크 23개, 화분·수목 61개, 카운터·표지·수하물·서비스실 |
| 구역별 특성 | T1 탑승 대기, T2 정원·라운지 |
| 초기 기준 위치 | `world` 좌표 `(−108, 0, 0)`, yaw=0; 환경 전용 launch에서는 로봇 미생성 |
| 보행자 | 32명, 라운지 주변과 연결 통로의 폐루프 동선, 시뮬레이션 시각에 따른 재현 가능한 이동 |

원본 배치·장애물 외곽·통로 폭·관찰 시점·보행 경로는 [airport_layout.json](../src/omslam_sim/config/airport_layout.json)에 저장합니다. 재질별로 정적 visual mesh를 합치고 단순 collision을 별도로 사용하여 큰 환경의 렌더링·물리 연산량을 줄입니다. 천장 대신 개방형 상부 구조를 사용하므로 조감도에서 내부를 확인할 수 있습니다.

라운지는 좌석군·카페·매대·화분·칸막이로 분절하며 가구 사이 약 1.4~3 m 통로를 둡니다. 통행 공간과 보행 경로를 먼저 확보한 뒤 주변을 채우므로, 열린 홀 전체를 큰 장애물 하나로 대체하지 않습니다. 보행자는 터미널당 16명이며 긴 주 동선과 카페·좌석 주변의 짧은 동선에 나누어 배치합니다.

## 시뮬레이터와 환경

대상은 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, `ros_gz` 1.0.24입니다. GUI는 Ogre를 기본으로 사용하고, 센서 서버는 Ogre2의 headless 렌더링을 사용합니다. 물리 시간 간격은 0.004 s입니다.

| 도구 | 이 환경에서의 선택 기준 |
| --- | --- |
| **Gazebo Harmonic** | ROS 2 시각·카메라·GPU LiDAR 연결, SDF 자산, actor 애니메이션을 이용하는 기준 실행 환경 |
| Isaac Sim / Isaac Lab | RTX 기반 센서·렌더링 작업에 적합한 별도 NVIDIA GPU 환경이 필요하므로 현재 Intel 내장 GPU PC의 기준 실행 경로로 사용하지 않음 |
| MuJoCo | 물리·제어 시뮬레이션에 사용할 수 있으나, 이 공항 환경에서는 기존 ROS 2 센서·SDF 연동을 활용하는 Gazebo를 선택 |

이는 해당 PC와 프로젝트의 통합 조건에 따른 선택이며 시뮬레이터 간 성능 비교 실험 결과가 아닙니다. [Gazebo ROS 2 연동](https://gazebosim.org/docs/harmonic/ros2_integration/) · [Isaac Sim 요구 환경](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) · [MuJoCo 개요](https://mujoco.readthedocs.io/en/stable/overview.html)

## 빌드와 실행

새 터미널에서 ROS underlay를 선택하고 해당 패키지를 빌드합니다. ROS용 Python을 명시하여 다른 로컬 Python 환경이 CMake에 선택되지 않게 합니다.

```bash
cd ~/MyRos/Oh_My_SLAM
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to omslam_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

빌드 성공 후 실행합니다.

```bash
./sim.sh
```

| 실행 | 의미 |
| --- | --- |
| `./sim.sh` | GUI·로봇·센서·EKF와 32명 보행자 |
| `./sim.sh pedestrians:=false` | 보행자 없는 공항과 로봇·센서 |
| `./sim.sh gui:=false` | 화면 없이 서버·센서 실행 |
| `./sim.sh gui_renderer:=ogre2` | GUI도 Ogre2로 선택 |
| `ros2 launch omslam_sim airport.launch.py` | 로봇 없는 환경 전용 실행(install source 필요) |

`Ctrl+C`로 종료합니다. 실행 래퍼는 자신의 Gazebo 서버에 종료를 요청한 뒤 자신이 시작한 프로세스만 정리합니다. GUI의 초기 시점은 공항 전체 조감도이며, 마우스로 이동·확대·회전할 수 있습니다.

`sim.sh`는 기본 ROS domain 86과 전용 Gazebo partition을 사용합니다. `OMSLAM_SIM_DOMAIN_ID`, `OMSLAM_GZ_PARTITION`으로 실행 간 격리를 지정할 수 있습니다. ROS 로그의 기본 위치는 `reports/airport/live/ros_logs`입니다.

로컬 Gazebo 통신은 기본 `GZ_IP=127.0.0.1`을 사용해 Wi-Fi 주소 변경의 영향을 피합니다. 명시적으로 지정한 `GZ_IP`는 유지합니다.

직접 launch할 경우 새 install을 source합니다. launch 파일은 자산·플러그인 경로를 설치된 패키지에서 찾습니다.

```bash
source install/setup.bash
ros2 launch omslam_sim airport.launch.py gui:=true pedestrians:=true
```

## 보행자 모델

걷기 mesh·관절 애니메이션을 가진 actor와 반경 0.30 m·높이 1.75 m의 원통 collision proxy가 같은 동선을 따라 움직입니다. Actor는 RGB·GPU 센서용이며 proxy가 물리 충돌 형상을 제공합니다. 동선은 좌석·카페 주변의 짧은 경로와 터미널 연결 경로를 나눠 정해진 폐루프를 반복하는 방식이고, 보행자 간 협상이나 로봇을 보고 피하는 사회적 군중 모델은 포함하지 않습니다.

`omslam::AirportPedestrians`가 이동·애니메이션을 시뮬레이션 시각으로 갱신하며 pause와 시간 초기화를 처리합니다. `/airport/pedestrians/poses`의 `gz.msgs.Pose_V`에는 actor와 물리 처리 후 proxy의 위치가 최대 5 Hz로 전달됩니다. 이 값은 환경 검증용이며 로봇 위치추정 입력으로 사용하지 않습니다.

모델은 Gazebo Fuel의 Mingfei/actor version 1 자산을 로컬로 포함합니다. 출처·CC BY 4.0 조건·파일 해시는 [자산 NOTICE](../src/omslam_sim/models/omslam_pedestrian/NOTICE)와 [manifest](../src/omslam_sim/models/omslam_pedestrian/asset_manifest.json)에 있습니다. 실행 시 Fuel 다운로드는 필요하지 않습니다. [Gazebo actor 특성](https://gazebosim.org/docs/harmonic/actors/)

## 지도와 좌표

- [평면 안내도](../src/omslam_sim/maps/airport_floorplan.png): 터미널·통로·가구·보행 동선 표시.
- [reference map](../src/omslam_sim/maps/airport_reference.yaml): 정적 collision과 바닥 영역에서 생성한 0.1 m 격자 지도. 보행자는 포함하지 않습니다.
- Gazebo `world` 좌표는 미터 단위, +Z가 위쪽입니다. 참고 지도 origin과 해상도는 YAML을 따릅니다.

reference map은 기하를 아는 시뮬레이터에서 만든 검증 자료입니다. SLAM으로 관측·작성한 지도로 판정하거나 새로운 SLAM 결과를 대신하지 않습니다. 로봇의 기본 주행 구성은 State Lattice+DWB이며 보로노이 전용 레이어는 적용하지 않습니다.

## 재생성과 수정

공간·가구·정적 collision·기준지도는 하나의 생성기에서 재생성합니다. 이어서 보행자를 결합하고 다시 빌드합니다.

```bash
/usr/bin/python3 tools/generate_airport.py
/usr/bin/python3 tools/add_airport_pedestrians.py \
  --world src/omslam_sim/worlds/airport_static.sdf \
  --layout src/omslam_sim/config/airport_layout.json \
  --output src/omslam_sim/worlds/airport.sdf
```

`generate_airport.py --package PATH`는 다른 출력 패키지를 지정합니다. 보행자 경로의 waypoint는 `(x,y)` 미터, speed는 m/s, phase는 시뮬레이션 초 단위입니다. 생성 파일을 직접 수정하면 재생성 시 덮어쓰므로 반복 가능한 수정은 생성기에 반영합니다.

## 환경 검증

`tools/verify_airport.py`는 새 설치본을 기준으로 정적 배치·연결성, Gazebo 실행·시각, 보행자 이동·proxy 정합, RGB 렌더링·GPU LiDAR 반환을 한 실행에서 확인하고 이미지·영상을 저장합니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 tools/verify_airport.py --output reports/airport/manual-check
```

검증 결과는 해당 출력 디렉터리에 기록합니다. 환경 검증 성공은 로봇의 오도메트리·SLAM 지도 품질·자율 목표 도착 검증을 의미하지 않습니다. 빌드가 실패한 경우 이 검증을 실행하지 않습니다.
