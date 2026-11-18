# 출처와 라이선스

현재 저장소에는 통합 설계, 공항 시뮬레이션, ROS 구동 브리지·EKF·SLAM·자율주행 패키지가 포함됩니다. `omslam_description/upstream`의 원본 로봇 description·mesh는 고정 의존성의 필요한 파일을 포함합니다.

- 신규 코드·문서: Apache-2.0.
- `Oh-My-Control`: `omslam_control_vendor/control`에 고정 버전의 호스트 제어·보정·HFI-A9 decoder 모듈 6개를 포함합니다. 원본 Apache-2.0 LICENSE와 파일별 SHA256·기준 커밋은 `omslam_control_vendor/provenance.json`에 보존합니다. `WheelDrive`에는 원본 명령 시각 인수와 늦은 zero 명령의 즉시 STOP 처리를 추가했습니다.
- `Oh-My-MobileRobot`의 description·CAD·mesh: `omslam_description`에 base URDF·참조 mesh·원본 고지와 해시를 포함하며 원본 CERN-OHL-P-2.0을 유지합니다. 새 저장소의 Apache-2.0이 모델 라이선스를 대체하지 않습니다.
- 기존 `mobile_robot_sim`: `omm_perception_msgs`, `omm_perception`, `omm_costmap_layers`의 인지·장애물 기억 부분을 선택 이관합니다. 각 패키지의 Apache-2.0 LICENSE와 `provenance.json`에 원본 경로·파일별 SHA-256을 보존합니다. 모델/기존 sim 구성은 이 인지 패키지의 라이선스로 재분류하지 않습니다.
- State Lattice 움직임 집합: Navigation2 1.3.13의 `5cm_resolution/0.5m_turning_radius/diff/output.json`을 변경 없이 포함합니다. Apache-2.0 LICENSE·원본 경로·SHA-256은 `src/omslam_navigation/config/lattice/`에 설치합니다.
- RViz 설정: `omslam_navigation/rviz/navigation.rviz`는 Navigation2 1.3.13의 `nav2_bringup/rviz/nav2_default_view.rviz`에서 Selector·Docking 패널을 제외한 Apache-2.0 파생 설정입니다. 원본 SHA-256은 파일 머리말에 기록합니다.
- `nav2_smac_planner`: Navigation2 1.3.13의 고정 커밋 `f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501`에서 패키지를 포함합니다. 연속 격자 좌표를 충돌 검사까지 보존하는 패치, 지도 밖 smoothing 경계 곡선의 배열 접근 방지 패치와 출처 설치 규칙을 적용했습니다. 원본 Apache-2.0 LICENSE·파일별 원본/변경 해시·패치는 `src/nav2_smac_planner/`에 보존하고 패키지 share에 설치합니다. 패키지와 공개 플러그인 이름은 원본을 유지합니다.
- 공항 geometry·가구·표지·생성기·보행자 제어 플러그인: 이 프로젝트에서 작성한 Apache-2.0 자산·코드.
- 보행자 `walk.dae`: Gazebo Fuel Mingfei/actor version 1, CC BY 4.0. 원본은 변경하지 않았으며 출처·라이선스·해시는 `src/omslam_sim/models/omslam_pedestrian/NOTICE`와 `asset_manifest.json`에 기록합니다.
- `airport_gui_ogre.config`, `airport_gui_ogre2.config`: Gazebo Sim 8.15.0의 기본 `gui/gui.config`에서 창 크기·카메라·색·렌더러·초기 대화상자 설정을 변경했습니다. Gazebo Sim 프로젝트의 Apache-2.0 출처는 https://github.com/gazebosim/gz-sim 입니다.

고정 커밋은 [dependencies.repos](dependencies.repos), 현재 조사 근거는 [baseline](docs/baseline.md)에 있습니다. 외부 소스를 이관하는 커밋에서 파일별 출처와 적용 라이선스를 갱신합니다.
