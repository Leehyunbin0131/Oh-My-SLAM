# 패키지 구조와 구현 범위

`omslam_description`과 `omslam_sim`은 Gazebo 로봇·센서·EKF를 연결하고, `omslam_mapping`은 SLAM·지도/그래프 저장·AMCL을 제공합니다. P4의 `omslam_navigation`은 이 하단에 State Lattice+DWB와 이관한 인지·장애물 기억을 연결하며, `omslam_operator` 계열 패키지는 지도 중심 통합 제어·계측 콘솔을 제공합니다. 모의 입력·시뮬레이션·실물 주행의 검증 근거는 구분합니다.

| 패키지 | 책임 | 첫 구현 단계 |
| --- | --- | --- |
| `omslam_control_vendor` | 고정 `Oh-My-Control` 모듈의 설치·버전 확인 | P1 |
| `omslam_base` | ROS 구동 어댑터·기구학·원본 명령 수명·측정 odom·진단 | P1 |
| `omslam_description` | 원본 CAD 모델 연계, joint/frame, 장비별 extrinsic | P2 |
| `omslam_sim` | 공항 월드·보행자, 로봇 하단 어댑터·가상 센서·EKF 연결 | 환경 선행 구현 · P2 |
| `omslam_localization` | wheel-only/EKF 설정, IMU 입력과 TF 발행 선택 | P2–P3 |
| `omslam_mapping` | SLAM·map/graph 저장·map_server/AMCL | P3 |
| `omslam_navigation` | State Lattice+DWB 설정·움직임 집합·속도 체인·BT·제어 권한 가드·실행 모드 | P4 |
| `nav2_smac_planner` | 공식 1.3.13 패키지, 연속 충돌 좌표 보존·smoother 경계 및 최종 경로 차체 검증 패치 | P4 |
| `omm_perception`, `omm_perception_msgs`, `omm_costmap_layers` | 기존 인지·장애물 기억 cost 기능 이관 | P4 |
| `omslam_navigation_contracts` | 실제 planner·DWB 플러그인의 차체 충돌 경계 검사 실행 파일 | P4 검증 |
| `omslam_operator`, `omslam_operator_msgs`, `omslam_operator_support` | 지도 중심 통합 제어·모니터링 Qt 콘솔, 운영 제어 인터페이스 및 프로세스·지도 세션 관리 | P4 콘솔 |
| `omslam_system_monitor` | 호스트·로봇 자원(CPU·메모리·온도·디스크) 1 Hz 진단 발행 | P4 콘솔 |
| `omslam_bringup` *(예정)* | 실물/시뮬레이션 통합 상위 bringup 프로파일 (현재는 `omslam_navigation`·`omslam_operator_support`에서 모드 조합 제공) | P5–P8 예정 |

P1의 순수 기구학·시간 처리는 `omslam_base` 내부 모듈로 두며 불필요하게 많은 패키지로 나누지 않습니다. 기존 인지 패키지는 이름을 바꾸기 위해 재작성하지 않고 원래 이름과 출처를 유지합니다.

launch/config/RViz/BT는 해당 패키지의 share 경로에 설치합니다. 실행은 source tree의 파일 경로가 아닌 설치된 패키지 이름을 기준으로 합니다.
