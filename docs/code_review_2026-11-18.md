# ROS 2 로봇 운용 코드 리뷰

수정 전 백업 커밋 `e9e34ad`에서 확인한 문제는 **32건**이다. **P1 8건**은 잘못된 주행 명령·목표·도착 성공 또는 장애물 누락으로 이어지는 문제다. **P2 23건**은 제공하는 기능이나 인수 검사가 명시된 조건에서 실패한다. **P3 1건**은 구형 검증 도구가 현재 GUI와 다른 화면을 시험하는 문제다.

P1은 **모터 정지(1번)**, **도착과 출발 자세 검사(8·9번)**, **낮은 장애물 감지와 기억(14·15번)**, **여러 콘솔의 지도·목적지·속도 제어(19·20·23번)**에 해당한다. 아래의 완료 기준은 후속 수정이 해당 문제를 해결했는지 판정하는 조건이다.

## 수정 후 확인

| 항목 | 변경한 동작 | 확인 방법 |
| --- | --- | --- |
| 1–4 | 정지 접수 후 이전 바퀴 명령을 폐기하고, MCU 고장 또는 시계 되감기 뒤에는 새 피드백과 활성화를 요구한다. 바퀴 RPM 포화를 구동 계층의 수치 범위 안에서 처리한다. | [직렬 전송 시험](../src/omslam_base/test/test_serial_worker.py) · [시뮬레이션 구동 시험](../src/omslam_sim/test/test_sim_guard.py) |
| 5–7 | 정지 요청 뒤 시각이 다른 오도메트리 표본으로 연속 정지를 확인한다. 횡이동도 병진 속도에 포함하고 정지 처리 중에는 새 제어권을 보류한다. | [정지 표본 시험](../src/omslam_navigation/test/test_measured_stop.py) · [제어권 시험](../src/omslam_navigation/test/test_operator_guard.py) |
| 8–13 | 원래 목표에 대한 위치·방향·속도를 함께 재검사한다. 실제 출발 자세부터 연결 구간을 검사하고 각 진입 경로의 비용과 대칭 heading을 처리한다. 목표 상태의 작은 시각 차이와 콘솔의 저속 운전을 지원한다. | 당시 `src/nav2_smac_planner/test/test_lattice_terminal.cpp` · [설치 DWB 주행·정체 시험](../src/omslam_navigation/test/test_installed_operator_motion.py) |
| 14–18 | 넓은 단차 윗면을 바닥과 구별한다. 관측이 끊기면 장애물 삭제 투표를 초기화하고, 보관소에서 가까운 장애물을 복원한다. 이전 시간대의 스냅샷과 유효기간이 지난 동적 TF를 비용지도의 새 관측으로 사용하지 않는다. | [바닥·단차 시험](../src/omm_perception/test/test_geometry.cpp) · [장애물 기억 시험](../src/omm_perception/test/test_memory.cpp) · [비용지도 투영 시험](../src/omm_costmap_layers/test/test_memory_layer_projection.cpp) |
| 19–22 | 지도 작업과 구동 권한을 서버에서 조정한다. 모든 콘솔이 같은 지도 세션과 AMCL 초기화 상태를 사용한다. 저장은 검증된 파일을 준비한 뒤 YAML을 마지막에 게시하므로 중간 실패 후 같은 이름으로 재시도할 수 있다. | [지도 작업 시험](../src/omslam_operator_support/test/test_map_operations.py) · [실패·재시도 시험](../src/omslam_mapping/test/test_bundle_transaction.py) |
| 23–28 | 거절된 RPM과 전송 전 취소된 목표를 폐기한다. 승인 대기 중 조이스틱 입력을 유지하고, 다른 창의 경로도 목표 상태와 함께 갱신한다. 경로 기록에는 수집한 플러그인 설정과 원본 DWB 명령을 포함한다. | [GUI 상태 시험](../src/omslam_operator/test/test_live_model.py) · [경로 수명 시험](../src/omslam_operator/test/test_path_lifecycle.py) · [기록기 시험](../src/omslam_operator_support/test/test_debug_lifecycle.py) |
| 29–32 | 모든 실행기가 월드별 기본 생성 위치를 공유한다. 인수 도구는 설치 지문을 기동 전에 검증하고, 최종 Gazebo 명령의 바퀴 RPM을 실행 중인 베이스 설정과 비교한다. 두 GUI 검증 명령 모두 현재 콘솔의 QML을 연다. | [월드별 생성 위치 시험](../src/omslam_navigation/test/test_world_spawn.py) · [인수 도구 시험](../tools/test_navigation_acceptance.py) · [빌드 및 주행 절차](sim_navigation.md) · [GUI 검증 절차](operator_gui.md) |

사용자가 남긴 원형 경로 두 건에 수정된 설치 플러그인을 적용한 결과, 목표 부근의 3.482 m 경로는 0.058 m, 3.372 m 경로는 0.087 m로 줄었다. 1·2 RPM의 DWB 시험에서는 20.5초간 각각 0.121 m와 0.242 m를 전진했고, 이후 정지 상태를 유지했을 때는 두 경우 모두 10.1초에 진행 실패를 검출했다. 이 재생과 시험에 사용한 입력은 시뮬레이션 기록과 모의 입력이다.

Gazebo 코스에서 탐색 단계의 차체 충돌 검사도 보강했다. 차체 끝이 미관측 셀을 지나는 후보를 탐색 중 제외하고, 장애물 셀의 누적 개수로 빈 구간의 중복 검사를 줄였다. 실패를 기록한 비용지도 네 장에서 기존 2초 예산으로 0.13–0.26초 안에 유효한 경로를 찾았다. 최종 설치본은 한 실행에서 목표 5곳에 연속 도착했다. 충돌과 복구 재시도는 모두 0회였고, 매번 새 오도메트리로 정지를 확인했다. 모의 직렬 장치와 Gazebo에서 검증했으며 실물 주행 검증은 별도다. [주행 기준과 측정값](sim_navigation.md#검증과-결과)

## 검토 범위와 확인 방법

리뷰는 커밋 `8040e2d7` 위의 작업 트리와 당시 미추적 패키지를 대상으로 했다. 아래 재현 결과와 코드 줄 번호는 수정 전 백업 `e9e34ad`를 기준으로 읽는다. ROS 패키지 16개, 셸 진입점 3개, 검증·생성 도구, 설정·모델·코스 및 운용 문서를 대조했다.

| 운용 흐름 | 함께 대조한 구현 |
|---|---|
| 명령에서 바퀴까지 | 실물 베이스, 고정 버전 모터 통신·펌웨어의 관련 경로, 시뮬레이션 베이스, Gazebo 브리지, 모델 기하, 속도·정지 가드 |
| 센서에서 경로까지 | IMU·엔코더·EKF, SLAM·AMCL, TF, RGB-D 장애물 기억과 메시지, 전역·지역 비용지도, Lattice·DWB·도착 판정, 행동 트리·액션 |
| 조작에서 화면까지 | Qt/QML, GUI 모델·작업 스레드, 조작 메시지·서비스, 제어권·경로 목록, 지도·금지 구역 저장 및 복원, 호스트 진단·경로 기록 |
| 실행 및 인수 | 콘솔·RViz·시뮬레이션 실행기, 패키지 설치 규칙, 플러그인 계약 검사, 지도·주행·GUI 검증 도구와 기존 시험 자료 |

작업공간에 설치된 Lattice·비용지도·인지 라이브러리와 ROS 2 Jazzy의 Nav2 Controller 1.3.14에 경계 입력을 넣었다. 베이스와 콘솔은 실제 제품 함수와 QML을 사용하고 직렬 장치, ROS 전송 또는 센서 입력을 필요한 지점에서 대체했다. 실물 구동과 Gazebo 전체 코스 재주행은 수행하지 않았다. 저장된 실행 기록, 이번에 실행한 검사, 소스·설정 대조 결과를 각 항목에 구분해 적었다. 외부 패키지는 이 제품이 호출하는 경로를 확인했다. 실물 자율주행은 프로젝트 로드맵상 후속 단계다. [README.md:83 · 현재 제공 범위](../README.md#L83)

현재 GUI의 **지도 → 지도 저장**과 지도·금지 구역 파일의 해시 결합은 구현돼 있다. 미관측 셀의 목적지를 막는 것도 GUI의 명시적 검사이고, 활성 계획기 설정은 미관측 영역 통과를 허용하지 않는다. [DrivingPage.qml:237 · 저장 메뉴](../src/omslam_operator/qml/design/DrivingPage.qml#L237) · [omslam_operator_support/session.py:138 · 구역 저장](../src/omslam_operator_support/omslam_operator_support/session.py#L138) · [omslam_operator/model.py:142 · 목표 셀 검사](../src/omslam_operator/omslam_operator/model.py#L142) · [nav2_common.yaml:26 · 계획기 설정](../src/omslam_navigation/config/nav2_common.yaml#L26)

## 구동, 정지 및 제어권

### 1. P1 · 정지 접수 후 이전 바퀴 목표가 한 번 더 전송된다

베이스 작업 스레드는 잠금 안에서 목표를 복사하고 잠금을 푼 뒤 모터 구동 승인(`ARM`) 응답을 기다린다. 그 사이 ROS 콜백이 정지를 접수해도 전송 직전에는 명령 만료와 엔코더 상태만 확인한다. 실제 작업 스레드와 기록용 직렬 장치에서 전송 순서는 **ARM → 정지 접수 → 양쪽 14.04 RPM → 양쪽 0 RPM**이었다. [omslam_base/node.py:251 · 목표 복사와 전송 호출](../src/omslam_base/omslam_base/node.py#L251) · [omslam_base/node.py:172 · 정지 입력](../src/omslam_base/omslam_base/node.py#L172) · [wheel_drive.py:96 · ARM 응답 후 검사](../src/omslam_control_vendor/control/wheel_drive.py#L96) · [wheel_drive.py:120 · 속도 전송](../src/omslam_control_vendor/control/wheel_drive.py#L120)

**수정 및 완료 기준** — 정지·비활성화가 이전 명령을 무효화하도록 명령 세대를 두고, 최종 유효성 확인과 전송 순서를 직렬화한다. ARM 응답을 늦춘 시험에서 정지 접수 이후 이전 세대의 비영 속도 프레임이 0건이어야 한다.

### 2. P2 · 시뮬레이션 시계를 되감으면 바퀴 피드백 없이 베이스가 다시 출력한다

되감기 콜백은 마지막 오도메트리를 지운다. 구동 가드는 이미 활성화돼 있으면 새 시간대의 명령을 받아들이고, 감시기는 마지막 오도메트리가 존재할 때만 만료를 검사한다. 실제 콜백에 시계 8초 → 1초와 새 원본·최종 명령 쌍을 넣자 새 바퀴 표본 0건으로 **0.12 m/s**가 출력됐다. 상위 Nav2 가드에는 별도 감시가 있으며 이 재현은 베이스 자신의 출력 경계에서 확인했다. [sim_base.py:115 · 시계 처리](../src/omslam_sim/omslam_sim/sim_base.py#L115) · [omslam_sim/control.py:29 · 명령 재개](../src/omslam_sim/omslam_sim/control.py#L29) · [sim_base.py:172 · 출력 전 검사](../src/omslam_sim/omslam_sim/sim_base.py#L172)

**수정 및 완료 기준** — 되감기 시 구동 승인을 철회하고 새 시간대의 바퀴 표본과 재활성화를 요구한다. 새 명령만 들어오는 동안 `/sim/cmd_vel`은 0을 유지해야 한다.

### 3. P2 · 정지 중 MCU 고장을 보고해도 베이스 활성화가 성공한다

목표가 없으면 작업 스레드가 `WheelDrive.tick()`을 건너뛴다. 활성화 조건은 호스트 오류·링크 준비·최근 엔코더 시각만 확인한다. 신선한 MCU 텔레메트리에 드라이버 I/O 고장 코드 2를 넣어도 활성화 결과는 `SUCCESS`였다. 첫 주행 시도에서는 드라이버가 차단한다. 고정된 펌웨어도 정지 출력 재시도 중 이 고장을 발생시킬 수 있다. [omslam_base/node.py:103 · 베이스 활성화](../src/omslam_base/omslam_base/node.py#L103) · [omslam_base/node.py:253 · 목표 없는 분기](../src/omslam_base/omslam_base/node.py#L253) · [wheel_drive.py:79 · 드라이버 고장 검사](../src/omslam_control_vendor/control/wheel_drive.py#L79) · [고정 펌웨어 504행](https://github.com/Leehyunbin0131/Oh-My-Control/blob/5d5d65b6ba208383d3dede73ffb8a108cf786a76/firmware/mega_motor_test/mega_motor_test.ino#L504)

**수정 및 완료 기준** — 수신한 MCU 오류를 목표 유무에 관계없이 베이스 고장 상태와 진단에 반영한다. 정상 주기의 엔코더가 계속 들어와도 고장 중에는 활성화가 실패하고 복구 절차 후에만 성공해야 한다.

### 4. P2 · 정상적으로 제한한 RPM이 부동소수점 오차로 모터 고장을 일으킨다

차동 기구학이 좌우 RPM을 같은 비율로 줄인 뒤, 직렬 드라이버가 결과를 정확한 상한과 다시 비교한다. 50 RPM 설정에서 선속도 0.34 m/s와 각속도 0.52 rad/s를 넣으면 우측 결과는 `50.00000000000001`이다. 실제 드라이버는 `MotorError`와 `STOP`을 냈다. 95 RPM 설정에서도 같은 결과를 재현했다. [omslam_base/core.py:26 · 포화 계산](../src/omslam_base/omslam_base/core.py#L26) · [wheel_drive.py:48 · 직렬 상한 확인](../src/omslam_control_vendor/control/wheel_drive.py#L48)

**수정 및 완료 기준** — 비율 축소 후 각 바퀴 값을 정확한 상한에 고정하거나 전송 단위인 0.01 RPM으로 일관되게 변환한다. 두 재현 입력은 상한 이내로 전송하고 실제 초과값에 대한 드라이버 검사는 유지한다.

### 5. P2 · 종료와 인수 시험이 새 오도메트리 없이 정지 확인을 통과한다

RViz 종료기는 마지막 0속도 표본이 0.5초 이내라는 조건을 0.2초간 반복한다. 주행 검증 도구도 같은 표본을 읽으며 시뮬레이션 시간만 0.3초 지났는지 본다. 두 검사에 정지 요청 전 표본 하나만 공급했더니 종료기는 0.22초 뒤 정지를 기록했고 인수 검사도 0.30초에 통과했다. 비GUI 구동 해제 서비스 자체는 액션 종료와 베이스 비활성만 확인한다. [nav.sh:197 · 종료 검사](../nav.sh#L197) · [verify_sim_navigation.py:373 · 인수 시험](../tools/verify_sim_navigation.py#L373) · [navigation_guard.py:674 · 서비스 성공 조건](../src/omslam_navigation/omslam_navigation/navigation_guard.py#L674)

**수정 및 완료 기준** — 요청 이후 원본 시각이 증가하는 복수의 측정 표본으로 정지 지속시간을 계산한다. 표본 유실·중복·동결 시 정지 확인 실패를 기록하고, 정상적인 연속 측정으로만 성공해야 한다.

### 6. P2 · 조작 가드가 오도메트리에 보고된 횡이동을 정지로 판정한다

가드는 `vy`가 유한한지는 확인하지만 정지 판정에는 `vx`와 `wz`만 전달한다. 바퀴는 정지이고 필터 오도메트리가 `vy=0.25 m/s`를 계속 보고하는 메시지로 실제 콜백을 실행하자 다섯 번째 표본에서 `stopped=true`가 됐다. 이 플래그는 모드 전환과 지도 작업에도 쓰인다. 기본 EKF는 휠의 종방향 속도와 yaw 각속도를 융합한다. 이 시험은 이미 오도메트리에 보고된 횡속도의 처리 범위를 확인한다. [navigation_guard.py:298 · 오도메트리 검사·전달](../src/omslam_navigation/omslam_navigation/navigation_guard.py#L298) · [operator_core.py:149 · 정지 계산](../src/omslam_navigation/omslam_navigation/operator_core.py#L149) · [ekf.yaml:20 · EKF 입력](../src/omslam_localization/config/ekf.yaml#L20)

**수정 및 완료 기준** — 가용한 평면 병진 속도는 `hypot(vx, vy)`로 확인하고 회전·신선도·지속시간 검사도 유지한다. 같은 메시지에서는 정지 승인을 내리지 않아야 한다. [Nav2 도착 시 정지 판정](https://github.com/ros-navigation/navigation2/blob/1.3.14/nav2_controller/plugins/stopped_goal_checker.cpp#L78)

### 7. P2 · 정지 처리 도중 부여한 제어권이 정지 완료와 동시에 사라진다

정지 요청이 기존 소유자를 지운 뒤, 다른 콘솔의 `acquire`는 진행 중 전환 검사보다 먼저 처리된다. 실제 서비스 콜백은 새 콘솔에 성공과 토큰을 반환했고, 이어진 정지 완료 처리가 새 소유자도 지웠다. 콘솔은 취득 성공 직후 첫 명령부터 거절된다. [operator_control.py:109 · 취득 처리 순서](../src/omslam_navigation/omslam_navigation/operator_control.py#L109) · [operator_control.py:149 · 정지 시작](../src/omslam_navigation/omslam_navigation/operator_control.py#L149) · [operator_control.py:313 · 정지 완료](../src/omslam_navigation/omslam_navigation/operator_control.py#L313)

**수정 및 완료 기준** — 정지 전환 중에는 새 취득을 거부하거나 완료까지 보류한다. 정지 직전·진행 중·직후 요청에서 성공 응답에 딸린 토큰은 해당 소유자가 사용할 수 있어야 한다.

## 전역 계획, 도착 및 자동 주행

### 8. P1 · 방향을 맞추는 중 10 cm 밖으로 밀려도 도착 성공을 반환한다

공유된 위치 허용오차는 0.10 m지만 `StoppedGoalChecker`의 `stateful` 설정이 켜져 있다. 한 번 범위 안에 들어오면 다음 경로로 판정기가 초기화될 때까지 위치 검사를 생략한다. 설치본에서 **9 cm·방향 불일치 → 미도착**, 이어 **13 cm·방향 일치·정지 → 도착**, 같은 13 cm를 초기화 후 검사하면 **미도착**이었다. 조작 액션은 Nav2 성공 뒤 측정 정지만 추가 확인한다. 이전 근접 경로 재생에서 3.482 m 고리가 0.058 m로 줄어든 것은 별도로 확인된 결과다. 기존 범위 밖 시험은 검사 직전에 판정기를 초기화했다. [sim_profile.yaml:19 · 10 cm 설정](../src/omslam_navigation/config/sim_profile.yaml#L19) · [nav2_common.yaml:46 · 판정기 설정](../src/omslam_navigation/config/nav2_common.yaml#L46) · [operator_control.py:254 · 상위 액션 성공](../src/omslam_navigation/omslam_navigation/operator_control.py#L254) · [code_review.md:28 · 기존 리뷰](code_review.md#L28) · [Nav2 위치 검사 100행](https://github.com/ros-navigation/navigation2/blob/1.3.14/nav2_controller/plugins/simple_goal_checker.cpp#L100)

**수정 및 완료 기준** — 최종 도착 판정에서 해당 주기의 원래 목표 위치·방향·측정 정지를 함께 요구한다. 목표 판정기의 XY 기억을 해제하는 방법부터 검토한다. 동일 목표를 초기화 없이 9 → 13 → 9 cm로 시험해 13 cm에서는 성공 및 다음 목적지 전송이 없어야 한다. 근접 회전·감속·차체 충돌 검사도 함께 확인한다.

### 9. P1 · 전역 경로가 실제 시작 자세에서 첫 경로점까지의 충돌을 놓친다

일반 Lattice 탐색은 시작 방향을 이산화한다. 경로 변환은 기본 자세를 이전 점으로 놓고 중복을 지워, 지도 원점·방향 0으로 이산화된 첫 점마저 지울 수 있다. 최종 검사는 반환 경로의 첫 점에서 시작한다. 설치본에 시작 `(0, 0, 0.20 rad)`, 목표 `(0.8, 0, 0)`과 차체 모서리에 걸리는 한 셀을 주자 실제 시작 footprint는 충돌이었지만 첫 점 `(0.0727, 0, 0)`부터의 12점 경로가 유효로 반환됐다. (`src/nav2_smac_planner/src/smac_planner_lattice.cpp:341` 시작 이산화 · `:471` 첫 점 제거 · `src/nav2_smac_planner/include/nav2_smac_planner/path_validation.hpp:71` 검사 시작점)

**수정 및 완료 기준** — 원래 연속 위치·yaw의 차체와 첫 경로점까지의 병진·회전 구간을 검사한다. 중복 비교는 실제 이전 점이 생긴 다음부터 수행한다. 평활화 on/off 모두 해당 입력은 실패해야 하고 장애물을 치우면 실제 시작 자세부터 유효한 경로가 나와야 한다.

### 10. P2 · 같은 격자점으로 들어오는 서로 다른 경로가 충돌 판정을 공유한다

노드에 저장한 유효성·비용에는 그 점까지 들어온 이동 경로의 중간 충돌 결과가 포함된다. 다른 방향에서 같은 노드를 재방문하면 다른 경로를 검사하지 않고 이 값을 쓴다. 설치된 Lattice에서 두 이동 primitive를 독립 검사하면 각각 통과·충돌이다. 같은 노드를 재사용하면 먼저 검사한 순서에 따라 **둘 다 통과**하거나 **둘 다 실패**했다. 최종 경로 검사가 충돌 경로를 거절해도 탐색에서 버린 대안은 회복되지 않는다. (`src/nav2_smac_planner/src/node_lattice.cpp:230` 캐시 재사용 · `:250` 중간 자세 검사 · `:554` 이웃 확장)

**수정 및 완료 기준** — 노드에는 끝점 자체의 결과만 저장하고, 들어오는 구간의 비용과 충돌은 시작점·방향·이동 primitive별로 검사한다. 두 방문 순서에서 같은 통과·충돌 결과와 유효한 대안 경로를 얻어야 한다.

### 11. P2 · 좌우 대칭인 상황에서 Lattice 거리 추정값이 크게 달라진다

음의 Y를 반사할 때 방향 인덱스 0을 방향 개수 `N`으로 바꿔 다음 행의 거리 값을 읽는다. 설치된 Lattice에서 같은 방향, 상대 격자 좌표 `(-10, +2)`와 `(-10, -2)`의 추정값이 **10.24셀과 73.27셀**이었다. 대칭인 공간에서 탐색 우선순위가 방향에 따라 달라진다. 함께 포함된 Hybrid 구현에도 같은 식이 있다. (`src/nav2_smac_planner/src/node_lattice.cpp:419` 활성 Lattice · `src/nav2_smac_planner/src/node_hybrid.cpp:758` Hybrid)

**수정 및 완료 기준** — 반사 인덱스를 항상 `0..N-1`로 정규화한다. 방향 0을 포함해 모든 방향에서 좌우 반사쌍의 거리가 일치해야 한다.

### 12. P2 · 정상 목표 상태가 로컬 시계보다 4 ms 먼저 오면 주행 가드가 고장으로 고정된다

RViz 주행 경로의 가드는 액션 서버의 목표 수락 시각이 현재 로컬 `/clock`보다 크면 과거 활성화의 목표와 함께 거절한다. 현재 코드에 2.200초의 시계와 2.204초의 새 목표 상태를 넣자 `FAULT_LATCHED`와 취소 요청이 생겼고 시계가 따라잡아도 남았다. 별개 토픽의 도착 순서에 민감하다. [navigation_guard.py:366 · 상태 콜백](../src/omslam_navigation/omslam_navigation/navigation_guard.py#L366) · [guard_core.py:200 · 시각·세대 판정](../src/omslam_navigation/omslam_navigation/guard_core.py#L200)

**수정 및 완료 기준** — 같은 활성화에서 조금 앞선 상태는 한정된 시간 동안 보류하고 시계가 따라잡으면 다시 판정한다. 보류 중 출력은 0이어야 한다. 실제 이전 세대·취소된 목표·허용 지연을 초과한 목표는 계속 차단한다.

### 13. P2 · GUI가 허용한 1 RPM으로 정상 전진해도 자동 주행은 진행 실패가 된다

AUTO 1 RPM의 직진 상한은 약 0.00712 m/s, 회전 상한은 0.006 rad/s다. 진행 판정기는 10초 이내 0.10 m 이동 또는 0.10 rad 회전을 요구한다. 설치된 판정기에 등속 궤적을 넣으면 1 RPM은 10.1초에 직진 0.0719 m와 회전 0.0606 rad 모두 실패하고, 같은 시험의 2 RPM은 둘 다 통과했다. 같은 진행 판정기를 쓰는 전역 재계획은 이 시간을 초기화하지 않는다. [live_model.py:625 · GUI 입력](../src/omslam_operator/omslam_operator/live_model.py#L625) · [operator_core.py:197 · AUTO 상한](../src/omslam_navigation/omslam_navigation/operator_core.py#L197) · [sim_profile.yaml:14 · 진행 판정 설정](../src/omslam_navigation/config/sim_profile.yaml#L14) · [Nav2 거리·회전 판정](https://github.com/ros-navigation/navigation2/blob/1.3.14/nav2_controller/plugins/pose_progress_checker.cpp#L46) · [재계획 시 처리](https://github.com/ros-navigation/navigation2/blob/1.3.14/nav2_controller/src/controller_server.cpp#L694)

**수정 및 완료 기준** — 승인된 RPM과 실제 적용되는 선·각속도 상한에 따라 유효한 최소 진행량 또는 허용시간을 정한다. 1·2 RPM에서 장애물 없는 장거리 전진과 제자리 회전은 진행을 유지하고, 실제로 멈춘 경우는 유한 시간 안에 검출해야 한다.

## 낮은 장애물, 기억 및 비용지도

### 14. P1 · 넓은 3 cm 단차가 정상 바닥으로 선택된다

바닥 후보는 높이 편차 4 cm까지 허용하고 가장 많은 점을 지지하는 평면을 택한다. 장애물 검사는 그 평면보다 2 cm 이상 높은 점에서 시작한다. 설치된 인지 라이브러리에 실제 바닥 504점과 3 cm 높은 평면 1,496점을 주자, 단차 윗면이 유효한 바닥으로 채택되고 윗면에서 검출된 장애물 점은 **0개**였다. 프레임은 바닥 추정 성공으로 처리된다. 기존 단차 시험은 8 cm 윗면을 사용한다. [ground.cpp:35 · 바닥 후보·선택](../src/omm_perception/src/ground.cpp#L35) · [ground.cpp:69 · 최종 유효 판정](../src/omm_perception/src/ground.cpp#L69) · [obstacle_memory_node.cpp:314 · 노드 건강 상태와 높이 필터](../src/omm_perception/src/obstacle_memory_node.cpp#L314) · [test_geometry.cpp:23 · 기존 시험](../src/omm_perception/test/test_geometry.cpp#L23)

**수정 및 완료 기준** — 차체 기준 지면 높이와 그 불확실성, 이전에 확인된 바닥, 더 낮은 평면의 지지를 함께 고려한다. 구분할 근거가 부족한 프레임은 건강한 자유 공간 관측으로 쓰지 않는다. 같은 3 cm 입력의 단차를 바닥으로 확정하지 않고 정상 경사면과 좁게 보이는 실제 바닥도 처리해야 한다.

### 15. P1 · 장애물을 오래 보지 못한 뒤 빈 공간 한 프레임만으로 기억이 지워진다

활성 메모리 밖에서는 카메라 주변의 보관 행만 갱신하고, 돌아온 장애물은 예전에 누적한 자유 공간 판정 횟수를 복원한다. 설치 라이브러리에서 빈 공간 4회 → 카메라 20 m 이동 → 관측 밖 20프레임 → 복귀 순서로 실행했다. 돌아온 횟수는 여전히 4였고 **새 관측 한 번으로 삭제**됐다. 설정값은 자유 공간 5프레임이다. [obstacle_memory_node.cpp:51 · 설정](../src/omm_perception/src/obstacle_memory_node.cpp#L51) · [memory.cpp:231 · 횟수 복원](../src/omm_perception/src/memory.cpp#L231) · [memory.cpp:294 · 삭제 기준·보관 범위](../src/omm_perception/src/memory.cpp#L294)

**수정 및 완료 기준** — 마지막 자유 공간 판정이 최신 처리 프레임과 연속인지 기억한다. 관측이 끊겼으면 다음 유효 관측부터 다시 센다. 재방문 후 1∼4회로는 장애물을 유지하고 연속 5회 및 기존 시간 조건을 만족할 때 삭제해야 한다.

### 16. P2 · 보관소의 가까운 장애물이 활성 메모리로 계속 돌아오지 못한다

재적재 쿼리는 사각형 범위에 먼저 행 개수 제한을 적용하고 C++에서 그중 원 밖의 점을 제외한다. 빈 슬롯 1개, 검색 반경 1 m에서 사각형 모서리의 `(-0.9, -0.9)`와 원 안의 `(0.2, 0)`을 보관한 뒤 10번 갱신해도 활성 0개·보관 2개였다. 해당 위치는 거친 보관 투영에는 남지만 근거리 정밀 표현으로 돌아오지 못한다. [memory.cpp:211 · SQL 제한](../src/omm_perception/src/memory.cpp#L211) · [memory.cpp:224 · 반경 밖 결과 폐기](../src/omm_perception/src/memory.cpp#L224)

**수정 및 완료 기준** — 반경 필터를 행 수 제한보다 먼저 적용하거나, 유효한 슬롯을 채울 때까지 다음 행을 읽는다. 동일 입력에서 첫 갱신에 원 안의 점이 활성 메모리로 돌아와야 한다.

### 17. P2 · 시계 되감기 전 장애물 스냅샷이 새 관측 없이 다시 나타난다

비용지도 레이어는 되감기에서 승인된 점·투영을 비우지만 수신 스냅샷 포인터는 유지한다. 설치 플러그인에 10.00초 관측 한 건을 주고 시계를 9.50초로 되감으면 해당 비용은 0이고 레이어는 비최신이다. 10.05초까지 진행하자 **새 메시지 0건**인데도 이전 장애물이 비용 254로 돌아오고 레이어가 최신이 됐다. [obstacle_memory_layer.cpp:59 · 수신 스냅샷과 되감기 처리](../src/omm_costmap_layers/src/obstacle_memory_layer.cpp#L59) · [obstacle_memory_layer.cpp:79 · 재승인 조건](../src/omm_costmap_layers/src/obstacle_memory_layer.cpp#L79)

**수정 및 완료 기준** — 시계가 되감기면 수신 대기분과 승인분 모두에 새 시간 세대를 적용한다. 시간이 이전 값에 도달해도 새 세대 관측 전까지 최신 상태나 이전 점유가 복원되지 않아야 한다.

### 18. P2 · 3초 지난 동적 TF로 투영한 전역 비용 레이어가 최신으로 표시된다

레이어는 최신 장애물 메시지를 확인한 뒤 마지막 `map ← odom` 변환을 조회하고 변환값의 수치만 검사한다. 설치 플러그인에 10초의 정상 관측과 7초의 마지막 동적 TF를 넣자 `current=true`, 해당 셀 비용 254였다. 지도 보정이나 위치추정이 끊기면 잘못된 위치에 점유를 놓고도 이 레이어는 정상이라고 보고한다. 상위 가드는 별도 TF 감시를 수행한다. [obstacle_memory_layer.cpp:82 · 최신 상태 결정](../src/omm_costmap_layers/src/obstacle_memory_layer.cpp#L82) · [obstacle_memory_layer.cpp:99 · TF 조회·검사](../src/omm_costmap_layers/src/obstacle_memory_layer.cpp#L99)

**수정 및 완료 기준** — 전역 투영에 필요한 동적 TF의 시각을 검증하고, 오래된 동안에는 마지막 유효 투영을 보수적으로 유지하며 비최신을 보고한다. 새 동적 TF로 회복하는 경우와 동일 프레임·정적 TF, 위치추정기가 허용 범위 안에서 미래 시각을 붙이는 경우를 함께 검사한다.

## 지도, 속도 및 다중 콘솔

### 19. P1 · 지도 저장 도중 다른 콘솔이 구동 권한을 취득할 수 있다

지도 서비스는 요청자를 승인한 뒤 공용 정지 서비스를 호출한다. 정지가 완료되면 가드가 소유권을 반납한다. 저장 서비스는 이후 최대 65초를 기다릴 수 있고, 마지막에는 금지 구역의 세션·개정 번호만 대조한다. 실제 지도·제어 서비스 콜백과 임시 지도 파일로 시험했을 때 **A가 저장 중 B는 제어권 취득 성공, A도 지도와 금지 구역 저장 성공**이었다. 지도 서비스의 작업 중 잠금은 일반 주행 서비스에 전달되지 않는다. [omslam_operator_support/session.py:305 · 지도 승인과 정지](../src/omslam_operator_support/omslam_operator_support/session.py#L305) · [omslam_operator_support/session.py:328 · 저장과 최종 확인](../src/omslam_operator_support/omslam_operator_support/session.py#L328) · [operator_control.py:313 · 가드의 소유권 해제](../src/omslam_navigation/omslam_navigation/operator_control.py#L313)

**수정 및 완료 기준** — 가드가 정지 확인부터 파일·금지 구역 확정 또는 실패 정리까지 지도 작업 상태를 유지하고 다른 구동 활성화를 막도록 한다. 정지 요청은 계속 수용한다. 저장을 늦춘 두 콘솔 시험에서 B의 일반 주행은 저장 종료까지 시작할 수 없어야 한다.

### 20. P1 · 다른 콘솔이 지도를 바꾸면 이전 지도의 목적지가 새 지도에서 전송된다

A가 목록에 넣은 목적지는 B가 지도를 교체해도 ‘중단됨’으로 남는다. A가 다시 이동을 누르면 새 지도의 그 숫자 좌표가 자유 공간인지 검사하고 목록 상태를 다시 ‘대기’로 만든다. 다른 지도에서 실제 모델이 이전 목표 `(0.5 m, 0.5 m, 1.0 rad)`를 전송 큐에 넣었다. 같은 자유 셀 좌표도 다른 건물이나 다른 원점에서는 다른 목적지다. [live_model.py:408 · 세션 교체](../src/omslam_operator/omslam_operator/live_model.py#L408) · [live_model.py:561 · 재실행 시 검사](../src/omslam_operator/omslam_operator/live_model.py#L561) · [omslam_operator/model.py:181 · 목록 상태 초기화](../src/omslam_operator/omslam_operator/model.py#L181)

**수정 및 완료 기준** — 목적지 목록을 생성한 지도 세션에 귀속시키고 다른 지도가 열리면 모든 창에서 이전 목록을 무효화한다. 온라인 SLAM이 같은 지도를 갱신할 때는 유지한다. 서로 다른 두 지도에서 같은 숫자 좌표가 모두 자유여도 이전 목적지는 전송되지 않아야 한다.

### 21. P2 · 다른 창이 새 지도의 초기 위치를 맞춰도 지도를 연 창은 이동할 수 없다

A가 저장 지도를 열고 B가 AMCL 초기 위치를 지정한 뒤, A에 새 위치 관측과 `auto_ready=true`를 전달해도 A는 초기 위치를 계속 요구했다. A는 자기 창에서 보낸 초기 위치 서비스의 성공 표시까지 기다리기 때문이다. 기존 시험은 지도를 연 창이 직접 초기화한다. [live_model.py:741 · 지역 상태 갱신](../src/omslam_operator/omslam_operator/live_model.py#L741) · [live_model.py:762 · 잠금 해제 조건](../src/omslam_operator/omslam_operator/live_model.py#L762) · [live_model.py:558 · 이동 차단](../src/omslam_operator/omslam_operator/live_model.py#L558) · [test_live_model.py:201 · 기존 시험](../src/omslam_operator/test/test_live_model.py#L201)

**수정 및 완료 기준** — 현재 지도 세션에 대해 서버가 확인한 초기화 상태와 신선한 위치 관측을 모든 콘솔이 사용하도록 한다. 다른 창이 초기화하면 A는 중복 초기화 없이 새 목표를 지정할 수 있어야 하고, 이전 지도의 지연 관측은 잠금을 풀지 않아야 한다.

### 22. P2 · 지도 저장 중간 실패로 같은 이름의 재시도가 막힌다

지도 저장 명령은 최종 경로에 YAML·PGM을 먼저 쓴 뒤 포즈 그래프를 직렬화한다. 모의 저장 서비스에서 그래프 단계만 실패시켰더니 두 파일이 남았고, 같은 이름으로 재시도하면 기존 파일 검사에서 즉시 실패했다. GUI는 불완전한 묶음에 표식을 두어 로드를 차단하지만 실패한 저장을 복구하지는 않는다. [map_bundle.py:97 · 기존 파일 확인](../src/omslam_mapping/omslam_mapping/map_bundle.py#L97) · [map_bundle.py:131 · 래스터 저장](../src/omslam_mapping/omslam_mapping/map_bundle.py#L131) · [map_bundle.py:141 · 그래프 직렬화](../src/omslam_mapping/omslam_mapping/map_bundle.py#L141) · [omslam_operator_support/session.py:318 · GUI 저장 표식](../src/omslam_operator_support/omslam_operator_support/session.py#L318)

**수정 및 완료 기준** — 이름을 독점 예약하고 임시 위치에서 전 파일을 쓰고 검증한 뒤 완성된 묶음을 공개한다. 실패 시 해당 작업이 만든 임시 상태만 회수한다. 그래프 실패·동일 이름 재시도·동시 저장에서 기존 성공 지도를 보존해야 한다.

### 23. P1 · 거절됐다고 표시한 95 RPM 입력이 나중에 자동 적용된다

GUI는 RPM 입력을 변경 대기 상태로 만든 뒤 제어권 취득을 시도한다. 다른 소유자 때문에 거절돼도 대기 값이 남는다. A가 10 RPM으로 제어 중 B가 95 RPM을 입력하면 B에 ‘다른 조작자가 제어 중입니다’가 표시된다. A가 나중에 해제하고 B가 제어권만 취득하자 실제 모델이 **추가 입력 없이 95 RPM 요청**을 보냈다. [DrivingPage.qml:184 · QML 활성 조건](../src/omslam_operator/qml/design/DrivingPage.qml#L184) · [live_model.py:360 · 소유자 거절](../src/omslam_operator/omslam_operator/live_model.py#L360) · [live_model.py:618 · 대기 값 등록](../src/omslam_operator/omslam_operator/live_model.py#L618) · [live_model.py:632 · 지연 적용](../src/omslam_operator/omslam_operator/live_model.py#L632)

**수정 및 완료 기준** — 입력 가능 상태를 소유권과 연결하고, 승인 가능한 요청만 해당 제어 세대에 보관한다. 거절·만료·세션 변경 시 폐기한다. B의 후속 취득·수동 운전·자동 이동 어느 것도 사용자가 다시 입력하기 전에 RPM을 올려서는 안 된다.

### 24. P2 · 이동 직후 정지를 누르면 전송하지 않은 목표의 결과를 계속 기다린다

GUI는 목표를 작업 스레드의 큐에 넣는 순간 결과 대기 상태가 된다. 작업 스레드가 꺼내기 전에 정지를 누르면 큐가 비워진다. 실제 모델에서 정지 후 `goal_active=false`, 액션 결과 없음, `queue_waiting=true`가 남아 측정 정지 이후에도 목록 편집과 재실행이 잠겼다. [omslam_operator/model.py:188 · 결과 대기 설정](../src/omslam_operator/omslam_operator/model.py#L188) · [omslam_operator/model.py:126 · 정지 시 큐 제거](../src/omslam_operator/omslam_operator/model.py#L126) · [omslam_operator/model.py:266 · 대기 해제 조건](../src/omslam_operator/omslam_operator/model.py#L266)

**수정 및 완료 기준** — 요청별로 대기열 등록·전송 중·서버 수락을 구별한다. 미전송 목표는 로컬 취소로 끝내고, 실제 전송분만 액션 종료와 연계한다. 작업 스레드가 요청을 꺼내기 직전·직후에 정지해도 측정 정지 후 편집이 풀려야 한다.

### 25. P2 · 조이스틱을 누르고 승인 대기 중이면 첫 화면 갱신이 입력을 해제한다

목표 편집 가능 여부가 제어권 전환 중 거짓이 되면 QML이 지도 제스처와 조이스틱을 함께 취소한다. 실제 Qt 오프스크린에서 포인터를 계속 누르고 있어도 첫 갱신 후 `held=false`가 되고 수동 전환 요청이 사라졌다. [DrivingPage.qml:13 · 편집 조건](../src/omslam_operator/qml/design/DrivingPage.qml#L13) · [DrivingPage.qml:49 · 입력 해제](../src/omslam_operator/qml/design/DrivingPage.qml#L49) · [DrivingPage.qml:101 · 상태 변경 처리](../src/omslam_operator/qml/design/DrivingPage.qml#L101)

**수정 및 완료 기준** — 지도 편집 취소와 수동 입력 해제를 구별한다. 포인터를 누르고 있는 동안에만 승인 후 입력을 이어가고, 놓기·포커스 상실·창 숨김·연결 단절 시에는 즉시 해제해야 한다.

### 26. P2 · 다른 콘솔의 전역 경로가 사라지거나 도착 뒤에도 남는다

경로 표시 차단은 자기 창이 보낸 액션의 결과에서 켜지고, 자기 창이 다음 목표를 보낼 때만 풀린다. A의 목표가 끝난 뒤 B가 운전하면 A가 새 경로를 무시했다. 반대로 B를 처음부터 관찰한 창에는 B가 도착한 뒤에도 마지막 경로가 남았다. 가드 상태에는 이미 세션과 현재 목표 ID가 있다. [omslam_operator/worker.py:278 · 종료 시 차단](../src/omslam_operator/omslam_operator/worker.py#L278) · [omslam_operator/worker.py:381 · 표시 필터](../src/omslam_operator/omslam_operator/worker.py#L381) · [omslam_operator/worker.py:587 · 자기 목표의 차단 해제](../src/omslam_operator/omslam_operator/worker.py#L587) · [operator_control.py:355 · 서버 목표 ID](../src/omslam_navigation/omslam_navigation/operator_control.py#L355)

**수정 및 완료 기준** — 서버 세션·목표 전환·종료 상태로 모든 창의 표시 수명을 결정하고, 목표 ID가 없는 `/plan`은 해당 시작 시각과 대조한다. 제어권을 바꿔 양방향으로 시험해 활성 경로는 보이고 도착 후 지연 메시지는 경로를 되살리지 않아야 한다.

## 경로 디버그, 실행기 및 인수 검증

### 27. P2 · 디버그 모드가 플러그인 설정을 못 읽고도 수집 완료로 기록한다

기록기는 매개변수 서비스가 보이면 즉시 한 번 읽고, 시간 초과와 예외도 완료 집합에 넣는다. 사용자 주행 기록에서 계획기 14개 키 중 `GridBased.*`는 플러그인 이름 1개, 제어기 22개 키 중 `FollowPath.*`와 `goal_checker.*`는 0개였다. 요약에는 대상 5개 모두 완료, 대기 0개로 남았다. 이 기록만으로 당시의 도착 허용오차나 DWB 가중치를 확정할 수 없다. [path_debug.py:425 · 수집 시작·시간 초과](../src/omslam_operator_support/omslam_operator_support/path_debug.py#L425) · [path_debug.py:449 · 성공·오류 처리](../src/omslam_operator_support/omslam_operator_support/path_debug.py#L449) · [path_debug.py:467 · 요약](../src/omslam_operator_support/omslam_operator_support/path_debug.py#L467)

**수정 및 완료 기준** — lifecycle 구성이 끝나고 실행 중인 플러그인의 필수 키가 확인된 시점에 완료로 표시한다. 지연과 오류는 재시도하고, 시간 내 미확정 항목은 별도로 남기며 세션 교체 시 갱신한다. 계획기와 제어기를 늦게 기동한 기록에도 실제 XY·yaw·정지 기준 및 주요 제어 설정이 포함돼야 한다.

### 28. P2 · 경로 기록에 DWB 원본 속도가 없어 계획·제어·속도 제한을 구분할 수 없다

launch는 컨트롤러 출력을 `/cmd_vel/nav`로 보낸다. 가드가 이 값을 검증하고 RPM에 맞게 제한한 뒤 `/cmd_vel/requested`로 발행한다. 기록기는 두 번째 토픽부터 구독한다. 사용자 주행 기록에 제한 후 명령은 305개, 최종 명령은 3,013개였고 원본 토픽은 수신·미수신 집계 양쪽에 없었다. 계획과 실제 이동 차이를 조사할 때 원래 요청한 회전량이 유실된다. [launch_support.py:416 · 컨트롤러 리매핑](../src/omslam_navigation/omslam_navigation/launch_support.py#L416) · [navigation_guard.py:374 · 가드의 속도 처리](../src/omslam_navigation/omslam_navigation/navigation_guard.py#L374) · [path_debug.py:298 · 기록 토픽](../src/omslam_operator_support/omslam_operator_support/path_debug.py#L298)

**수정 및 완료 기준** — `/cmd_vel/nav`의 원본 메시지·시각·프레임을 별도로 기록하고 수신 통계에 넣는다. RPM 제한이 걸리는 한 입력에 대해 원본과 제한 직후의 동일한 헤더 시각을 확인하고, 후단 최종 출력도 같은 주행 기록에서 대조할 수 있어야 한다.

### 29. P2 · 코스만 바꾸어 시뮬레이터를 실행하면 로봇이 코스 밖에서 생성된다

`./sim.sh world:=p4_course`와 시뮬레이션을 포함하는 직접 navigation launch는 X 기본값으로 공항 좌표 `-108`을 전달한다. P4 코스의 X 범위는 `[-1, 11]`, 선언된 출발점은 `(0, 0)`이다. RViz 실행기 `nav.sh`와 콘솔 세션은 월드별 좌표를 따로 지정한다. 실행 인자·생성 노드·코스 원본을 대조한 결과다. [sim.launch.py:25 · 생성 노드](../src/omslam_sim/launch/sim.launch.py#L25) · [sim.launch.py:44 · 시뮬레이터 기본값](../src/omslam_sim/launch/sim.launch.py#L44) · [launch_support.py:440 · Nav2 기본값](../src/omslam_navigation/omslam_navigation/launch_support.py#L440) · [p4_course.json:5 · 코스 정의](../tools/p4_course.json#L5) · [nav.sh:134 · RViz 실행기](../nav.sh#L134) · [omslam_operator_support/session.py:497 · 콘솔 세션](../src/omslam_operator_support/omslam_operator_support/session.py#L497)

**수정 및 완료 기준** — 두 launch와 셸 진입점이 같은 월드별 생성 위치를 사용하고 사용자가 명시한 좌표를 우선한다. 좌표 생략 시 공항은 −108 m, P4는 0 m로 해석되는지 모든 진입점을 검사한다.

### 30. P2 · 속도 인수 검사가 정상 100 RPM을 실패시키고 최종 시뮬레이터 명령을 제외한다

`V4-04`는 `/sim/cmd_vel`을 제외한 명령들을 한데 모아 95 RPM 한도를 적용한다. 현 시뮬레이션 사양은 100 RPM이고 베이스 출력은 바로 이 제외된 토픽을 통해 Gazebo로 간다. 실제 설정 반경 0.068 m에서 허용 직진 0.71209433 m/s를 넣으면 정확히 100 RPM이지만 검증식은 실패했다. 반대로 최종 변환에서만 생긴 초과는 이 검사로 발견할 수 없다. [verify_sim_navigation.py:1133 · 인수 검사](../tools/verify_sim_navigation.py#L1133) · [sim_base.yaml:7 · 시뮬레이션 한도](../src/omslam_sim/config/sim_base.yaml#L7) · [sim_base.py:45 · 실제 출력](../src/omslam_sim/omslam_sim/sim_base.py#L45) · [robot_bridge.yaml:1 · Gazebo 연결](../src/omslam_sim/config/robot_bridge.yaml#L1) · [p4_implementation_plan.md:464 · 인수 기준](p4_implementation_plan.md#L464)

**수정 및 완료 기준** — 실제로 로드한 기하·백엔드 한도로 바퀴 RPM을 계산하고, 원본 요청·가드 제한 후·최종 구동 출력을 단계별로 판정한다. 허용 100 RPM 직진과 복합 회전은 통과하고 최종 토픽에만 들어간 초과 명령은 실패해야 한다.

### 31. P2 · 문서의 빌드 명령만으로는 새 작업공간에서 P4 인수 검사를 시작할 수 없다

문서는 일반 `colcon build` 뒤 검증 도구를 실행하게 한다. 두 검증기는 Git에서 제외된 `reports/sim-navigation/latest_build.json`을 요구하고, 통합 인수의 일부 항목은 `latest_validation.json`까지 요구한다. 저장소에는 이 경로·스키마의 기록을 만드는 명령이 없다. GUI 빌드 도구가 만드는 것은 별도의 operator 기록이다. 현재 로컬 파일도 빌드 239개·검증 191개의 소스 지문을 담고 있어 현재 계산한 253개와 모두 불일치했다. [sim_navigation.md:54 · 문서 빌드](sim_navigation.md#L54) · [sim_navigation.md:165 · 검증 진입](sim_navigation.md#L165) · [verify_nav_contracts.py:87 · 빌드 기록 요구](../tools/verify_nav_contracts.py#L87) · [verify_sim_navigation.py:432 · 주행 검증의 기록 검사](../tools/verify_sim_navigation.py#L432) · [verify_sim_navigation.py:1140 · 계약 결과 요구](../tools/verify_sim_navigation.py#L1140)

**수정 및 완료 기준** — 빌드·설치 성공 후 일치하는 지문을 생성하고 계약 결과를 산출하는 지원 명령을 제공한다. 검증기에 해당 경로를 전달해 주행 기동 전 자료를 확인한다. 빈 `reports/`에서 문서 절차만 수행해 인수가 시작되고, 소스·설치본이 달라지면 명확히 실패해야 한다.

### 32. P3 · 이전 GUI 검증기가 현재 콘솔과 다른 QML을 실행한다

운용 앱은 `LiveConsoleModel`, `NativeMap`, `qml/design/Main.qml`을 쓴다. `verify_operator_gui.py`는 `RobotMap`을 등록하고 `qml/Main.qml`을 연다. 소스 패키지에는 구형 파일이 없고 기존 증분 설치 디렉터리에만 남아 있었다. 새 설치에서는 QML 로드가 실패하고, 잔여 파일이 있는 설치에서는 다른 화면을 검사한다. 별도의 `verify_operator_live.py`는 현행 GUI를 대상으로 한다. [verify_operator_gui.py:538 · 구형 검증 진입](../tools/verify_operator_gui.py#L538) · [omslam_operator/app.py:16 · 운용 앱 경로](../src/omslam_operator/omslam_operator/app.py#L16) · [omslam_operator/app.py:84 · 등록 모델과 타입](../src/omslam_operator/omslam_operator/app.py#L84) · [omslam_operator/setup.py:6 · 현행 설치 규칙](../src/omslam_operator/setup.py#L6) · [verify_operator_live.py:697 · 현행 검증의 QML 경로](../tools/verify_operator_live.py#L697)

**수정 및 완료 기준** — 구형 검증기를 현행 진입점에 통합하거나 폐기하고 사용자가 찾는 검증 명령을 일치시킨다. 빈 설치 경로에서 실제 콘솔 화면을 검사하고, QML 로드가 실패할 때도 이미 시작한 관측 프로세스와 작업 스레드를 정리해야 한다.

## 재검증 순서

1. **실물 배치 전**에는 1·3·4번을 같은 모의 직렬 전송 기록으로 검사한다. ARM 대기 중 정지, 대기 중 MCU 오류, RPM 포화는 실제 바퀴를 돌리지 않고도 판정할 수 있다.
2. **자동 주행**은 8·9·10·13·14·15번부터 설치 플러그인에서 재검증한다. 도착·충돌·저속 진행의 경계가 통과하면 같은 설정으로 경로와 실제 이동을 함께 기록하는 통합 주행을 수행한다.
3. **운영 콘솔**은 7·19∼26번을 두 창에서 검사한다. 지도 저장이나 모드 전환을 의도적으로 지연시키면 순서에 따른 실패를 재현할 수 있다.
4. **인수 근거**에는 5·27·28·30·31번을 반영한다. 실행한 설정, 원본과 최종 명령, 연속된 정지 관측이 같은 실행에 포함돼야 현장 증상을 다시 판별할 수 있다.

기존 GUI·지도·기록 관련 테스트 56개를 이번에 재실행해 모두 통과했다. 현재 빌드에 남은 colcon 결과 집계는 63개 통과다. 63개는 기존 결과 파일에서 확인한 수다. 이번 리뷰는 기존 테스트가 포함하지 않은 입력 순서나 경계 조건을 확인했다.
