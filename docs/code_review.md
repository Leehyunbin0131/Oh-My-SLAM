# 코드 리뷰 수정 결과와 검증 범위

[README](../README.md) · [통합 설계](architecture.md) · [기술 기준](baseline.md) · [자율주행](sim_navigation.md)

> **참고 (역사적 기록)**: 이 문서는 워크스페이스 패치판 `nav2_smac_planner` 및 42 RPM 프로파일 시점의 코드 리뷰·검증 기록입니다. 이후 공식 ROS 2 Jazzy `nav2_smac_planner`·`navigation_guard` 단독 체인 및 시뮬레이션 3–70 RPM 범위로 단순화되었습니다. 현재 운용 기준은 [통합 설계](architecture.md)와 [시뮬레이션 자율주행](sim_navigation.md)을 따릅니다.

통합 리뷰의 조사 16건과 외부 리뷰 33건에 대해 수정 내역, 설치본 검증 결과, 실물 측정 대상을 번호별로 정리했습니다. Gazebo에서 검증한 주행 상한은 42 RPM입니다. 최고 속도에서 목표를 네 번 취소했을 때 차체 정지를 0.30초 유지하기까지 0.948~0.956초, 이동거리는 0.092~0.095 m였습니다. [항목별 최종 결과](code_review_fix_results.md) · [통합 수정 계획과 인수 기준](code_review_fix_plan.md)

앞선 리뷰에서는 최종 경로 검증, 시계 초기화, 명령 전달 순서, 프로세스 교체, MCU 시각 동기화를 수정했습니다. 당시의 세부 결과는 아래에 보존합니다. [11월 18일 운용 코드 리뷰](code_review_2026-11-18.md)

## 결함별 수정

| 문제 | 적용한 동작 | 구현·회귀 검사 |
| --- | --- | --- |
| 평활화 후 차체 충돌 검사 누락 | 최종 Lattice 경로를 연속 yaw의 채워진 footprint로 검사합니다. 자세 사이도 병진 거리와 외접 반경×회전각의 합이 샘플당 반 셀 이하가 되도록 보간합니다. 실패하면 원본 경로를 검사해 복원하고, 원본도 불안전하면 계획 실패를 반환합니다. 같은 셀의 시작·목표 조기 반환에도 적용합니다. | 당시 워크스페이스 패치본 `src/nav2_smac_planner/include/nav2_smac_planner/path_validation.hpp`, `src/nav2_smac_planner/src/smac_planner_lattice.cpp`, `src/nav2_smac_planner/test/test_path_validation.cpp` |
| 맵 경계 접근 | 좌표 유효 범위를 `0 ≤ value < size`로 제한하고 비유한 좌표와 범위 밖 선형 인덱스도 거부합니다. 평활화 반복의 `worldToMap()` 실패를 확인한 뒤 셀에 접근합니다. | 당시 워크스페이스 패치본 `src/nav2_smac_planner/src/collision_checker.cpp`, `src/nav2_smac_planner/src/smoother.cpp`, `src/nav2_smac_planner/test/test_smoother_bounds.cpp` |
| SLAM↔AMCL 교체와 프로세스 감시 경쟁 | 종료 대상은 신호 전송 전에 소유 목록에서 제거하고, 감시 대상 선택과 `poll()`을 동일한 잠금 안에서 처리합니다. 교체 전 프로세스의 정상 종료를 장애로 보고하지 않으며 실제 새 프로세스 종료는 감지합니다. | [session.py](../src/omslam_operator_support/omslam_operator_support/session.py), [교체 검사](../src/omslam_operator_support/test/test_map_operations.py) |
| 시간 역행 후 정지 관측·TF 잔류 | `stop_samples`, `stopped_since`와 동적 TF 기록을 폐기합니다. GUI Buffer는 노드 시계를 사용하고 가드·GUI 모두 1초 미만 역행도 명시적으로 초기화합니다. 초기화 후 새 관측·제어권·활성화가 필요합니다. | [operator_core.py](../src/omslam_navigation/omslam_navigation/operator_core.py), [navigation_guard.py](../src/omslam_navigation/omslam_navigation/navigation_guard.py), [worker.py](../src/omslam_operator/omslam_operator/worker.py), [복구 검사](../src/omslam_navigation/test/test_review_recovery.py), [GUI 검사](../src/omslam_operator/test/test_cloud_display.py) |
| 출발 대기용 zero가 새 이동 명령 폐기 | 유효한 새 이동 명령이 downstream 응답을 기다리는 동안 주기적 대기 zero를 생략합니다. 실제 정지·만료·차단은 그대로 적용하며 동일 명령을 다시 보내도 정지를 취소할 수 없습니다. ROS/steady 매핑 지터로 `issued_at`이 역행하는 입력은 lease를 연장하지 않고 거부합니다. | [guard_core.py](../src/omslam_navigation/omslam_navigation/guard_core.py), [base core](../src/omslam_base/omslam_base/core.py), [연결 검사](../src/omslam_navigation/test/test_review_recovery.py) |
| MCU 시계 드리프트 누적 | 적분 `dt`는 MCU tick 차이를 유지합니다. 메시지 시각만 2초 EMA와 `dt`의 1% slew 한도로 호스트에 맞춥니다. 보정 전 400 ms 초과 오프셋 점프, 호스트 시각 역행, MCU 리셋·피드백 단절은 계속 거부합니다. | [base core](../src/omslam_base/omslam_base/core.py), [모의 드리프트 검사](../src/omslam_base/test/test_core.py) |

경로 검사는 현재 5 cm 격자에서 샘플 이동량 최대 2.5 cm의 보간 검사이며 연속 공간의 무충돌 증명을 뜻하지 않습니다. 하위 DWB의 검사는 계속 필요합니다. 당시 상류 Nav2 코드 변경은 `src/nav2_smac_planner/provenance.json`과 패치로 추적했습니다.

## 성능·운용 수정

- **비용지도 갱신 범위:** 이전·현재 점군의 투영 영역을 합친 bounds를 사용합니다. rolling origin과 map→odom 재투영, 시간 epoch 변경을 검사했습니다. 오래되거나 비정상인 센서 입력만으로 기존 점유를 free로 만들지 않습니다. GUI의 전체 `costmap_raw` 구독을 위해 `always_send_full_costmap=true`는 유지합니다.
- **Inflation:** 전역·지역 반경을 `0.55 m`로 조정했습니다. padded 외접 반경 약 `0.4965 m`와 5 cm 셀 여유를 포함하며 launch에서 최소값을 검사합니다. 작은 반경의 기존 문제는 빠른 판정 비활성화였으며, 그 자체가 footprint 검사의 누락을 뜻하지는 않습니다.
- **SQLite:** 저장 트랜잭션마다 준비한 upsert 구문을 재사용합니다. 예외 때 statement 정리와 transaction rollback을 수행하며, 대량 보관·재로드 검사를 포함합니다.
- **일반 fault 복구:** 실행 중 `nav.sh`의 일반 fault는 전체 프로세스 종료 대신 정지 상태를 유지하고 명시적 재활성화 방법을 표시합니다. 원인 해소·새 관측·이전 목표 종료 조건을 만족한 뒤 새 목표를 지정해야 합니다. 자동 재출발은 없습니다.
- **ROS 종료:** guard와 sim base는 SIGINT/SIGTERM을 종료 요청으로 기록하고 현재 콜백을 마친 뒤 최종 zero, executor, node, context 순으로 정리합니다. 실행 중 오류를 종료 오류로 숨기지 않습니다.
- **구동 해제 전환:** AUTO→DISABLED 전환 중 readiness가 존재하지 않는 `DISABLED` 그래프 항목을 조회하던 오류를 수정했습니다. 해제 중에는 MANUAL의 관측 요건을 사용하며, 편집 허가는 별도로 구동 해제와 측정 정지를 확인합니다.
- **목표 근처의 원형 전역 경로:** Lattice의 heuristic 기반 탐색 허용값과 DWB의 XY 도착 범위가 달라 가까운 목표에 긴 돌아오기 경로가 생성되던 문제를 수정했습니다. 도착 판정의 위치 범위와 병진 정지 기준을 launch에서 공유하고, 범위 안에서는 원래 목표를 유지한 짧은 경로와 제자리 회전 구간을 충돌 검사합니다. 실제 성공은 방향·측정 정지 확인을 계속 요구합니다. GUI는 목표 종료 시 경로를 지우고 이전 경로의 지연 수신을 무시합니다.
- **Gazebo·속도 문서:** 종료 시 선택한 월드의 control 서비스를 사용하도록 world 인수를 전달합니다. 시뮬레이션과 실물의 속도 프로파일을 분리했습니다. 현재 시뮬레이션 차륜 상한은 42 RPM이며, 기존 실물 제어기 상한 95 RPM에서의 제동거리는 별도 측정 대상입니다.

계산량·구문 준비 횟수 감소를 특정 CPU 개선율이나 장시간 운용 성능으로 환산하지 않습니다.

## 유지한 복구 정책과 사실 정정

큰 `map→odom` 보정(`0.10 m` 또는 `0.10 rad` 이상)은 정지·목표 취소 후 **내비게이션 세션 재초기화**를 요구합니다. `PerceptionHealth.geometry_healthy`는 바닥 평면 추정의 유효성이며 누적 장애물 기억의 map 정합성을 입증하는 값이 아닙니다. 현재 별도의 기억 재검증 절차 없이 이 제한을 풀면 오래된 기억을 신뢰하게 되므로 자동 해제나 같은 세션의 단순 rearm을 추가하지 않았습니다. 일반 sensor fault와 clock 역행의 명시적 복구와 구분합니다.

AMCL의 `update_min_d`·`update_min_a`는 로봇 이동량 기준입니다. `map→odom` 보정량 한계와 직접 같은 값으로 비교할 수 없으므로 이 이유로 AMCL 설정을 임의 변경하지 않았습니다. EKF 횡방향 관측 추가, Python 가드 부하 조정, 바닥 평면 임계값 변경 역시 이번 리뷰에서 결함으로 입증되지 않아 적용하지 않았습니다.

## 검증 결과와 적용 조건

- 의존성 그래프 16개 패키지 빌드 성공, workspace 설치 경로와 소스·설치본 대응을 확인했습니다.
- 종료 처리 보완 후 guard/sim base에 센서 부하를 주는 SIGINT·SIGTERM 4개 조합이 모두 exit 0, 예외 없이 통과했습니다.
- 설치본 회귀 검사 149개(Python 106, 인지·비용지도·sim 32, 계획기 11)와 실제 planner/DWB 플러그인 검사 10항목이 통과했습니다.
- headless `p4_course`에서 8개 lifecycle ACTIVE, 건강한 센서·기억, 운용 smoothing·inflation 설정을 확인했습니다. 0.7 m 목표를 성공(status 4)으로 마치고 측정 정지·구동 해제를 확인했습니다.
- 설치본에서 명령 대기→전달→실제 정지→새 입력, 시간 역행→새 관측→명시적 재활성화, 프로세스 교체 및 실제 장애 감지를 검사했습니다.
- MCU `±2,000 ppm` 각각 모의 1시간 입력으로 timestamp 단조성·적분·급격 시각 오류를 확인했습니다. 실제 보드 발진기·USB 지연·모터 동작 시험을 대신하지 않습니다.
- 계획기 검증은 설치된 병합 운용 설정(`smooth_path=true`)을 읽습니다. 센터 셀만 빈 후미 충돌, 자세 사이 이동·회전 충돌, 범위 경계, 안전한 원본 복원과 둘 다 실패하는 경우를 포함합니다.
- 실제 설치 launch의 lifecycle·센서·설정과 짧은 목표 주행·측정 정지를 시뮬레이션에서 확인했습니다. 기존 전체 코스 결과를 새 설정의 전체 재검증으로 표시하지 않습니다.

로컬 실행 근거는 `reports/code-review-fixes/`에 보관합니다. 실물 보드, 장시간 연속 주행, 큰 map 보정 후 기억 자동 재구축은 이번 검증 범위에 포함하지 않습니다.

[GUI 지도 도구와 RPM 설정](operator_gui.md)의 추가 검증은 `reports/gui-map-tools/completion-report.md`에 기록합니다. 구동 해제 전환 회귀를 포함한 120개 검사와 설치 QML 조작 9개가 통과했습니다. 시뮬레이션에서 37.5 RPM 명령 상한, 금지 구역 밖 정지, 구역 삭제 후 동일 출발·목표 경로 복구를 확인했습니다. 앞선 검사와 중복되는 항목이 있으므로 두 검사 수를 합산하지 않습니다.

목표 근처의 원형 경로 수정은 `reports/near-goal-fix/completion-report.md`에 기록합니다. 회귀 검사 103개와 설치된 계획기·DWB·도착 판정 플러그인의 오프라인 검사 35항목을 통과했습니다. 저장된 지도·시작·목표 재생에서 3.37 m·3.48 m 루프가 각각 8.70 cm·5.81 cm의 마무리 경로로 바뀌었고, 평활화 on/off 모두 원래 목표를 유지했습니다. 방향 오차·잔여 병진 속도·잔여 회전 속도가 있으면 도착 성공을 거부했습니다. 이 검증은 실물·Gazebo 재주행을 포함하지 않습니다.
