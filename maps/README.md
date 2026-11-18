# 지도 보관

실제 지도는 Git에 포함하지 않습니다. `omslam_mapping`의 `save_map` 도구는 출력 prefix에 occupancy map(`.yaml`, `.pgm`), SLAM pose graph(`.posegraph`, `.data`)와 `.metadata.json`을 함께 저장합니다. 기존 저장 파일은 덮어쓰지 않습니다.

metadata는 backend, 소스·의존성 버전, frame 이름과 설정·geometry/calibration 식별 정보를 기록합니다. 시뮬레이션 지도와 실제 공간 지도를 구분하며, 공개가 필요한 예시는 별도로 선정합니다.

실행과 재로딩 절차는 [지도 작성·위치추정 안내](../docs/sim_mapping.md)를 따릅니다. 공항 자산의 `airport_reference` 지도는 기하 기반 참고 자료이며 SLAM으로 생성한 지도와 구분합니다.
