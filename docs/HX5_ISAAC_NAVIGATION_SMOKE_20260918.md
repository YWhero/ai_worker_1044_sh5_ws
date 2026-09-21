# SH5/HX5 Isaac Navigation 실제 연결 검증

2026-09-18 12:43–12:46 KST에 새 Isaac GUI와 전용 ROS domain `115`에서 검증했다. Cyclo API는 `http://127.0.0.1:7880/api`, 컨테이너는 `ai_worker_1044_hx5_isaac`과 `cyclo_intelligence_1044_hx5_isaac`이다.

실제 PhysX 스캔으로 Mapping을 시작하고 맵을 저장한 다음, Navigation으로 맵을 다시 불러와 초기 위치를 지정했다. `(x=0.15, y=0, yaw=0)` 목표는 **12.8459초 후 SUCCEEDED**로 종료했다. Nav2 `error_code=0`, recovery 횟수는 `0`이었다. 완료 후 검증용 Navigation 서비스를 종료했다.

| 단계 | 결과 |
| --- | --- |
| Mapping 시작 | 성공; 실제 SLAM `/map` 수신 |
| 진단 맵 저장 | `20260918_isaac_smoke.yaml` 및 `.pgm` 저장 성공 |
| Mapping 종료, Navigation 시작 | 성공; 저장된 맵 로드 |
| 초기 위치 `(0,0,0)` 지정 | 성공; subscriber 1개, no-motion update 8회 요청 |
| 목표 `(0.15,0,0)` 실행 | SUCCEEDED; 12.8459초; recovery 0; error code 0 |
| Navigation 종료 | 성공; Task Engine이나 Task를 재시작하지 않음 |

맵은 `simulation/isaac/ai_worker/maps/`에 저장되어 Gazebo와 분리되어 있다. 초기 맵 크기는 `104×113`, 해상도는 `0.05 m`, origin은 `[-1.391,-2.714,0]`이다. 관측 당시 자유 공간 셀은 2,415개, 장애물 셀은 40개였다. 로봇을 이동하며 전체 작업장을 매핑한 결과는 아니다.

종료 후 실제 물리 위치에서 계산된 spawn 기준 odom은 `(0.117643,0.078977)`, yaw는 `0.050352 rad`이었다. 시작 위치에서 약 `0.1417 m` 이동했으므로 실제 바퀴 구동을 포함한 성공이다. 종료 후 속도는 약 `0.00036 m/s`로 정지 상태였다. map→base 위치는 `(0.131191,0.021492)`, yaw는 `0.074685 rad`이었다. 목표와의 map XY 오차는 약 `0.0286 m`, 실제 odom XY 오차는 약 `0.0854 m`로 사용한 공식 Navigation 설정의 `0.10 m` 허용 범위 안이다.

처음 정지 상태에서 만든 맵으로 AMCL을 초기화했을 때 실제 odom은 거의 `(0,0)`이었지만 map→base는 `(0.002760,-0.147889)`, yaw는 `-0.06082 rad`이었다. 짧은 이동 후 오차가 줄었지만, 이 진단은 전체 작업장의 절대 위치 정확도나 긴 Mission 경로를 검증하지 않는다. 실제 Mission waypoint를 만들기 전에는 전체 작업장을 Mapping하고 위치를 확인해야 한다.

API 응답, 목표의 전체 feedback, 전후 실제 위치와 맵 체크섬은 `/tmp/hx5-isaac-navigation-smoke.json`에 보존했다. 별도의 물리 바퀴·관절 궤적·손끝 접촉·로봇/72개 물체 reset 검증은 `/tmp/hx5-isaac-physics-smoke.json`, 관절·네 카메라·스캔 스트림 검증은 `/tmp/hx5-isaac-smoke.json`에 있다. 이 검증 동안 기존 Gazebo 맵·Mission·Navigation 설정을 수정하지 않았다.
