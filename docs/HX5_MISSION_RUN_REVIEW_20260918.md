# 2026-09-18 SH5/HX5 Mission Run 검토

검토 대상은 `ai_worker_1044_hx5_sim` / `cyclo_intelligence_1044_hx5_sim`에서 사용자가 실행한 `0914_BGF_test_1`입니다. 저장된 미션 revision 20, Nav2 결과, TaskEngine 서비스 로그를 대조한 결과, **저장된 4단계 경로가 전체 성공했습니다. Navigation 4/4, Waypoint Task 4/4가 완료됐습니다.**

- 지도: `0916_BGF_yw`
- 저장 시각: 2026-09-18 11:55:44 KST (02:55:44 UTC)
- 첫 Navigation 시작: 11:56:21.336 KST
- 마지막 Waypoint Task 완료: 11:57:40.773 KST
- 첫 Navigation 시작부터 마지막 Task 완료까지: **79.437초**
- 실제 저장 경로: **Waypoint 1 → Waypoint 2 → Waypoint 3 → Waypoint 1**

마지막 Waypoint 1 방문은 `global.xml`의 명시적인 `Step_4_Waypoint_1`입니다. 저장된 전역 Sequence는 4단계이며 무한 반복 노드는 없습니다. Canvas에도 `Waypoint 3 → Waypoint 1` 연결이 저장돼 있습니다.

## 단계별 결과

시각은 KST입니다. 이동 시간은 Nav2 `Begin navigating`부터 `Goal succeeded`까지, Task 시간은 `Tree loaded and started`부터 `Behavior Tree completed successfully`까지 계산했습니다.

| 순서 | 목표 지도 좌표 `(x, y, yaw)` | Navigation | 실행한 Waypoint Task | Task 결과 |
|---|---|---|---|---|
| 1. Waypoint 1 | `(1.226379, 0.184284, -0.074912)` | 11:56:21.336 → 28.886, **7.550초 / 성공** | Head `[0.05, 0.0]` → Rotate `+90°` → Wait `5초` | 11:56:29.048 → 41.252, **12.204초 / 성공** |
| 2. Waypoint 2 | `(0.282364, 1.405273, 1.619937)` | 11:56:41.882 → 50.732, **8.850초 / 성공** | Head `[0.01, 0.0]` → Wait `5초` | 11:56:50.850 → 57.568, **6.718초 / 성공** |
| 3. Waypoint 3 | `(-0.313273, -0.087091, -1.015801)` | 11:56:58.318 → 11:57:13.118, **14.800초 / 성공** | Wait `5초` | 11:57:13.267 → 18.281, **5.014초 / 성공** |
| 4. Waypoint 1 | `(1.226379, 0.184284, -0.074912)` | 11:57:18.942 → 28.642, **9.700초 / 성공** | Head `[0.05, 0.0]` → Rotate `+90°` → Wait `5초` | 11:57:28.768 → 40.773, **12.005초 / 성공** |

두 Rotate는 각각 **89.91° / 5.321초**, **89.94° / 5.238초**에 성공했습니다. 두 번 모두 완료 후 mobile zero velocity가 기록돼 있습니다. Head JointControl 세 번은 각각 약 **1.782초 / 1.701초 / 1.701초**에 목표 도달을 보고했습니다. 각 XML의 궤적 duration은 2초입니다.

Waypoint 2 Head 시작 직후 `head_joint1 not found` 경고가 한 번 발생했습니다. 이후 동일 동작이 목표 도달 및 성공을 보고했으며 다음 Wait와 전체 Task도 완료했습니다. 초기 피드백 도착 전의 일시적 경고로 판단되며 이번 실행의 실패 원인은 아닙니다.

Navigation 기동 직후 11:56:02–03에는 AMCL이 요청한 시각이 최신 odom TF보다 2–14ms 앞선다는 extrapolation 경고가 있었습니다. 이후 initial-pose API 응답은 11:56:05에 성공했고 실제 미션 이동은 11:56:21부터 수행됐습니다. 이번 4개 목표에는 timeout/abort/충돌 실패가 기록되지 않았습니다. 초기 경고를 줄이려면 초기화 시의 TF 준비 확인/재시도 조건을 검토할 수 있습니다. Docking 설정 경고도 있었지만 이 미션에는 Docking 동작이 없습니다.

## 마지막 완료와 TaskEngine 종료

`/var/log/bt_node/current`에는 마지막 Task에 대해 다음 순서가 남아 있습니다.

1. 11:57:40.772: 마지막 Wait 완료
2. 11:57:40.773: Sequence의 모든 자식 성공, `Behavior Tree completed successfully`
3. 11:57:40.904: TaskEngine process group 정리 시작
4. 11:57:42.927: 정리 완료 및 `/services/bt_node/stop` HTTP 응답

따라서 TaskEngine이 내려간 것은 마지막 성공 이후의 정리입니다. 실패나 중간 Abort로 판정할 근거가 없습니다. UI의 [useMissionRunner.js](../src/cyclo_intelligence/orchestrator/ui/src/hooks/useMissionRunner.js)는 마지막 Task를 마치면 `done`을 처리하고 `finally`에서 TaskEngine을 release하는 구조입니다. API 스냅샷에서도 TaskEngine은 `down`, Navigation은 `up`이었습니다.

개별 ROS 파일 `python3_1604_1789700180112.log`에는 마지막 Wait 시작까지만 남아 있습니다. 서비스 stdout에는 마지막 Wait와 전체 성공이 모두 보존돼 있으므로, 개별 로그의 누락만으로 실패/중단을 판단하면 안 됩니다.

## 이번 실행에서 검증된 범위

이번 실행은 지도 기준 Navigation, Head JointControl, Rotate, Wait를 검증했습니다. 팔/손/lift 동작과 모델 추론, 촉각 파지 조건은 이번 실행에 포함되지 않았습니다.

- 실행한 JointControl은 `enable_arms=false`, `enable_lift=false`인 head 전용 동작입니다. 저장돼 있는 arm position 값들은 이번 실행에 사용되지 않았습니다.
- Waypoint 1 파일에 `ArmStateGate_1`이 존재하지만, `BehaviorTree ID="__pending__1"`에 분리돼 있습니다. 실행 진입점 `MainTree`의 Sequence에서 참조하지 않으므로 이번 실행에서는 Gate가 실행되지 않았습니다.
- 세 Waypoint의 실행 트리에 `SendCommand`가 없습니다. 이번 미션에서 LOAD/RESUME/STOP/CLEAR inference 동작을 수행했다는 증거도 없습니다. 서비스 로그의 11:31대 ViTac/Gate 성공은 앞선 개발 검증 실행이며, 11:56대 사용자 미션과 구분했습니다.
- Waypoint 2 XML에는 비활성 arm 설정에 SG2 이름 `gripper_l_joint1`, `gripper_r_joint1`이 남아 있습니다. 이번 head 전용 실행에는 영향을 주지 않았습니다. 해당 팔 설정을 활성화하려면 SH5 arm 7개와 HX5 hand 20개를 현재 Capture/편집 기능으로 구성해야 합니다.
- 마지막 WP1 Task는 Navigation 이후 `+90°`를 추가 회전합니다. 따라서 미션 종료 방향은 마지막 Navigation goal yaw 자체와 달라지는 것이 Task의 의도된 동작입니다.

판정은 **Nav2의 지도 프레임 결과와 Task의 피드백/완료 로그**를 기준으로 합니다. 이 검토에서는 절대 월드 방향의 정확도, 실제 파지/운반/붓기 결과를 검증하지 않았습니다. 앞선 진단에서 기존 지도의 AMCL heading 변화가 관측됐으므로, 절대 위치/방향 검증에는 현재 physics odometry로 새 Mapping을 수행하고 localization을 확인하는 작업이 별도로 필요합니다.

브라우저의 `Mission complete` 알림과 React runner 상태를 직접 추출하지는 않았습니다. 전체 runner 상태는 프런트엔드의 React state로 관리되고 저장 미션 API에는 실행 이력 endpoint가 없습니다. 그럼에도 저장된 모든 4단계의 Nav2 성공과 Task 성공이 서버 로그에 대조 가능하게 남아 있으므로, 서버 측 관측 범위에서 전체 완료를 판정할 수 있습니다.

## 보존한 증거

컨테이너 재시작 전 아래 파일에 로그/API/XML을 복사했습니다. 이 검토에서 미션 재실행, 이동 명령, 서비스 stop/restart, 사용자 미션 변경은 수행하지 않았습니다.

- `/tmp/hx5-20260918-mission-review.bt-service.log`: 최종 완료가 포함된 TaskEngine 서비스 stdout
- `/tmp/hx5-20260918-mission-review.bt-node.log`: 개별 TaskEngine ROS 로그
- `/tmp/hx5-20260918-mission-review.bt-launch.log`: TaskEngine launch 로그
- `/tmp/hx5-20260918-mission-review.nav-bt.log`: 4개 Nav2 목표 시작/성공
- `/tmp/hx5-20260918-mission-review.nav-controller.log`: 4개 controller `Reached the goal!`
- `/tmp/hx5-20260918-mission-review.nav-service.log`: Navigation 서비스 stdout
- `/tmp/hx5-20260918-mission-review.cyclo-docker.log`, `.ai-docker.log`, `.cyclo-ros.log`: 컨테이너/Orchestrator 로그
- `/tmp/hx5-20260918-mission-review.api.json`, `.mission.json`: revision 20 미션/상태 API 스냅샷
- `/tmp/hx5-20260918-mission-review.global.xml`, `.locals.waypoint_1.main.xml`, `.locals.waypoint_2.main.xml`, `.locals.waypoint_3.main.xml`: 저장 XML 스냅샷
