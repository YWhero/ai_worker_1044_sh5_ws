# AI Worker 1044 SH5 통합 검토 — 2026-09-16

> 1~5절은 수정 전 검토 기록이다. 이후 보완 사항은 6절과 워크스페이스 가이드
> 12절을 따른다. 최초 검토와 이후 수정·빌드 결과를 구분한다.

> 추가 재검토에서 확인한 타이밍 전달·SAC 목록·Jog 정지 확인과 FastWAM/이미지
> 후속 보완의 최신 상태는 `AI_WORKER_1044_SH5_INTEGRATION_20260916.md`를 따른다.
> 아래의 소스·이미지 미준비 상태는 당시 기록이며 최신 상태와 구분한다.

## 범위와 결론

- 대상: `/home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws/src/cyclo_intelligence`
- 원본: `DanielFH1/cyclo_intelligence`, `feature/local-functions-20260914`, `7bcc876`
- 공식 비교 기준: 로컬에 확보된 `ROBOTIS-GIT/cyclo_intelligence` main `e648425`
- 비교 대상은 커밋된 HEAD뿐 아니라 staged/unstaged 변경을 모두 포함한 실제 파일이다.
- 이번 작업은 검토다. 실행 코드, tactile 설정, 손 제어값, 기존 컨테이너를 변경하지 않았다.

촉각·손 전용 원본 구현의 보존은 확인했다. 그러나 현재 checkout을
"완전 격리됐으며 모든 기능을 즉시 실행할 수 있는 환경"으로 판정할 수는 없다.
Docker 실행 연결, 서브모듈, 일부 새 UI 제약에 아래 보완 사항이 있다.
이전 완료 답변에서 실행 준비와 격리를 확정적으로 설명한 부분은 이 검토 결과로 정정한다.

## 1. 원본 기능 보존

원본 feature commit이 변경한 156개 파일을 비교했다. 110개는 바이트 단위로
동일하고 46개는 통합 과정에서 변경됐다. 이 156개 중 없어진 파일은 없다.
이 수치는 원본의 모든 실행 동작이 동일하다는 보증이 아니라 파일 비교 결과다.

다음 전용 구현은 원본과 동일하다.

| 기능 | 확인 경로 |
|---|---|
| 촉각 RAW UI·구독 | `orchestrator/ui/src/components/TactileHandsPanel.js`, `orchestrator/ui/src/hooks/useTactilePressureSubscription.js` |
| 손 프리셋 선택·편집·3D·프로토콜 | `HandPresetControl.js`, `HandPresetDialog.js`, `HandPreset3DViewer.js`, `handPresetProtocol.js`, `useHandPresetStatus.js` |
| Tactile ACT·baseline 처리 | `cyclo_brain/policy/lerobot/lerobot_engine/tactile_act.py`, `tactile_runtime.py` |
| ViTacFormer·preload/switch·촉각 유지 | `cyclo_brain/policy/vitacformer/` |
| T-Rex adapter와 별도 실행 프로필 | `lerobot_engine/trex.py`, `docker/docker-compose.trex.yml` |
| SH5 hand-act 실행 프로필 | `docker/docker-compose.hand-act.yml` |
| 데이터 변환 | `cyclo_data/cyclo_data/converter/base_converter.py`, `to_lerobot_v21.py`, `to_lerobot_v30.py` |
| 녹화 서비스·MCAP recorder | `cyclo_data/cyclo_data/services/recording_service.py`, `cyclo_data/recorder/rosbag_recorder/src/service_bag_recorder.cpp` |
| 모델 전환 로직 | `orchestrator/orchestrator/internal/communication/model_switch.py` |
| SH5 URDF | `shared/shared/robot_configs/urdf/ffw_sh5_follower.urdf` |

SH5 YAML을 파싱해 비교한 결과 `robot_name`, `urdf_path`, `joystick`,
`observation`, `action`, `recording`은 원본과 동일하다. 추가된 것은
Canvas용 `behavior_tree` 항목이다. 양손 pressure 토픽, HandPressures 타입,
카메라 구성, 팔·손 관절명/순서, 기존 명령 토픽은 유지된다.

관절 state와 action은 양팔 14 + 양손 40 = 54차원이다. 다만 변환 설정의
`state_mean`은 원래부터 tactile 평균을 observation.state에 붙일 수 있다.
따라서 "모든 모델의 observation.state가 항상 54차원"이라는 뜻은 아니다.
이번 통합은 그 tactile 변환 규칙을 바꾸지 않았다.

공용 RobotClient 파일은 수정됐다. 그 안의 `get_tactile*`,
`wait_for_tactile*`, `apply_real_action_safety`, `validate_real_action_contract`
함수는 AST 비교상 동일하다. schema의 tactile/state/action/recording 토픽
조회 함수도 동일하다. 다만 공용 시작·정지 흐름은 Initial Pose Sync 때문에
변경됐으므로 "관련 파일까지 전부 무수정"이라고 표현하면 부정확하다.

## 2. 우선 보완이 필요한 실행 문제

### P1 — LeRobot 이미지 버전 불일치

- `docker/docker-compose.yml:168`: `robotis/lerobot-zenoh:1.4.1-${ARCH}`
- `docker/supervisor_api/app.py:336`: 기본 LeRobot 판정은 여전히 `1.3.2-${ARCH}`
- `_local_backend_image`는 supervisor의 이미지 목록만 조회한다.
- 1.4.1만 있으면 UI는 이미지를 없다고 판단할 수 있다. 1.3.2도 있으면
  정상적인 1.4.1 컨테이너를 `image_mismatch`로 판정하여 재생성할 수 있다.
- T-Rex의 별도 1.3.2 이미지까지 무조건 올릴 문제가 아니다. 기본 LeRobot과
  T-Rex를 구분해서 Compose와 supervisor의 이미지 기준을 일치시켜야 한다.

### P1 — Docker 격리가 CLI에서 UI까지 이어지지 않음

`docker/container_1044.sh`는 전용 project name을 export하지만 main 컨테이너의
environment에는 `CYCLO_COMPOSE_PROJECT_NAME`이 전달되지 않는다.
main 환경으로 Compose config를 다시 해석해 다음 차이를 재현했다.

| 시작 경로 | Compose project |
|---|---|
| 1044 전용 CLI | `cyclo_intelligence_1044_sh5` |
| main 안 supervisor의 backend 시작 | `cyclo_intelligence` |

정책 컨테이너 이름 자체는 전용 이름으로 전달된다. 그러나 Compose project
소유권은 달라서 CLI와 UI의 생성·중지·재생성 대상이 일치하지 않는다.

또한 main/정책 이미지 태그가 모두 기존의 `robotis/...`와 같다.
전용 CLI의 `--build`는 같은 Docker daemon의 공용 태그를 새 이미지로 지정한다.
기존 실행 컨테이너를 즉시 바꾸지는 않지만, 다른 워크스페이스의 다음 생성이나
이미지 일치 판정에 영향을 줄 수 있다. 별도 컨테이너명만으로 완전 격리는 아니다.

`ROS_DOMAIN_ID`도 기존 shell 값을 물려받는다. 예를 들어 기존 작업용 73을
export한 터미널에서는 명시적 1044 override가 없을 때 104가 아니라 73이 된다.
전용 기본값과 의도적인 `CYCLO_1044_ROS_DOMAIN_ID` override를 구분해야 한다.

### P1 — 정책 의존성 서브모듈 미초기화

`git submodule status`의 세 항목이 모두 `-`로 표시됐다.

- `cyclo_brain/policy/lerobot/lerobot`
- `cyclo_brain/sdk/zenoh_ros2_sdk`
- `cyclo_brain/policy/groot/Isaac-GR00T`

LeRobot Dockerfile이 COPY하는 `lerobot/setup.py`와 `src/`가 없으므로
현재 상태의 정책 소스 빌드는 준비가 끝난 상태가 아니다. 빈 SDK bind mount도
추론 프로세스 import를 막는다. 시작 wrapper에는 자동 초기화가 없다.

LeRobot pin은 원본 `c8ce413`에서 공식 main의 `240b4a0`로 바뀌었다.
Tactile ACT adapter 자체가 그대로여도 외부 ACT 클래스 API와의 호환성은
이 pin의 실제 의존성 환경에서 별도로 확인해야 한다. 이번 테스트는 해당
모델의 실제 checkpoint 로딩·GPU 추론 성공까지 증명하지 않는다.

## 3. 안전장치·사용 제약의 출처

| 항목 | 출처 | 실제 적용 범위와 평가 |
|---|---|---|
| Real action 관절 한계·첫 명령 차이·tracking bridge·상태 freshness | 원본 SH5 | 값과 핵심 함수 유지. 이번에 임의로 더 강하게 만든 제한이 아님 |
| ViTacFormer tactile baseline·입출력 계약 검증 | 원본 SH5 | 유지. 센서 데이터가 없을 때의 추론 실패는 원래 조건 |
| Initial Pose Sync | 공식 main 기능을 공용 runtime에 통합 | 기본 OFF. ON이면 첫 모델 자세로 기본 5초 이동; 1~60초 설정 |
| Sync 중 pause/stop의 현재 자세 hold와 재시작 제한 | 위 기능에 추가된 실행 처리 | Sync가 켜지고 hold가 실패한 경우에 적용. 일반 추론에 무조건 5초를 추가하지 않음 |
| Pose Capture 안정성 검사 | hero 커스텀 이식 | 최근 500ms 창, 최소 4표본, 최소 300ms 관찰, 회전 변동 0.02rad·lift 0.005m 이내 |
| Head/Lift Jog와 Mobile Teleop 동시 명령 제한 | hero 커스텀 이식 | 이동 중 jog 차단, jog pending 중 해당 UI의 mobile 입력 차단. LiDAR/SLAM을 정지시키는 제한은 아님 |
| Jog 증분·범위·대기 시간 | hero 커스텀 이식 | 기본 head 0.05rad/lift 0.02m, head 0.5초/lift 0.8초 궤적, 최대 3초 결과 대기 |

### 불편을 유발할 수 있는 과도한 범위

1. `useJointPoseCapture.js:31`의 안정성 검사는 선택한 관절만 검사하지 않는다.
   팔만 캡처하고 싶어도 다른 손가락이 0.02rad 이상 움직이면 캡처가 막힌다.
   팔 고정 + 손가락만 움직이는 입력으로 `moving` 판정을 재현했다.
2. `MappingUpperBodyJogPanel.js:159`는 head 2개와 lift의 상태를 모두 요구한다.
   lift 상태가 빠지면 head도 조절할 수 없으며 반대 경우도 같다.
3. Jog 관절 범위는 실제 controller 설정을 실시간으로 읽는 구조가 아니다.
   고정 숫자와 URDF의 교집합이다. 예를 들어 yaw 고정값 ±0.30rad는
   SH5 URDF ±0.35rad보다 좁다. 실제 1044 controller 범위 확인이 필요하다.

### 과도한 제한과 별도로 발견한 빈틈

- Pose Capture는 합쳐둔 관절값의 개별 수신 시각을 추적하지 않는다.
  `/arm_hand/joint_states`가 끊기고 `/joint_states`만 계속 오면 오래된
  팔·손 값도 최신 표본에 복사되어 `stable`이 될 수 있다. 입력 재현으로 확인했다.
- Initial Pose Sync ON 경로는 기존 `preflight_start`를 건너뛰고 첫 목표에
  관절 범위 검사를 적용한다. 기존 first-action/warm-start 최대 차이 검사는
  그 최초 sync 궤적에 그대로 적용되지 않는다. 상태 freshness 검사는
  `publish_initial_pose_sync`에서 별도로 수행되지만 원본 시작 동작과 동등하지 않다.
- Upper Body Jog는 실제 leader/추론/recording 상태를 직접 조회해 잠그지 않는다.
  주된 차단 조건은 UI busy, 지도 편집, 현재 base 이동, jog pending이다.
  주석이나 이전 구상만으로 모든 명령 충돌 감지가 구현됐다고 보면 안 된다.

## 4. LiDAR·Mapping·Localization·Mission Canvas

`navigation.py`, `navigation_missions.py`, `navigation_spots.py`,
`navigation_grid_cache.py`는 공식 main `e648425`와 바이트 단위로 동일하다.
BT 지원 목록에는 `ffw_sh5_rev1`이 포함되며 SG2만 허용하던 제한은 확장됐다.

| 기능 | 코드 상태 | 실제 실행 전제/제한 |
|---|---|---|
| LiDAR·scan 표시 | `/scan` 소비 경로 유지 | 실제 드라이버/스캔 합성 및 TF는 AI Worker bringup 필요 |
| Mapping·지도 저장 | 공식 API 유지 | 대상 컨테이너의 `ai_worker_navigation`, SLAM, map saver, maps 쓰기 권한 필요 |
| Localization | 공식 AMCL 시작 경로 유지 | 대상 컨테이너·map·TF·scan 필요. 현재 명시적으로 `use_sim_time:=false` |
| Waypoint·yaw·route·task 저장/로드 | 공식 Canvas + 커스텀 유지 | 실제 이동은 Nav2 연결 필요 |
| Route 주행·회피 | Nav2 action 연결 유지 | Nav2 controller/costmap/센서 구성은 Cyclo 외부 AI Worker 쪽 설정 |
| Mobile Teleop 횡이동 | `Twist.linear.y` 발행 | 실제 SH5 base controller가 해당 명령을 수용해야 함 |
| JointControl 팔·손·head·lift | SH5 schema로 확장 | 실제 1044 topic/controller 이름 일치 필요 |
| ArmStateGate | 팔·finger joint 목표 판정 가능 | SG2 gripper 감지 기본 이름은 HX5 관절이 아님. 옵션을 켜면 이름/값 조정 필요 |
| 원본 Hand Preset | UI·프로토콜 유지 | 로봇 측 `/leader/hand_preset/*` 서비스가 있어야 함 |
| tactile grasp/FK 높이 gate | 구현되지 않음 | 앞서 논의했던 추가 기능이며 원본에서 지워진 기능은 아님 |

관측 시 실행 중인 컨테이너는 `hero_cyclo_intelligence`, `ai_worker`,
`cyclo_intelligence`였다. 1044 전용 컨테이너는 실행 중이 아니었고,
새 workspace의 src에는 Cyclo만 있다. 기본 navigation 대상 이름은
`ai_worker_1044_sh5`이므로 현재 바로 mapping/navigation이 되는 상태는 아니다.
별도 AI Worker를 준비하거나 실제 1044의 컨테이너와 ROS domain을 연결해야 한다.

HX5를 장착했다고 LiDAR 사용을 차단하는 코드는 발견되지 않았다.
SH5 URDF의 좌우 LiDAR는 base_link에 고정되어 있다. 다만 Cyclo 소스 검토만으로
실제 센서 데이터, Nav2 주행, map 저장 성공을 확인했다고 말할 수는 없다.
실로봇용 `use_sim_time:=false`는 정상적인 기본값이지만, hero Gazebo 환경을
그대로 재현한 것으로 간주해서는 안 된다.

## 5. 검증 결과와 보완 순서

공식 main의 구조 변경으로 과거 video server의 `/bt/launch`, `/bt/shutdown`,
`/bt/save_tree`, `/bt/node-status`와 ROS 서비스 `/bt/list_trees`는 기존 위치에
남아 있지 않다. 현재 UI는 supervisor 기반 제어·tree API를 사용한다.
기능의 이관과 별개로, 과거 endpoint를 직접 호출하는 외부 스크립트까지
수정 없이 호환된다고 보장할 수는 없다.

- 원본 feature 156개 파일 비교: 110 동일 / 46 변경 / 누락 0.
- SH5 YAML 기존 항목 구조 비교: 모두 동일, `behavior_tree`만 추가.
- tactile·기존 관절 보호·schema 핵심 함수 AST 비교: 동일.
- ViTacFormer 계약·RobotClient real-action·schema 기존 테스트: **91 passed**.
- tactile UI·Hand Preset·프로토콜·모델 전환 UI 기존 테스트: **37 passed**.
- UI 테스트용 소스 사본과 현재 UI source 전체: 차이 없음.
- Pose Capture 판정 문제 2건을 로봇 명령 없이 재현.
- Docker config 해석으로 CLI/UI project 불일치 재현. 빌드/시작은 하지 않음.
- 실제 1044 연결, LiDAR/SLAM/Nav2, 학습/추론 checkpoint 실행: 미실시.

보완 우선순위는 (1) 전용 이미지 태그·project/domain 전달과 backend 버전 정합,
(2) pin된 서브모듈 준비와 tactile 정책 의존성 호환 확인,
(3) 선택 관절 기준 캡처·관절별 freshness·head/lift 독립 활성화,
(4) 실제 1044의 bringup/navigation 연결 검증이다.
원본 tactile·hand 로직과 기존 SH5 보호값은 이 작업 때문에 임의로 제거하거나
완화할 대상이 아니다.

## 6. 후속 보완 — 2026-09-16

### 수정 완료

1. Compose/관리 서버의 LeRobot 기본 버전 1.3.2 정합, 전용 이미지 태그·project·
   컨테이너명·도메인을 CLI에서 UI까지 전달했다. UI의 이미지 준비도 로컬 빌드를
   사용한다. main 안에서도 동일 호스트 소스 경로를 읽을 수 있게 read-only mount를 추가했다.
2. 원본 LeRobot pin `c8ce413`과 `eval_freq`를 복원하고 3개 및 중첩 서브모듈을 초기화했다.
3. 원본에 없는 신규 MolmoAct2/VLA-JEPA 선택 항목을 제외했다. 원본 정책 선택은
   유지하고 Action Canvas에 Tactile ACT 선택을 보완했다.
4. Pose Capture는 선택 관절만 검사하고, 토픽별 수신을 합칠 때 오래된 값의
   타임스탬프가 갱신되지 않게 수정했다. Head/Lift Jog 상태 요구도 분리했다.
5. Initial Pose Sync에도 원본 보호 검사와 첫 시작/재개별 warm-start 범위를
   적용했다. 보호 수치는 변경하지 않았다.
6. Dockerfile의 `.bashrc`가 전달된 ROS_DOMAIN_ID를 30으로 덮어쓰던 문제를
   수정했다. 정책 이미지 6개와 main 2개 Dockerfile 모두 동일하게 적용했다.
7. 읽기 전용 `./docker/container_1044.sh check`를 추가했다. 미준비 사항을
   표시할 뿐 일반 UI 시작이나 로봇 기능에 새로운 전역 잠금을 걸지 않는다.

### 보존 재확인 및 검증

- 손·촉각 UI/adapter/데이터 변환/recording/model switch/SH5 URDF와 ViTacFormer
  실행 코드 등 33개 파일이 기준 `7bcc876`과 바이트 동일하다.
- SH5 YAML의 모든 원본 키·값이 유지된다. RobotClient tactile 및 원본 real-action
  보호 함수 AST도 동일하다. ViTacFormer Dockerfile 변경은 셸 domain 보존뿐이다.
- 전체 UI: 88 suites, 864 tests 통과.
- supervisor + control loop + service handler: 216 tests 통과.
- Docker CLI/Dockerfile/check: 35 tests 통과. CLI/UI Compose 정합: 2 tests 통과.
- 손·촉각·정책 adapter·schema 및 RobotClient 검사: 141 passed, 2 skipped.
- RobotClient Initial Pose Sync: 별도 프로세스에서 9 passed. 합쳐 실행하면
  기존 테스트의 전역 Zenoh stub이 충돌하므로 분리했다.
- 새 main 이미지의 BT 명령·모델 전환·orchestrator sync 회귀: 106 passed.
- 전체 policy/RobotClient 디렉터리를 한꺼번에 실행하는 시도는 rosbag 통합 테스트의
  Zenoh stub 및 engine 의존성 import에서 중단되어 성공으로 집계하지 않았다.
- 새 AMD64 main 이미지의 UI production build와 ROS2 6개 패키지 빌드 완료.
  `--network none`, entrypoint=bash인 일회용 컨테이너에서 domain=104 유지,
  orchestrator/cyclo_data 패키지 설치를 확인했다. 로봇 명령은 전송하지 않았다.

### 남은 사항 — 전 기능 실행 완료로 판정하지 않음

- 기준 저장소가 고정한 LeRobot에는 native T-Rex/FastWAM 로컬 이식 코드가 없다.
  1044에서 사용한 수정 소스 확보가 필요하다. 테스트의 mocked adapter 통과는
  누락된 실제 정책 패키지나 checkpoint 추론 성공을 뜻하지 않는다.
- AI Worker/HX5 드라이버·hand-preset/navigation 컨테이너와 모델 checkpoint가
  필요하다. 기존 Gazebo/AI Worker 컨테이너를 대신 연결하지 않았다.
- UI Jog 범위 확대는 자동 승인 검토에서 차단되어 기존 범위를 유지했다.
- 정책 이미지 빌드, GPU checkpoint 로딩, 실센서·mapping·Nav2 주행은 미실시다.
- 상세 준비 명령과 담당자에게 필요한 정보는 `AI_WORKER_1044_SH5_WORKSPACE.md`
  3절·9절·12절을 따른다.
