# AI Worker 1044 + HX5 전용 Cyclo Intelligence 워크스페이스

최신 통합 수정·이미지 준비 범위와 비구동 `build` 명령은
`AI_WORKER_1044_SH5_INTEGRATION_20260916.md`에 정리했다.

## 1. 목적과 위치

이 워크스페이스는 일반 SG2 개발 환경과 분리해서, HX5 hand가 장착된
AI Worker 1044에서 사용할 Cyclo Intelligence와 Autonomy Studio를 준비한다.

```text
/home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws/
└── src/cyclo_intelligence
```

기준 소스는 다음 두 계통을 함께 보존한다.

- SH5 실기기 기능: `DanielFH1/cyclo_intelligence`의
  `feature/local-functions-20260914` (`7bcc876`)
- 최신 공식 구조와 Mission Canvas: `ROBOTIS-GIT/cyclo_intelligence`의
  `main` (`e648425`)

작업 브랜치는 `feature/1044-sh5-mission-canvas`다. 기존
`hero_gazebo_ws`, 기존 Cyclo 컨테이너, 기존 `ai_worker` 컨테이너는 수정하거나
중지하지 않는다.

## 2. 격리 규칙

항상 일반 `docker/container.sh` 대신 아래 전용 래퍼를 사용한다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws/src/cyclo_intelligence
./docker/container_1044.sh status
```

전용 래퍼의 기본값은 다음과 같다.

| 항목 | 1044 전용 값 |
|---|---|
| Compose project | `cyclo_intelligence_1044_sh5` |
| Main container | `cyclo_intelligence_1044_sh5` |
| LeRobot | `lerobot_server_1044_sh5` |
| ViTacFormer | `vitacformer_server_1044_sh5` |
| GR00T | `groot_server_1044_sh5` |
| Navigation target | `ai_worker_1044_sh5` |
| UI | `http://localhost:7280` |
| rosbridge | `7290` |
| supervisor API | `7300` |
| 개발용 ROS domain | `104` |

`start`, `start-lerobot`, `start-vitacformer`, `start-groot`는 이 checkout의
커스텀 구현을 이미지에 포함하도록 자동으로 `--build`를 사용한다.
이미지도 `cyclo-1044-sh5/*` 전용 태그를 사용한다. 기존의
`CYCLO_1044_USE_PREBUILT` 우회는 지원하지 않는다. CLI와 UI 관리 서버 모두
같은 project, container, image, domain을 사용하며, UI의 이미지 준비도
전용 소스로 빌드한다. 빌드할 소스는 main 안에 호스트와 동일한 절대 경로로
읽기 전용 연결된다.

일반 `ROS_DOMAIN_ID=73` 같은 기존 셸 설정은 상속하지 않는다.
1044 domain 변경은 `CYCLO_1044_ROS_DOMAIN_ID`로만 명시한다.
컨테이너 셸 진입 후에도 전달된 domain을 유지한다.
이것은 이름·데이터·ROS 도메인 격리이지 보안 샌드박스는 아니다.
공식 구조의 host network, IPC, GPU와 Docker daemon은 공유한다.

## 3. 최초 실행

먼저 원본에 고정된 서브모듈을 준비하고, 부족한 항목을 확인한다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws/src/cyclo_intelligence
./docker/container_1044.sh prepare
./docker/container_1044.sh check
```

`check`는 파일과 Docker 메타데이터만 읽는다. 컨테이너나 로봇을 시작하지
않으며 빠진 항목이 있으면 종료 코드 1을 반환한다. 센서·실제 추론의 성공을
검증하는 명령은 아니다. 현재의 중요한 미해결 항목은 12절에 기록했다.

1044의 실제 ROS domain과 AI Worker 컨테이너 이름을 먼저 확인한다. 실제 장비의
컨테이너 이름이 공식 기본값인 `ai_worker`라면 다음처럼 실행한다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws/src/cyclo_intelligence

export CYCLO_1044_ROS_DOMAIN_ID=<1044의_ROS_DOMAIN_ID>
export CYCLO_1044_NAVIGATION_CONTAINER=ai_worker

./docker/container_1044.sh start
./docker/container_1044.sh status
```

개발 PC에서 기존 `ai_worker`와 완전히 분리해 둘 때는
`CYCLO_1044_NAVIGATION_CONTAINER`를 설정하지 않는다. 이 경우 Mission Canvas는
존재하지 않는 격리 이름 `ai_worker_1044_sh5`만 찾으므로 기존 로봇 컨테이너를
잘못 제어하지 않는다.

UI는 다음 주소로 연다.

```text
http://localhost:7280
```

셸과 로그는 다음 명령을 사용한다.

```bash
./docker/container_1044.sh enter
./docker/container_1044.sh logs
./docker/container_1044.sh status
./docker/container_1044.sh stop
```

`stop`도 1044 전용 Compose project에만 적용된다.

## 4. 로봇 유형과 토픽 계약

Home에서 Robot Type을 `ffw_sh5_rev1`로 선택한다. 주요 입력은 다음과 같다.

- 팔·HX5 관절: `/arm_hand/joint_states`
- head·lift 관절: `/joint_states`
- 왼손 tactile: `/left_hand/finger_pressures`
- 오른손 tactile: `/right_hand/finger_pressures`
- head camera: ZED left/right compressed image
- wrist camera: left/right RealSense compressed image

HX5 정책의 기존 checkpoint·dataset 호환성을 위해 관절 state/action은 다음
54차원을 유지한다. 원본의 `state_mean` 변환처럼 tactile 통계를 state에
추가하는 옵션은 별도이며, 모든 모델의 observation.state가 54차원이라는
뜻은 아니다.

```text
왼팔 7 + 오른팔 7 + 왼손 20 + 오른손 20 = 54
```

head와 lift는 policy 벡터에 추가하지 않았다. 대신 Mission/Action Canvas 전용
`behavior_tree.state`와 `behavior_tree.action`으로 분리했다. 따라서 Canvas에서
head/lift를 사용해도 기존 ViTacFormer, Tactile ACT, HX5 dataset의 차원 계약은
변하지 않는다.

## 5. Mission Canvas

Autonomy Studio의 Mission Canvas는 공식 main 구조를 사용한다.

1. Home에서 `ffw_sh5_rev1`을 선택한다.
2. `Autonomy Studio` → `Mission Canvas`로 들어간다.
3. Mapping에서 AI Worker navigation을 시작한다.
4. Mobile Teleop으로 지도를 작성한다.
5. map을 저장하고 Localization으로 전환한다.
6. waypoint를 만들고 방향을 설정한다.
7. waypoint마다 Action Canvas task를 연결한다.
8. mission route를 저장하고 Run에서 실행한다.

이 워크스페이스에 추가된 기능은 다음과 같다.

- Mobile Teleop 전후진·회전과 좌우 횡이동
- waypoint 화살표 조작과 별도의 정확한 yaw 수치 입력
- Mapping 중 독립된 Upper Body Jog 패널
  - head pitch/yaw
  - head yaw center
  - lift up/down
  - base가 이동 중이면 관절 jog 차단
  - `/robot_description`과 controller 한계의 교집합 적용

map·waypoint·mission 데이터는 이 저장소의 `docker/workspace/navigation` 아래에
저장된다. map 원본은 Mission Canvas가 지정한 1044 navigation 컨테이너의
`ffw_navigation/maps`와 동기화한다.

## 6. Action Canvas와 HX5

Behavior Tree runtime은 `ffw_sg2_rev1`과 `ffw_sh5_rev1`을 지원한다.

### JointControl

SH5에서 다음 그룹을 독립적으로 선택하고 명령할 수 있다.

- head 2축
- 왼팔 7축
- 오른팔 7축
- 왼 HX5 hand 20축
- 오른 HX5 hand 20축
- lift 1축

`enable_arms`와 `enable_hands`가 분리되어 있어 팔 자세는 유지하면서 손만
바꾸거나, 손을 유지하면서 팔만 움직일 수 있다.

### ArmStateGate

`/joint_states`와 `/arm_hand/joint_states`를 함께 읽어 조건을 평가한다.
SH5의 기본 target은 양팔 7축씩이며, 필요하면 finger joint를 선택해 손 자세도
조건에 포함할 수 있다.

현재 ArmStateGate는 joint-space 조건이다. tactile 기반 grasp 성공 판정과
forward kinematics 기반 end-effector 높이 조건은 아직 별도 gate로 구현하지
않았다. 두 조건은 실제 1044의 tactile 범위와 URDF frame을 확인한 후 별도
`TactileGraspGate`와 `EndEffectorHeightGate`로 추가하는 것이 안전하다.

## 7. Capture Current Pose와 Pose Preset

리더 teleoperation을 종료할 필요가 없다.

1. teleoperation으로 follower를 원하는 자세에 둔다.
2. 움직임을 멈춘다.
3. JointControl 또는 ArmStateGate에서 `Capture Current Pose`를 누른다.
4. 필요한 joint 값이 현재 node 입력에 자동 반영됐는지 확인한다.
5. 재사용할 자세는 이름을 입력하고 `Save Pose Preset`으로 저장한다.

캡처 대상은 리더의 명령값이 아니라 follower가 발행한 실제 joint state다.
SH5는 `/joint_states`와 `/arm_hand/joint_states`에서 선택한 관절만 캡처한다.
각 관절이 최근 500ms 안에서 최소 4개 표본과 300ms 이상의 정지 관찰 구간을
가져야 한다. 다른 손가락의 움직임이 팔만 캡처하는 것을 막지 않는다.
끊긴 팔 토픽이 head 토픽 수신으로 최신 상태처럼 처리되지 않도록 관절별
수신 시각을 검사한다. Head/Lift Jog도 해당 그룹의 상태가 있으면 독립 사용한다.

프리셋은 로봇 유형별로 분리된다.

```text
docker/workspace/bt/pose_presets/ffw_sh5_rev1/<name>.json
```

파일에는 `cyclo_joint_pose_v1`, robot type, source topics, capture time, joint
이름별 값이 저장된다. 같은 이름을 덮어쓸 때는 UI에서 overwrite 확인을 거친다.

## 8. 데이터 수집

Record에서 Robot Type을 `ffw_sh5_rev1`로 선택한 뒤 task와 subtask를 설정하고
teleoperation으로 수집한다.

기본 SH5 recording layout은 다음을 포함한다.

- 네 카메라
- 54D 팔·손 joint state/action
- 양손 raw tactile pressure
- TF와 camera info

저장 위치는 이 워크스페이스에 한정된다.

```text
docker/workspace/rosbag2
docker/workspace/dataset
```

다른 Cyclo checkout의 `docker/workspace`와 공유하지 않는다.

## 9. 학습과 추론 백엔드

필요한 backend만 시작한다.

```bash
# ACT, Tactile ACT, Diffusion, SmolVLA, XVLA, Pi0/Pi0.5
./docker/container_1044.sh start-lerobot

# HX5 tactile/vision 전용 기존 구현
./docker/container_1044.sh start-vitacformer

# GR00T N1.7
./docker/container_1044.sh start-groot
```

FastWAM은 전용 빌드 이식분을 추가했다. native T-Rex는 여전히 소스가 필요하므로
12절과 최신 통합 문서를 먼저 확인한다.
T-Rex용 원본 커스텀 LeRobot 소스와 모델별 추가 소스를 확보한 뒤에는
main과 policy container 모두 같은 dependency flavor로 시작한다.

```bash
export CYCLO_LEROBOT_POLICY_FLAVOR=trex
./docker/container_1044.sh start
./docker/container_1044.sh start-lerobot
```

서로 다른 backend 컨테이너는 동시에 준비할 수 있다. 다만 하나의 LeRobot
engine 안에서 여러 checkpoint를 동시에 메모리에 유지하는 구조는 아니므로,
LeRobot model을 바꾸면 LOAD가 다시 필요하다. ViTacFormer의 same-episode manual
model switch, preload, Cycle Home 등 feature branch의 SH5 기능은 보존했다.

Initial Pose Sync는 기본 OFF다. 사용자가 켰을 때만 첫 action으로 기본
5초 동안 동기화한다. 이 경로에도 원본의 first-action/warm-start 최대 차이,
관절 한계, 상태 freshness 검사를 적용한다. 원본 보호값을 확대하지 않았다.
Sync 중 pause/stop의 current-pose hold 동작은 유지한다.

## 10. 1044 실기기 적용 전 확인 사항

다음 항목은 실제 1044에 연결해서 최종 확인해야 한다.

1. 실제 `ROS_DOMAIN_ID`
2. AI Worker Docker container 이름
3. `ffw_sh5_rev1` bringup과 controller가 모두 active인지
4. `/arm_hand/joint_states`, `/joint_states`, tactile, 네 camera topic 이름
5. navigation/SLAM/Nav2 서비스가 AI Worker 컨테이너에 설치됐는지
6. HX5 joint 순서와 54D checkpoint 순서가 일치하는지
7. 손가락·팔 joint limit과 initial-pose sync 속도가 실기기에 안전한지
8. Mission Canvas map 저장 경로에 쓰기 권한이 있는지

실기기 확인 전에는 `CYCLO_1044_NAVIGATION_CONTAINER=ai_worker`를 개발 PC에서
설정하지 않는다. 잘못된 컨테이너를 지정하면 Mission Canvas의 mapping,
localization, map 저장 명령이 그 컨테이너에 전달된다.

## 11. 구현 위치

| 기능 | 주요 위치 |
|---|---|
| SH5 robot/policy/BT schema | `shared/shared/robot_configs/ffw_sh5_rev1_config.yaml` |
| BT robot capability | `shared/shared/robot_configs/schema.py` |
| JointControl | `orchestrator/orchestrator/bt/actions/joint_control.py` |
| ArmStateGate | `orchestrator/orchestrator/bt/actions/arm_state_gate.py` |
| Pose capture UI | `orchestrator/ui/src/hooks/useJointPoseCapture.js` |
| Pose preset API | `docker/supervisor_api/bt_pose_presets.py` |
| Mission Canvas | `orchestrator/ui/src/features/missionCanvas` |
| Initial pose sync | `cyclo_brain/policy/common/runtime/main_runtime/control_loop.py` |
| 1044 isolation wrapper | `docker/container_1044.sh` |

## 12. 2026-09-16 보완 결과와 남은 준비

- LeRobot을 기준 feature가 고정한 `c8ce413`(0.5.2)로 복원했다. Compose와
  supervisor의 기본 이미지 버전도 1.3.2로 일치시켰다. 1044에서는 그 공용
  이미지를 덮어쓰지 않고 `cyclo-1044-sh5/lerobot:sh5-c8ce413-<arch>`로 빌드한다.
- 모든 고정 서브모듈과 중첩 서브모듈 초기화를 완료했다.
- 0.6.1용 `env_eval_freq` 대신 원본 `eval_freq`를 복원했다.
- 새 main에서 유입된 MolmoAct2/VLA-JEPA는 원본 LeRobot에 없으므로 선택 목록에서
  제외했다. 원본 모델 항목은 유지했고 Canvas에 Tactile ACT 선택을 추가했다.
- AMD64 main 이미지 `cyclo-1044-sh5/main:local-amd64` 빌드를 완료했다.
  UI production build, ROS2 6개 패키지 빌드, ROS domain 104 유지와 패키지 검색을
  확인했다. 기존 컨테이너를 중지하거나 새 로봇 제어 서비스를 시작하지 않았다.

### 원본에서 별도로 전달받아야 하는 소스

추가로 사용자가 허용한 Task608 ViTacFormer 공개 소스를 비교하여
Upright H100 로딩을 확장했다. 원본 T-Rex/HX5 소스 확보와는 별개의 보완이며,
방법과 검증 범위는 `VITACFORMER_UPRIGHT_H100_INTEGRATION.md`를 따른다.

기준 feature의 문서/adapter는 native T-Rex와 FastWAM을 사용하지만, 그 feature가
Git으로 고정한 LeRobot `c8ce413`에는 두 정책 패키지가 없다. 공식 main의
`240b4a0`에도 native T-Rex는 없다. 원본 문서 `docs/fastwam-inference.md`에는
LeRobot 커밋은 유지한 채 FastWAM을 로컬 이식했다고 적혀 있으나, 그 변경은
고정 커밋만 clone해서는 재현되지 않는다.

FastWAM은 해당 공식 커밋의 패키지를 parent repo의 `backports/`에 추가하고
이미지 빌드에서만 적용하도록 보완했다. 서브모듈 pin은 유지한다. 단, 담당자의
추가 로컬 수정분과 완전히 동일한지까지 확인한 것은 아니다.

native T-Rex에는 담당자의 **1044 커스텀 LeRobot 저장소/브랜치 또는 수정분을
포함한 소스 사본**이 필요하다. native T-Rex를 legacy adapter로 강제
대체하거나 전체 LeRobot을 최신화해서 손 정책 호환성이 확보됐다고 간주하지 않았다.
T-Rex 전용 Dockerfile의 native import 검사도 삭제하지 않았다.
T-Rex legacy 모델은 `docker/workspace/T-Rex-SH5-0807`의 모델별 소스도 필요하다.

사용자가 전달한
[SH5 데이터셋](https://huggingface.co/datasets/Dongkkka/Task_000608_Upright_Water_Bottle_Hand_Intern_lerobot_v30)은
학습 데이터이며 커스텀 정책 소스가 아니다. 데이터셋 README/loader 검증 기록에는
해당 커스텀 소스 저장소 링크나 commit이 없었다. 모델 checkpoint 역시 별개다.

### 실기기 연결과 한계

- 1044용 AI Worker/HX5 드라이버·navigation 컨테이너와 hand-preset 서비스는
  이 Cyclo 저장소에 포함되어 있지 않다. 실제 사용한 ai_worker 저장소/브랜치와
  실기기 domain을 확인한 뒤 연결해야 한다. 기존 PC의 `ai_worker`를 대신 쓰지 않았다.
- main, ViTacFormer, LeRobot AMD64 전용 이미지를 빌드했다. 정책별 최신 준비 결과는
  통합 문서와 `./docker/container_1044.sh check`에서 확인한다.
  GPU checkpoint 로딩·실기기 주행은 미검증이다.
- Jog의 고정 controller 한계와 URDF 교집합은 유지했다. 범위를 확대하는 변경은
  자동 승인 검토에서 로봇 안전 한계 완화로 차단되었다. 실제 controller 범위와
  별도 승인을 확인하기 전에는 이 제한을 해제하지 않는다.
- tactile grasp 성공/FK 높이 Gate는 여전히 신규 기능 구상이며 이 보완에서
  임의 구현하지 않았다. 일반 mapping/localization을 막는 추가 잠금은 없다.
