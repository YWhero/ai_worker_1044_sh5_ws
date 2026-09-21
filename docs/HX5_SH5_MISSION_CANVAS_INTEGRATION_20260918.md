# SH5/HX5 Gazebo · Mission Canvas · ViTacFormer 연결 안내

작업 위치: `/home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws`

2026-09-18에 수정한 Pose Capture, 양손 제어·종료 조건, 추론 백엔드 연결 및 초기화 CLI를 설명합니다. 기존 [Gazebo·데이터 수집 가이드](HX5_GAZEBO_MISSION_CANVAS_GUIDE.md)와 함께 사용합니다. 아래 설정은 전용 시뮬레이션 프로필이며 실제 1044 follower 장치를 마운트하지 않습니다. 전체 PourWater 성공과 장시간 Mission Route 성공은 이 문서의 구현·단위 테스트만으로 확인된 결과가 아닙니다.

추가 요청에 따라 기본 시작 자세를 `initial_pose=inference`로 변경했습니다. 같은 ViTacFormer의 초기 정렬 목표를 다시 관측해 팔·손의 고정 기준값으로 저장하고, Head를 좌우 중앙·최대 하향, Lift를 최상단으로 설정했습니다. 현재 세션에서는 브라우저를 강력 새로고침하고 SH5 Rev1을 선택한 뒤 필요한 서비스를 켜면 됩니다. 아래 bringup 명령은 전체 종료 후 다시 시작할 때 사용합니다.

수정한 AI Worker·Cyclo 이미지를 적용해 새 기본 자세로 실행 중입니다. ViTacFormer backend는 모델 없는 대기 상태이고 Navigation·Task Engine은 꺼두었습니다. 새 시작 위치에서 미션을 실행하기 전에 지도상의 위치와 방향을 다시 맞춥니다. Head·Lift 설정은 시작값이며 이후 JointControl 블록에 지정된 목표는 정상 적용됩니다.

## 1. 실행 순서와 연결 대상

터미널 1에서 컨테이너와 Gazebo를 실행합니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_sim.sh start
./runtime/hx5_sim.sh gazebo
```

기본 `initial_pose=inference`는 [고정 초기 자세 YAML](../src/hx5_simulation/hx5_simulation/initial_poses/vitacformer_task519_sync.yaml)을 사용합니다. 좌우 팔은 약 `[1.57, 0, 0, -2.8, 0, 0, 0] rad`의 접힌 자세이며 손 40축은 실제 초기 장면의 모델 첫 출력을 사용합니다. Head는 `head_joint1=0.6951 rad`(상하 최대 하향), `head_joint2=0`(좌우 중앙), Lift는 `lift_joint=0.0 m`(최상단)입니다. 관절 축과 한계는 [공식 body Xacro](https://github.com/ROBOTIS-GIT/ai_worker/blob/main/ffw_description/urdf/common/follower/ffw_follower_body.xacro)를 확인했고 생성되는 로봇 모델의 실제 한계와 대조합니다.

과거 초기 정렬 target 원본은 로그에 저장되지 않았습니다. 같은 checkpoint revision을 사용해 이전 `navigation` 시작 장면에서 카메라·관절·촉각 입력을 다시 받아 읽기 전용 추론으로 기준값을 확보했습니다. 저장한 입력의 재추론 결과는 54축 모두 오차 0 rad였습니다. 모델의 손 목표는 입력에 따라 달라지므로 이 YAML은 특정 장면의 고정 기준 자세입니다. 새로운 Head 방향에서 다음 LOAD가 만든 손 목표가 항상 같지는 않으며, LOAD의 초기 정렬은 계속 사용할 수 있습니다. 원본 입력과 receipt는 `simulation/cyclo/pose_references/vitacformer_task519_sync_20260918.inputs.pt`와 같은 이름의 `.json`에 보관합니다.

앞서 주행 검증에 사용한 접힌 팔·열린 손·Head 중앙의 공식 설정은 `initial_pose:=navigation` 옵션으로 계속 사용할 수 있습니다. HX5의 긴 손가락이 conveyor 높이로 돌출되던 기존 SG2 수집 자세는 사용하지 않습니다.

공식 SH5 마지막 준비 단계가 필요한 별도 작업에서는 다음 옵션을 사용할 수 있습니다. 작업 자세는 주변 물체와의 간격을 확인한 뒤 사용합니다.

```bash
./runtime/hx5_sim.sh gazebo initial_pose:=task
```

터미널 2에서 Cyclo ROS를 실행합니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_sim.sh cyclo
```

브라우저에서 `http://localhost:7380/`에 연결하고 **FFW SH5 Rev1 (`ffw_sh5_rev1`)**을 선택합니다. 기존 페이지가 열려 있으면 새 UI 번들이 적용되도록 새로고침합니다.

| 구성 | 이 프로필의 값 |
|---|---|
| ROS domain | `105` |
| Zenoh router | `tcp/127.0.0.1:7455`; `start`가 자동 시작 |
| Gazebo partition | `ai_worker_1044_hx5_sim` |
| AI Worker | `ai_worker_1044_hx5_sim` |
| Cyclo | `cyclo_intelligence_1044_hx5_sim` |
| ViTacFormer | `vitacformer_server_1044_hx5_sim` |
| 지도·미션·데이터 | 이 워크스페이스의 `simulation/cyclo/` |

시뮬레이션 전용 policy Compose는 공식 Cyclo의 `main-runtime`과 `engine-process`를 확장하고, 동일한 domain·router 및 이 워크스페이스의 설정·모델 경로를 사용합니다. 정책 시작에 기존 실기기 프로필의 `docker/container_1044.sh`를 사용하지 않습니다.

현재 소스의 UI·orchestrator를 다시 이미지에 반영하려면 아래 명령을 사용합니다. 준비된 이미지를 매 실행마다 재빌드할 필요는 없습니다. `build`만 실행하면 AI Worker와 Cyclo 둘 다 빌드하며, `build cyclo`는 Cyclo만 빌드합니다. 실행 중인 컨테이너에 새 이미지가 반영되는 시점은 다음 `start`의 재생성 때이므로 진행 중 작업과 녹화를 먼저 정리합니다.

```bash
./runtime/hx5_sim.sh build cyclo
# AI Worker까지 함께 빌드할 때:
./runtime/hx5_sim.sh build
```

Cyclo 전용 이미지 태그는 `cyclo-1044-sh5/main:hx5-sim-amd64`이며, `runtime/hx5_sim/Dockerfile.cyclo`가 현재 UI 및 shared/orchestrator를 빌드합니다.

Navigation 서비스 로더는 Cyclo에만 존재하는 ROS transport overlay를 AI Worker에서 무조건 source하지 않도록 수정했습니다. 이 프로필의 Compose는 수정한 `runtime/hx5_sim/ros_service.sh`를 읽기 전용으로 마운트하며, 새 AI Worker 이미지 빌드에도 같은 파일이 포함됩니다.

## 2. Current Pose가 19축으로 잡히던 원인과 수정

`Captures the 19 selected ...`는 오류 코드가 아니라 선택한 관절 수 안내였습니다. SH5 블록에 SG2 기본값이 남아 있으면 좌우 `gripper_l_joint1`, `gripper_r_joint1`도 선택됩니다. SH5에는 그 두 관절이 없어 Capture가 데이터 도착을 계속 기다리게 됩니다.

현재 UI는 새 블록 생성, 저장 Task 로딩, 로봇 종류 변경 때 그 두 SG2 관절을 SH5 선택에서 제거합니다. 위치 CSV에서도 해당 관절의 위치만 같은 인덱스로 제거하여 팔 목표를 유지합니다. SH5의 팔 8번째 값으로 손가락 목표를 대체하지 않습니다.

| 선택 | Capture 관절 수 |
|---|---:|
| 좌우 팔만 | 14 |
| 좌우 팔 + Head + Lift | 17 |
| 좌우 팔 + 양손 | 54 |
| 좌우 팔 + 양손 + Head + Lift | 57 |

선택한 일부 관절만 캡처할 수도 있습니다. `/arm_hand/joint_states`는 팔 14축과 손 40축을 제공하고, Head/Lift는 `/joint_states` 피드백에서 가져옵니다. 선택한 각 관절의 신선한 샘플이 약 0.5초 구간 동안 안정되어야 버튼이 활성화됩니다. 상태 안내에 `Waiting for 관절명`, `Selected joint is still moving`, `Joint state is stale`가 표시되면 해당 피드백을 먼저 확인합니다.

Pose Capture는 상태 읽기입니다. 팔·손을 정지한 뒤 **Capture Current Pose**를 누르면 선택한 블록의 목표 위치가 갱신됩니다. Capture 또는 Preset Load 뒤 Pose Preset을 저장하여 다른 블록에 다시 적용할 수 있습니다. 로봇 종류를 바꾸면 이전 로봇의 저장 대기 중 snapshot은 초기화됩니다.

## 3. JointControl에 양손 목표 넣기

| 그룹 | 활성화 | 관절 선택 | 위치 값 |
|---|---|---|---|
| Head | `enable_head` | `head_joint1`, `head_joint2` | `head_positions` |
| 좌우 팔 | `enable_arms` | `left_joint_names`, `right_joint_names` | `left_positions`, `right_positions` |
| 양손 | `enable_hands` | `left_hand_joint_names`, `right_hand_joint_names` | `left_hand_positions`, `right_hand_positions` |
| Lift | `enable_lift` | `lift_joint` | `lift_position` |

팔은 `arm_l_joint1..7`, `arm_r_joint1..7`; 손은 `finger_l_joint1..20`, `finger_r_joint1..20`입니다. 팔과 손은 별도 그룹으로 선택합니다. 관절 chip을 추가·제거하면 대응하는 위치 CSV도 함께 정렬됩니다. 회전 관절 값은 rad, Lift는 m입니다. 좌우 손의 대칭 자세에서 값의 부호가 같다는 뜻은 아니므로 실제 원하는 가상 자세를 만든 뒤 Capture하는 편이 정확합니다.

한쪽만 움직이려면 반대쪽 관절 선택과 위치 CSV를 비웁니다. 명시적으로 비운 그룹에는 명령을 발행하지 않습니다. 선택된 관절은 `duration` 동안 목표로 이동하고 신선한 피드백을 확인합니다. `state_max_age_sec`의 기본값은 1초입니다.

LG2 리더의 연속 명령과 JointControl 또는 추론 명령이 겹치지 않도록 실행 전 리더를 종료합니다.

```bash
./runtime/hx5_sim.sh leader-stop
```

## 4. ArmStateGate의 양손·Head·Lift 조건

Gate는 가상 로봇 상태를 읽으며 직접 이동 명령이나 추론 STOP 명령을 발행하지 않습니다. **설정한 모든 조건은 AND로 결합**됩니다. 사용하지 않는 target list는 비우고, 같은 관절을 여러 target list에 중복 선택하지 않습니다. 적어도 하나의 관절·이벤트·접촉 조건이 필요합니다.

### 특정 자세에 도달한 경우

ArmStateGate 편집기는 JointControl과 같은 그룹으로 정리했습니다. SH5의 팔 selector에는 각 7축만 표시하고 손은 별도 20축 selector, Head와 Lift도 독립된 섹션에 표시합니다. 저장 Task의 팔 target에 손·Head·Lift가 섞여 있으면 관절 이름과 위치값의 짝을 유지해 해당 그룹으로 옮깁니다. 이미 별도 그룹에 서로 다른 목표값이 있거나 CSV 개수가 맞지 않으면 값을 보존하고 경고를 표시하므로 실행 전에 하나의 목표로 정리합니다. 손 close/open 이벤트와 촉각 조건은 자세 목표와 별도로 설정합니다.

| 대상 | 관절 list | 목표 위치 CSV |
|---|---|---|
| 왼팔 | `left_target_joints` | `left_target_positions` |
| 오른팔 | `right_target_joints` | `right_target_positions` |
| 왼손 | `left_hand_target_joints` | `left_hand_target_positions` |
| 오른손 | `right_hand_target_joints` | `right_hand_target_positions` |
| Head | `head_target_joints` | `head_target_positions` |
| Lift | `lift_target_joints` | `lift_target_positions` |

손가락 몇 개만 선택해도 됩니다. 사용하려는 list를 선택하고 **Capture Current Pose**를 누르면 그 관절들만 목표로 기록됩니다. 손 target을 선택했다고 빈 팔 list까지 자동으로 조건에 추가하지 않습니다. target과 이벤트가 모두 없는 새 Gate에서 Capture를 사용할 때만 좌우 팔 7축씩을 기본 target으로 기록합니다.

`joint_threshold`는 관절별 목표 오차의 공통 허용값이며 기본값은 `0.01`입니다. rad 관절과 m 단위 Lift를 같이 쓰는 경우 단위를 고려하여 설정합니다. SG2의 단일 gripper open/close 필드는 SH5 UI에서 숨기고 사용하지 않습니다.

### 손이 닫혔다가 다시 열린 경우

왼손은 `detect_left_hand=true`, 오른손은 `detect_right_hand=true`로 활성화합니다.

| 왼손 | 오른손 | 의미 |
|---|---|---|
| `left_hand_event_joints` | `right_hand_event_joints` | 이벤트에서 확인할 손가락 관절 |
| `left_hand_closed_positions` | `right_hand_closed_positions` | 닫힘 기준 위치 |
| `left_hand_open_positions` | `right_hand_open_positions` | 다시 열림 기준 위치 |

활성화한 손을 원하는 닫힘 자세로 만든 뒤 **Capture Closed Hand Pose**, 원하는 열림 자세로 만든 뒤 **Capture Open Hand Pose**를 각각 누릅니다. 이벤트 관절 list가 비어 있으면 Capture는 해당 손 20축을 기본으로 선택합니다. 열림/닫힘 목표는 서로 구분되는 위치 배열이어야 합니다. 관절 선택을 바꾸면 두 목표 CSV가 모두 같은 순서로 갱신됩니다.

Gate가 실행된 뒤 닫힘을 관측하고, 이어서 열림을 관측해야 이벤트 조건이 통과합니다. `hand_threshold` 기본값은 `0.05 rad`입니다. 이벤트를 켠 양손과 다른 target/contact 조건을 모두 설정하면 전부 만족해야 통과합니다.

### 촉각 접촉 또는 접촉 해제

| 필드 | 설정 |
|---|---|
| `detect_left_contact`, `detect_right_contact` | 사용할 손만 `true` |
| `left_contact_sensor_names`, `right_contact_sensor_names` | sensor_name CSV; 빈 값은 해당 손의 5개 센서 |
| `left_contact_condition`, `right_contact_condition` | `contact` 또는 `released` |
| `contact_pressure_threshold` | 센서별 9개 raw pressure 합계 기준; 기본값 `30.0` |
| `contact_min_sensors` | 접촉 조건에서 기준을 만족해야 할 최소 센서 수; 기본값 `1` |

센서 이름은 `finger_l_sensor1..5`, `finger_r_sensor1..5`입니다. 공식 `HandPressures.sensors[].pressure_values[]`를 읽습니다. 선택한 센서의 유효하고 신선한 데이터가 먼저 필요하며, 데이터 누락·중단을 접촉 해제로 간주하지 않습니다. `released`는 선택한 모든 센서가 threshold 아래여야 합니다.

`released`만 설정하면 시작부터 접촉이 없는 손도 조건을 만족합니다. 이전 접촉을 반드시 거친 뒤 해제되기를 기다리려면 Sequence에 **contact Gate → released Gate**를 배치하거나 손 close→open 이벤트와 결합합니다. 이때 추론 STOP은 마지막 Gate 뒤에 둡니다.

**접촉 조건의 성공은 안정적인 파지 성공 인증이 아닙니다.** raw pressure 합계는 N 단위 파지력이나 미끄러짐·물체 유지 판정과 다릅니다. 가상 contact-force 기반 pressure는 실기기의 보정된 촉각 값과 일치하지 않으므로, 무접촉 baseline과 대상 물체의 접촉 범위에 맞춰 threshold와 센서 선택을 조정합니다.

공통 필드 `state_max_age_sec=1.0`은 오래된 관절·촉각 상태를 거부합니다. `hold_sec`는 모든 조건이 연속으로 유지되어야 하는 시간을 지정합니다. `timeout_sec=0`은 대기 제한이 없는 설정이므로 운영 미션에는 예상 동작 시간을 고려한 유한값을 지정합니다.

## 5. 사용자 ViTacFormer 모델 준비

다음 모델 준비 명령은 선택한 PourWater 모델의 고정 revision을 다운로드하고 파일 크기·SHA-256을 확인합니다. 다운로드 완료가 정책의 동작 성공 검증을 의미하지는 않습니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_sim.sh model-prepare
./runtime/hx5_sim.sh policy-start vitacformer
```

| 항목 | 값 |
|---|---|
| 모델 repository | `Dongkkka/Task000519_PourWater_ViTacFormer_H100_LR1e4_B512_Hand_Intern` |
| 고정 revision | `b4f21caa4e6a7a05814958b63d5ced46a9374321` |
| 호스트 모델 폴더 | `simulation/cyclo/model/vitacformer/task519_pour_h100/` |
| **SendCommand policy_path** | **`/workspace/model/vitacformer/task519_pour_h100`** |
| checkpoint | `checkpoints/best_validation.pt` |
| 다운로드 검증 기록 | 모델 폴더의 `download_receipt.json` |
| 전용 backend 이미지 | `cyclo-1044-sh5/model-vitacformer:hx5-sim-amd64` |

`policy_path`에는 호스트의 `/home/...` 경로나 checkpoint 파일명 대신 위의 **컨테이너 모델 폴더**를 넣습니다. GPU/CUDA 호환성, 모델 설정·정규화 통계, 입력 카메라·관절·촉각 layout은 실제 LOAD 응답과 backend 로그로 확인합니다. 실행 중인 Docker 이미지가 소스 mount 및 현재 runtime에 맞아야 합니다.

RTX 5090에서는 기존 CUDA 12.6 기반 이미지를 그대로 사용하면 native `sm_120` 지원이 부족했습니다. 전용 이미지에는 공식 PyTorch `2.7.1` / torchvision `0.22.1` / torchaudio `2.7.1`의 CUDA 12.8 wheel을 사용합니다. PyTorch가 설명하는 [Blackwell·CUDA 12.8 지원](https://pytorch.org/blog/pytorch-2-7/)에 맞춘 이 시뮬레이션 전용 이미지이며, 기준 ViTacFormer 이미지가 준비되어 있으면 다음과 같이 재빌드합니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws/src/cyclo_intelligence/cyclo_brain
docker build -f policy/vitacformer/Dockerfile.hx5_sim.amd64 \
  -t cyclo-1044-sh5/model-vitacformer:hx5-sim-amd64 .
```

기본 `BASE_IMAGE`는 `cyclo-1044-sh5/vitacformer:local-amd64`입니다. 빌드가 끝나면 워크스페이스 루트에서 `policy-start vitacformer`를 사용합니다. 실제 GPU의 weights 로딩·입력 fixture 검증, 실제 Gazebo 입력의 SendCommand preview 및 짧은 실제 명령 발행·STOP 순서는 완료했습니다. HandPressures 등록은 공식 `robotis_interfaces` 메시지 패키지를 backend에 읽기 전용으로 연결하고 offline registry를 준비하여 해결했으며, Jazzy 쪽과 같은 패키지 hash를 확인했습니다. 아래 검증 결과에서 preview와 Gazebo 명령 발행을 구분하며, 둘 다 전체 PourWater 성공 검증은 아닙니다.

ViTacFormer 전용 시뮬레이션 환경은 `CYCLO_SENSOR_HISTORY_MODE=simulation`, `CYCLO_SIM_TACTILE_HISTORY_MAX_AGE_S=0.075`로 Gazebo callback 시간 차이를 허용합니다. 최대 허용값은 0.1초이고, 미래 샘플을 쓰거나 누락된 촉각 데이터를 채우지 않습니다. 하드웨어의 기본 tactile history 허용값인 1/30초와 데이터 유효성 검사는 유지합니다.

```bash
docker logs --tail 100 vitacformer_server_1044_hx5_sim
./runtime/hx5_sim.sh status
```

## 6. LOAD → RESUME → Gate → STOP → 다음 액션

Action Canvas에서 **Sequence**의 자식 노드를 아래 순서대로 연결합니다. Waypoint Task에 연결할 때도 같은 Task 순서를 사용합니다.

1. **SendCommand LOAD**: 모델 준비와 입력 layout 검사를 완료합니다.
2. **SendCommand RESUME**: 추론 동작을 시작합니다.
3. **ArmStateGate**: 원하는 팔·손 자세, 손 close→open 이벤트 또는 contact/released를 관측합니다.
4. **SendCommand STOP**: 추론 명령 발행을 멈춥니다.
5. **JointControl / Rotate / 다음 Task**: 이후 동작을 실행합니다.

LOAD 블록에는 다음 값을 사용합니다.

| LOAD 필드 | 값 |
|---|---|
| `target` | `INFERENCE` |
| `command` | `LOAD` |
| `model` | `vitacformer:vitacformer` |
| `policy_path` | `/workspace/model/vitacformer/task519_pour_h100` |
| `inference_mode` | **`robot`** |
| `initial_pose_sync` | **`true`** — 고정 기준 자세에서 현재 모델 입력의 시작 목표로 정렬할 때 |
| `initial_pose_sync_duration_s` | **`5.0`** — 허용 범위 1–60초 |
| `task_instruction` | 선택한 정책에 맞는 PourWater instruction |

이 Cyclo의 **`inference_mode=simulation`은 추론 preview를 표시하고 로봇 명령 topic에는 발행하지 않는 모드**입니다. Gazebo 로봇을 움직이려면 `robot`을 사용합니다. 여기서 `robot`은 명령 발행 여부를 뜻하며, 실제 발행 대상은 격리된 domain `105`·router `7455`의 Gazebo입니다.

`initial_pose_sync`의 일반 기본값은 `false`이며 LOAD에서만 설정합니다. 위의 `true` 추천은 모델의 시작 자세 정렬을 요청하는 **이 Gazebo 프로필**에 적용합니다. 공식 runtime은 모델의 TaskInfo와 첫 목표에 따라 초기 정렬을 처리하므로, 접힌 Navigation 자세가 모델의 팔·손 시작 자세와 모두 같다고 가정하지 않습니다. 초기 정렬 중에는 SYNCING(phase 4)을 유지하며, 정렬이 끝나 INFERENCING(phase 2)에 도달해야 SendCommand의 해당 단계가 완료됩니다. LOAD는 준비·초기 정렬 후 PAUSED로 마치고, 다음 RESUME는 INFERENCING 도달을 기다린 뒤 Gate로 진행합니다. RESUME의 phase 대기는 최대 60초 정렬을 지원하도록 70초이며 LOAD 대기는 600초입니다. `released` 조건 자체가 과거 접촉을 요구하지 않는다는 점은 별도로 고려합니다.

이 ViTacFormer 시뮬레이션 backend는 `CYCLO_INITIAL_POSE_SYNC_MODE=simulation`, `CYCLO_SIM_INITIAL_POSE_SYNC_HAND_MAX_DELTA_BY_KEY=hand_right=1.55`를 명시합니다. **명시적인 initial sync와 요청 duration 5초 이상일 때만** 오른손 초기 목표 차이를 최대 1.55 rad까지 허용하며, 공식 HX5의 1.57 rad 범위 안에 둡니다. 일반 warm-start·RESUME·추론 tracking 검사와 하드웨어 guard는 유지합니다. 모델 artifact·정규화 통계와 하드웨어 관절 한계를 바꾸는 설정이 아닙니다.

RESUME와 STOP도 `target=INFERENCE`로 두고 각 `command`를 선택합니다. Gate가 성공해도 추론은 자동 중단되지 않으므로 STOP 블록을 반드시 이어 붙입니다. STOP 뒤 다시 같은 모델을 사용하려면 RESUME할 수 있고, 모델을 해제하려면 별도의 SendCommand CLEAR를 실행합니다.

### Gate 실패·취소 뒤 정리

Gate timeout이나 다른 자식 노드 실패는 Sequence를 즉시 실패시키므로 뒤의 STOP은 실행되지 않습니다. Task Engine을 끄는 것만으로 추론 backend가 멈추는 것도 아닙니다. 현재 등록된 control은 Sequence와 Loop이므로 지원하지 않는 Fallback 노드로 자동 정리를 구성하지 않습니다.

1. Mission 또는 실행 중인 Task를 중단합니다.
2. 추론 제어 패널에서 **Stop**을 실행하고 **PAUSED**를 확인합니다. 이미 PAUSED이면 다음 단계로 갑니다.
3. **Clear**를 실행하고 **READY**를 확인한 뒤 다음 액션이나 새 Episode를 시작합니다.

Task로 정리하려면 실행 중인 Task를 먼저 중단하고 아래와 같이 독립적인 **STOP → CLEAR** Task를 실행합니다. 이 XML은 실패한 Sequence의 자동 후처리가 아니라 사용자가 별도로 실행하는 정리 Task입니다. 이미 READY이면 실행할 필요가 없습니다. STOP 자체가 실패하면 이 Sequence의 CLEAR도 실행되지 않으므로 추론 패널의 상태와 backend 로그를 확인합니다.

```xml
<root BTCPP_format="4" main_tree_to_execute="Cleanup">
  <BehaviorTree ID="Cleanup">
    <Sequence name="StopAndClear">
      <SendCommand name="StopInference" target="INFERENCE" command="STOP" />
      <SendCommand name="ClearInference" target="INFERENCE" command="CLEAR" />
    </Sequence>
  </BehaviorTree>
</root>
```

추론 API가 응답하지 않으면 전용 backend를 `./runtime/hx5_sim.sh policy-stop vitacformer`로 종료합니다. 이후 Cyclo의 추론 상태를 다시 정리하여 READY를 확인하고 필요한 backend를 재시작합니다. backend 종료만으로 Cyclo의 저장된 상태가 자동으로 READY가 되었다고 가정하지 않습니다.

## 7. Rotate 속도와 Navigation timeout 확인

Rotate에는 `angle_deg`, `angular_velocity`, `tolerance_deg`, `timeout_sec`를 설정할 수 있습니다. `angular_velocity` 기본값은 공식 Rotate의 `0.6 rad/s`, tolerance 기본값은 `0.1°`입니다.

느린 회전·미완료의 원인은 Swerve controller의 `angular_vel_deadband=0.1 rad/s`였습니다. 공식 Rotate의 비례 제어는 도착 직전 `0.05–0.09 rad/s`를 발행하는데 controller가 이를 0으로 처리하여 남은 각도를 계속 줄이지 못했습니다. 생성되는 **Gazebo 전용 `controllers.yaml`의 linear/angular deadband만 `0.001`로 설정**하여 작은 회전·최종 위치 보정 명령이 통과하도록 했습니다. 실기기 YAML, 기본 회전 속도 및 tolerance는 유지합니다. 기존 Gazebo를 재시작해야 새 controller 설정이 반영됩니다.

```bash
curl -fsS http://localhost:7380/api/navigation/status
curl -fsS http://localhost:7380/api/services/bt_node/status
```

Mission 실패 메시지에는 Navigation backend의 남은 거리·recovery·Nav2 error 등의 진단이 있으면 함께 표시합니다. `Navigation timeout at Waypoint 1`을 시간 제한만 늘려 해결했다고 간주하지 않습니다. Gazebo가 pause 상태인지, `/clock`이 진행하는지, 저장 지도에서 현재 위치와 방향을 맞췄는지, 목표 주변이 점유되거나 경로가 막혔는지 먼저 확인합니다. 기존 저장 지도와 현재 Gazebo의 지도 좌표는 같다고 가정하지 않습니다.

추가 실행 확인에서 회전·주행 후 wheel odometry가 Gazebo의 실제 위치·방향과 약 **0.44 m / 80°** 어긋났습니다. 기존 wheel odometry의 SUCCEEDED 응답만으로 가상 로봇이 물리적으로 목표에 도달했다고 판단할 수 없어, 이 프로필의 `/odom`과 `odom → base_link` TF를 **Gazebo의 실제 모델 pose**로 교체했습니다.

| 이동 상태 | 현재 출처 |
|---|---|
| `/odom`, 유일한 `odom → base_link` TF | 공식 Gazebo OdometryPublisher의 물리 pose를 초기 spawn 기준으로 변환 |
| `/simulation/wheel_odom` | Swerve wheel odometry; 비교·진단용으로 유지 |

이는 [공식 Gazebo OdometryPublisher](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html)를 이용한 시뮬레이션 설정입니다. 초기 spawn의 위치뿐 아니라 방향도 역변환하여 Odometry frame을 맞춥니다. 하드웨어 odometry 구현은 변경하지 않습니다. 새 설정은 Gazebo 재시작 후 반영합니다.

물리 pose 전환 후 저장 Waypoint 1에서 약 **0.26 m를 남기고 정지**하던 원인은 오른손 작은 손가락 끝 `finger_r_link20_collision`과 conveyor belt 사이의 실제 접촉이었습니다. 시작 자세를 공식 SH5의 접힌 Navigation 단계로 변경하여 손을 주행 영역 안쪽으로 올렸습니다. 충돌 판정·지도·목표·Nav2 tolerance를 변경하지 않았습니다.

수정 후 같은 저장 Waypoint 1과 처음 측정한 AMCL map pose로의 복귀는 **Nav2 map frame에서 모두 성공**했습니다. 다만 기존 지도에서 AMCL의 `map → odom` 방향이 주행 중 바뀌어 Gazebo의 절대 시작 방향까지 동일하게 복귀한 것은 아닙니다. 신뢰할 수 있는 절대 방향으로 반복 실행하려면 **접힌 시작 자세와 물리 `/odom` 수정이 반영된 상태에서 새 Gazebo 지도를 Mapping**하고 Localization을 다시 맞춥니다. 기존 지도에 이전 wheel odometry 오차가 포함되었거나 Localization이 모호했을 가능성은 추가 확인이 필요한 추정입니다.

## 8. 데이터 유지하며 시뮬레이션 초기화

HX5 초기화는 라이브 gz_ros2_control entity를 순간적으로 reset하는 방식이 아니라 **현재 Gazebo workcell launch와 자식 프로세스를 종료한 뒤 같은 옵션으로 재시작**합니다. Cyclo 컨테이너와 지도·Task·Preset·녹화 데이터는 유지합니다. 전용 AI Worker 컨테이너 내부의 해당 launch 및 Gazebo server/GUI만 추적하고, 최대 약 12초의 SIGINT → TERM → KILL 정리를 확인한 뒤 재실행합니다.

초기화 전 다음 상태를 만듭니다.

1. 진행 중 녹화를 저장 또는 폐기하여 Recording READY 상태로 만듭니다.
2. Mission과 Mapping/Navigation을 중단하고 **Task Engine을 끕니다**.
3. 추론을 STOP하고 CLEAR하여 모델을 해제합니다.
4. 리더와 정책 backend 컨테이너를 종료합니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_sim.sh leader-stop
./runtime/hx5_sim.sh policy-stop vitacformer
# 다른 backend를 시작했다면 해당 backend도 종료합니다.
./runtime/hx5_sim.sh policy-stop lerobot
./runtime/hx5_sim.sh policy-stop groot

./runtime/hx5_sim.sh reset --check
./runtime/hx5_sim.sh reset
# 이전 launch 옵션과 다른 자세로 초기화할 때:
./runtime/hx5_sim.sh reset --initial-pose inference
```

`reset --check`는 idle 조건만 검사하고 재시작하지 않습니다. 녹화 상태가 확인되지 않거나 Mapping/Navigation·Task Engine·추론·리더가 실행 중이면 초기화를 거부합니다. 이전 Gazebo가 정상 종료되지 않으면 두 번째 시뮬레이터를 시작하지 않습니다.

옵션 없는 `reset`은 이전 launch의 GUI/RViz/자세 옵션을 유지합니다. `--initial-pose inference|navigation|task`는 자세만 덮어쓰고 다른 옵션을 보존합니다. 새로 시작하는 `gazebo`의 기본값은 `inference`입니다.

재시작 로그는 다음에서 확인합니다.

```bash
docker exec ai_worker_1044_hx5_sim tail -n 80 /tmp/hx5-gazebo-reset.log
```

Gazebo `/clock`, controller 및 카메라가 준비되면 저장 지도 Localization을 시작하고 **현재 위치·방향을 다시 지정**합니다. 이후 필요한 backend를 시작하고 LOAD·RESUME 및 Mission Run을 진행합니다. 물체 초기화는 현재 workcell에 정의된 시작 배치를 복구하는 것이며 임의 사용자 월드의 물체 배치를 추정하지 않습니다.

전체 종료는 다음 명령으로 합니다.

```bash
./runtime/hx5_sim.sh stop
```

## 9. 공식 근거와 검증 범위

- [ROBOTIS HX5 소개](https://docs.robotis.com/docs/systems/hx5_d20/introduction/): 한 손 20축, 손가락당 4축, 손끝 촉각 센서.
- [공식 Cyclo SH5 설정](https://github.com/ROBOTIS-GIT/cyclo_intelligence/blob/main/shared/shared/robot_configs/ffw_sh5_rev1_config.yaml): 팔·손·Head·Lift 관절 이름과 HandPressures topic.
- [공식 JointControl](https://github.com/ROBOTIS-GIT/cyclo_intelligence/blob/main/orchestrator/orchestrator/bt/actions/joint_control.py): active robot YAML의 action/state 그룹을 사용하는 제어.
- [공식 ROBOTIS Hand](https://github.com/ROBOTIS-GIT/robotis_hand), [공식 AI Worker](https://github.com/ROBOTIS-GIT/ai_worker): 손 모델·촉각 메시지 연결과 AI Worker 시뮬레이션 기반.
- [1044 기능 기준 Cyclo 브랜치](https://github.com/DanielFH1/cyclo_intelligence/tree/feature/local-functions-20260914), [1044 AI Worker 기준](https://github.com/DanielFH1/ai_worker/tree/main): 이 워크스페이스의 기준 구현.

양손 Gate, contact/released 조건, 모델 다운로드 CLI 및 HX5 relaunch 초기화는 공식 메시지·runtime 계약을 이용한 **이 워크스페이스의 확장**입니다. 공식 upstream에 동일한 파지 성공 Gate나 동일한 초기화 CLI가 배포되어 있다는 뜻은 아닙니다.

2026-09-18에 다음을 확인했습니다.

- 첫 통합의 Pose/Gate/editor 54개, Mission Runner 36개, XML serializer 6개, **UI 테스트 합계 96개 통과**. Mission Runner는 재렌더와 BT RUNNING 5회 동안 같은 미션을 유지하고 Navigation·Task를 중복 요청하지 않는 회귀 검증을 포함합니다. LOAD의 초기 자세 정렬 checkbox·이동 시간 편집, RESUME의 비활성화 및 LOAD 설정의 XML 저장/복원도 확인했습니다.
- 첫 통합의 ESLint를 포함한 production UI build 및 전용 Cyclo 이미지 빌드 성공. 당시 UI HTTP 200 응답과 `main.b8d3329a.js` 제공을 확인했습니다. 추가 Gate 그룹 분리 및 CSV migration 후 **UI 71개 테스트**, 새 production build·AI Worker/Cyclo 이미지 build가 통과했고 현재 배포 UI는 `main.70aab325.js`입니다.
- 실제 Cyclo의 SH5 SetRobotType 서비스 성공. 실제 ROS subscriber에서 **57개 관절 값 모두 유한값**, 상태 최대 age **0.00475초**, 3초 동안 각 topic 약 **295개 메시지** 수신을 확인했습니다. 이는 실시간 상태 경로 확인이며 모든 브라우저 조작 조합의 검증은 아닙니다.
- 실제 SH5 Task Engine에서 **JointControl → ArmStateGate → Wait**가 **0.71초**에 완료되었습니다. JointControl은 왼손 finger 1의 현재 위치를 `duration=0.25`로 지정하고 오른손은 빈 값으로 두었으며, Gate는 양손 target·Head·Lift·양손 tactile released와 `hold_sec=0.2`, Wait는 `0.1`초를 사용했습니다.
- 모델 다운로드의 revision 변경·checksum 오류 거부·기존 checkpoint 보존 및 policy Compose의 전용 transport·장치 mount 제거를 포함한 runtime 테스트 9개 통과.
- RTX 5090 전용 backend 이미지 빌드 및 PyTorch **2.7.1+cu128 / native sm_120** 확인. 사용자 checkpoint의 reference CPU/CUDA 비교에서 action 최대 오차 **2.98×10⁻⁸**, tactile 오차 **0**; weights load **0.702초**, warm inference 약 **14.95 ms**를 확인했습니다. 관련 backend 테스트 **99개 통과·3개 skip**입니다. 이 입력 fixture 비교는 PourWater의 물리적 수행 성공을 의미하지 않습니다.
- 실제 Gazebo 입력으로 **LOAD → RESUME → 양손·Head·Lift·tactile Gate → STOP → Wait → CLEAR**가 **6.21초**에 완료되었고 추론 phase 1·2·3·0 전환을 확인했습니다. 이 실행은 `inference_mode=simulation` preview이며 로봇 명령 topic에는 발행하지 않습니다. 공식 HandPressures 등록·offline 패키지 hash 확인 및 시뮬레이션 전용 tactile history 설정을 포함한 SDK 테스트 62개도 통과했습니다.
- 최종 배포 이미지와 새 공식 SH5 접힌 시작 자세에서 `inference_mode=robot`, `initial_pose_sync=true`, 요청 duration 5초로 **LOAD → RESUME → Wait 1초 → 양손 tactile released Gate(`hold_sec=0.2`) → STOP → Wait 0.1초 → CLEAR**가 **11.48초**에 완료되었습니다. phase 순서는 **LOADING(1) → SYNCING(4) → INFERENCING(2) → PAUSED(3, LOAD 완료) → INFERENCING(2, RESUME) → PAUSED(3, STOP) → READY(0, CLEAR)**였습니다. 같은 실행의 좌우 팔·좌우 손 leader JointTrajectory topic에서 **각각 146개 명령**, 실제 Gazebo 최대 관절 변화 **왼팔 0.649435 rad / 오른팔 0.560837 rad / 양손 각각 1.5 rad**를 측정했습니다. 초기 정렬 완료 후 짧은 모델 명령 발행·팔/손 움직임·Gate 이후 STOP/CLEAR 경로를 확인한 결과이며 전체 PourWater의 성공 검증은 아닙니다.
- 초기 정렬 응답의 기존·신규 문구를 모두 인식하고 SYNCING 완료 전 INFERENCING으로 넘어가지 않는 수정은 BT 테스트 55개와 ROS adapter 테스트 38개에서 통과했습니다.
- 최종 검증 후 실제 `reset` CLI로 복원하고 Gazebo server **1개**, `/odom` publisher **1개**, 시작 위치 오차 **0.01 m 미만**, 공식 접힌 팔·열린 손을 확인했습니다. 다시 받은 **57축 모두 유한값**, 관절 상태 최대 age **0.002초**, Recording READY였으며 Navigation·Task Engine은 정지 상태였습니다. ViTacFormer backend는 모델을 해제한 뒤 재시작해 다음 LOAD를 기다리도록 두었습니다.
- 추가 요청의 새 기본 `inference` 자세로 재시작한 뒤 실제 **57축 모두 유한값**, 고정 목표와의 최대 수치 오차 **1.97×10⁻¹⁰**, 상태 최대 age **0.004초**를 확인했습니다. 실제 Head `[0.6951, 0]`, Lift `0`이고 `base_link → arm_base_link` 높이는 **1.4316 m**였습니다. Head 카메라 forward vector의 z가 **-0.64046**이므로 실제 아래를 향하는 것까지 확인했습니다. 팔7+7·손20+20·Head2·Lift1의 그룹을 모두 사용하는 Gate → Wait가 **0.65초**에 완료됐으며 명령을 발행하거나 관절을 움직이지 않았습니다. 새 기본 자세와 기존 IO/촉각/odometry의 시뮬레이션 회귀 **22개**, reset의 기존 guard와 자세 옵션 override 회귀 **13개**도 통과했습니다.

회전 점검에서는 수정 전 10° 요청이 5.36° 이동 후 작은 속도 명령에도 7초 동안 정지한 상태를 재현했습니다. deadband 수정 직후 wheel odometry 기준 Rotate 완료 응답은 10°에 2.60초, 90°에 5.09초였습니다. 이후 wheel odometry drift가 발견되어 **이 수치와 기존 Waypoint 1의 8.15초 완료 응답만으로 실제 물리 pose 도달을 확인했다고 간주하지 않습니다**.

물리 pose 기반 `/odom`과 공식 SH5 접힌 시작 자세 반영 후 최종 Navigation 결과는 다음과 같습니다. 기존 지도·목표·tolerance 및 충돌 판정은 유지했습니다.

| 실행 | Nav2 결과 | 시간 |
|---|---|---:|
| 시작 방향 조절 | SUCCEEDED, status 4 / error 0 | 1.55초 |
| 저장 Waypoint 1 `(1.702312659, -0.024969526, -0.074912319)` | SUCCEEDED, status 4 / error 0 | 8.99초 |
| 처음 측정한 AMCL map pose `(0.02895, -0.08906, -0.00893)`로 복귀 | SUCCEEDED, status 4 / error 0 | 18.20초 |

복귀 후 실제 Gazebo 위치는 spawn에서 약 **0.109 m** 떨어져 있었지만 실제 방향은 spawn과 **-1.249 rad** 차이 났습니다. AMCL의 map/odom 방향 정렬이 주행 중 변했으므로, 위의 결과는 **같은 map-frame 목표들의 Nav2 완료**를 확인한 것입니다. 정확한 Gazebo 절대 시작 방향 복귀나 모든 미션 성공을 보장하는 결과로 사용하지 않습니다. 최신 설정에서 새 Mapping을 권장합니다.

### 최종 실행 확인의 구분

사용자가 이후 실행한 `0914_BGF_test_1`은 4/4 Navigation과 4/4 Task가 모두 성공했습니다. 저장 경로, 단계별 동작, 79.437초 실행 시간 및 Gate·추론이 이번 실행에 포함되지 않은 이유는 [사용자 미션 검토 보고서](HX5_MISSION_RUN_REVIEW_20260918.md)에 별도로 정리했습니다.

| 확인 대상 | 현재 결과 |
|---|---|
| 57축 상태 수신·JointControl·양손/Head/Lift/tactile Gate | 실제 Cyclo·Gazebo 경로 완료 |
| CUDA 사용자 모델과 LOAD/RESUME/STOP/CLEAR | 실제 Gazebo 입력을 사용한 preview 순서 완료 |
| `inference_mode=robot`의 초기 정렬 완료·모델 명령·팔/손 움직임·Gate 후 STOP/CLEAR | 최종 배포에서 11.48초 순서 완료; SYNCING 포함 phase 전환과 실제 움직임 확인 |
| 같은 저장 Waypoint 1과 초기 AMCL map pose 복귀 | Nav2 map frame에서 완료; 절대 Gazebo 방향 복귀는 미확인 |
| PourWater 전체 수행·성공률, 새 지도 반복 Mission Route | 미검증 |

**남은 실행 확인:** 다른 모델·시작 자세 조합의 동작, 새 지도의 절대 방향 재현성과 전체 Mission Route 완료, 사용자 PourWater의 장시간 연속 추론 동작과 Task 성공률. 상기 일부 경로의 완료 결과를 전체 미션 성공으로 확대하지 않습니다.
