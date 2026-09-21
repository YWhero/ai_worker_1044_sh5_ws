# HX5 양손 Gazebo · Mission Canvas · 데이터 수집

작업 위치: `/home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws`

이 문서는 **새 워크스페이스의 시뮬레이션 프로필**용입니다. 기존 `hero_gazebo_ws`나 실제 1044 로봇의 실행 명령을 섞지 않습니다. Mapping·수동 제어·데이터 수집에는 모델 추론 컨테이너가 필요하지 않습니다. 2026-09-18에 추가한 양손 Pose/Gate, ViTacFormer 연결 및 HX5 초기화 CLI는 [SH5/HX5 Mission Canvas 통합 안내](HX5_SH5_MISSION_CANVAS_INTEGRATION_20260918.md)를 따릅니다.

## 1. 처음 켜기

### 터미널 1 — 컨테이너와 시뮬레이터

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_sim.sh start
./runtime/hx5_sim.sh gazebo
```

`start`가 전용 AI Worker 및 Cyclo 컨테이너를 시작하고 **Zenoh router도 자동 실행**합니다. 별도 router 터미널이나 ROS 환경변수 export는 필요하지 않습니다. `gazebo`는 L자 작업장, HX5 양손이 장착된 AI Worker, RViz를 실행합니다. 창이 필요한 명령은 PC의 그래픽 데스크톱 터미널에서 실행합니다.

최초 `start` 또는 `cyclo`에서는 Cyclo의 ROS 서비스 통신 보완본을 자동 준비합니다. 공식 소스 다운로드 때문에 최초 준비에는 인터넷이 필요하며, 이후 동일 이미지·패치에서는 저장된 빌드를 재사용합니다. 저장 위치는 이 워크스페이스의 `simulation/cyclo/ros_transport/`이며 호스트 ROS나 다른 컨테이너의 설치를 덮어쓰지 않습니다.

### 터미널 2 — Cyclo ROS 실행

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_sim.sh cyclo
```

브라우저에서 **http://localhost:7380/** 을 열고 연결한 뒤 로봇 종류를 **FFW SH5 Rev1 (`ffw_sh5_rev1`)** 으로 선택합니다. 기존 환경의 `7180` 페이지가 아닙니다.

### 터미널 3 — 실제 LG2 리더가 필요한 경우만

먼저 기존 환경의 리더 teleoperation을 종료합니다. 동일 USB 리더를 두 환경에서 동시에 열면 안 됩니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_sim.sh leader
```

이 명령만 실제 LG2 리더 장치를 엽니다. 실제 follower/1044/HX5 장치는 연결하지 않습니다. `/dev/left_leader`, `/dev/right_leader`가 없으면 명령이 중단됩니다. 별도 장치 이름을 쓴다면 실제 리더 장치임을 확인하고 아래 환경변수로 지정합니다.

```bash
export HX5_SIM_LEFT_LEADER_DEVICE=/dev/실제_왼쪽_리더
export HX5_SIM_RIGHT_LEADER_DEVICE=/dev/실제_오른쪽_리더
./runtime/hx5_sim.sh leader
```

리더가 없어도 Mapping의 Mobile Teleop, Head/Lift Jog, Action Canvas의 JointControl 및 손 Preset UI로 가상 로봇을 조작할 수 있습니다.

## 2. Mapping → Localization → Waypoint Task

1. Mission Canvas에서 Mapping을 시작합니다.
2. Mobile Teleop으로 앞뒤·횡이동·회전을 하며 충분한 영역을 스캔하고 지도를 저장합니다.
3. Mapping을 종료하고 저장한 지도를 불러옵니다.
4. Localization에서 지도 위의 현재 로봇 위치와 방향을 지정합니다. **지도 좌표와 Gazebo 월드 좌표는 일반적으로 같지 않습니다.**
5. Waypoint를 만들고 필요하면 yaw 숫자 입력으로 도착 방향을 설정합니다.
6. 각 Waypoint에 Action Canvas Task를 연결하고 Mission Route를 실행합니다.

이 프로필의 Mapping/Nav2/Task Engine은 시뮬레이션 `/clock`을 사용합니다. UI에서 시작하는 Navigation의 대상도 `ai_worker_1044_hx5_sim`으로 고정되어 있습니다. 이전에 통합한 Capture Current Pose, Pose Preset, Head/Lift Jog, 횡이동, yaw 입력 UI는 그대로 사용합니다.

SH5 팔은 **좌우 각각 7관절**, 손은 **좌우 각각 20관절**입니다. SG2의 8번째 gripper 값을 SH5 팔 목표에 그대로 넣지 않습니다. 손가락의 좌우 대칭 자세는 숫자 부호까지 같다는 뜻이 아닙니다. 원하는 실제 시뮬레이션 자세를 만든 뒤 Capture Current Pose로 가져오는 것이 편합니다.

JointControl로 팔·손을 제어할 때에는 리더의 연속 명령과 겹치지 않게 `./runtime/hx5_sim.sh leader-stop`으로 리더 명령을 먼저 종료합니다. **Pose Capture 자체는 읽기 기능이므로 리더를 켠 채 정지한 자세에서 사용해도 됩니다.**

모델을 로딩·재개하는 SendCommand는 추론 백엔드와 checkpoint 준비가 필요합니다. 현재는 `model-prepare`, `policy-start vitacformer` CLI 및 전용 backend 연결이 구현되어 있습니다. ArmStateGate에는 손 20축씩의 target·close→open 이벤트, Head/Lift target 및 선택 센서의 contact/released 조건을 추가했습니다. 접촉 Gate의 성공을 안정적인 파지 인증으로 간주하지 않습니다. 설정과 `LOAD → RESUME → Gate → STOP` 순서는 [통합 안내](HX5_SH5_MISSION_CANVAS_INTEGRATION_20260918.md)를 참조합니다.

## 3. 손 제어와 리더의 차이

- Gazebo 모델은 공식 SH5 URDF에서 참조하는 **HX5 D20 Rev2 양손**을 사용합니다.
- 손 Preset UI의 서비스·상태 메시지 계약은 유지하고, 시뮬레이션 전용 서비스 구현을 연결했습니다.
- 기본 Preset은 `Simulation Open`, `Simulation Power Grasp`, `Simulation Pinch`입니다. 커스텀 curl 편집·저장·이름 변경·삭제·복원을 지원합니다.
- 기본 Preset은 URDF 관절 범위로 만든 **가상 예제 자세**이며, 1044 실기기에서 사용하던 실제 프리셋 수치를 복원한 것이 아닙니다.
- LG2의 팔 7관절은 그대로 전달하고, gripper 1축은 선택한 손 Preset의 열림/닫힘 비율로 변환합니다. **LG2로 손가락 20관절을 각각 독립 조작할 수 있다는 뜻은 아닙니다.**
- `Simulation Open`을 선택하면 리더 트리거를 움직여도 계속 열린 상태입니다. 트리거로 잡으려면 Power/Pinch/사용자 Preset을 먼저 선택합니다.
- 손 관절 20개를 독립 목표로 지정하려면 Action Canvas의 손 JointControl을 사용합니다.

기존 리더 소스의 조이스틱 축 비활성화 설정은 유지됩니다. 베이스·고개·리프트는 해당 UI로 조절하고, 녹화용 버튼은 사용할 수 있습니다.

## 4. 촉각은 어떻게 생성되는가

각 손가락 끝의 **실제 Gazebo 충돌 위치·접촉 힘**을 읽어 손가락당 가상 3×3 격자로 투영합니다. 양손 합계 10손가락 × 9셀 = 90개 값입니다. 손을 닫았다는 관절각만 보고 가짜 접촉을 만드는 방식이 아닙니다.

기존 SH5가 사용하는 메시지 그대로 발행합니다.

| 데이터 | 토픽/형식 |
|---|---|
| 왼손 촉각 | `/left_hand/finger_pressures`, `robotis_interfaces/msg/HandPressures` |
| 오른손 촉각 | `/right_hand/finger_pressures`, 같은 형식 |
| 셀별 힘 근사치(N) | `/simulation/left_hand/taxel_forces_newtons`, 오른손 동명 토픽 |
| 모델 출처·스케일 | `/simulation/tactile_model` |
| 54관절 상태 | `/arm_hand/joint_states` |

각 pressure 값은 0–255이며 기본 스케일은 셀당 8 N에서 포화합니다. contact가 없는 유효 샘플은 0, 물리 엔진/브리지 데이터가 끊기면 해당 손 데이터 발행을 멈춥니다. 정지·일시정지 상태에서 오래된 값을 새 관측처럼 계속 만드는 방식은 피했습니다. 로봇 내부 자기접촉은 작업 물체 접촉에서 제외합니다.

**이것은 실물 HX5 촉각 센서의 정밀 digital twin이 아닙니다.** 실제 taxel의 배치·압력 보정·고무 변형·노이즈·전단력·미끄러짐·통신 지연까지 재현한 모델이 아니며, 접촉 힘의 크기를 격자로 근사합니다. 따라서 데이터 파이프라인과 촉각 조건의 개발·시험에는 쓸 수 있지만, 실기기와 수치가 같거나 학습 정책이 그대로 이전된다고 보장하지 않습니다. 파지 안정성과 물체별 물성도 별도 확인이 필요합니다.

기존 tactile UI의 RAW/baseline 처리와 모델용 입력 계약을 바꾸지 않았습니다. 촉각 baseline을 설정할 때는 **손이 아무 물체에도 닿지 않은 상태**로 합니다.

센서 이름은 실기기 broadcaster와 같은 `finger_l_sensor1..5`, `finger_r_sensor1..5`, 셀 이름은 `Present Pressure 1..9`를 사용합니다. 변환 데이터의 feature 이름도 불필요하게 달라지지 않도록 맞췄습니다.

## 5. 데이터 수집

1. SH5 로봇 종류를 선택하고 카메라 4개와 양손 tactile 상태를 확인합니다.
2. Data Collection에서 **새 Task**를 준비하고 instruction/subtask를 입력합니다. 기존 SG2/실기기 데이터 폴더에 섞어 저장하지 않습니다.
3. 리더 또는 손 Preset/JointControl로 시뮬레이션 동작을 만듭니다.
4. UI 버튼으로 녹화·저장·폐기하거나, LG2 연결 시 기존에 쓰던 **왼쪽 버튼 짧게 누르기 = 시작/구간 저장, 오른쪽 = 폐기**를 사용합니다. 여러 subtask의 진행은 원본 Cyclo의 구간 녹화 규칙을 따릅니다. 긴 누름 이벤트는 중복 실행하지 않습니다.
5. 저장 및 영상 처리 완료를 확인한 다음 초기화/종료합니다.

녹화에는 head 좌우와 wrist 좌우 4개 카메라, 54관절 실제 상태, 팔/손 명령, 양손 촉각이 들어갑니다. 시뮬레이션 프로필은 `/clock`, 촉각 출처 메타데이터, 셀별 힘 토픽도 추가 기록합니다.

`episode_info.json`에는 다음 구분을 자동 추가합니다. 실제 로봇 프로필이나 기존 저장 데이터에는 자동 적용하지 않습니다.

```json
{
  "schema_version": "cyclo_gazebo_mcap",
  "simulation": {
    "tactile_source": "gazebo_contact_force_projected_grid_v1",
    "hardware_calibrated": false
  }
}
```

## 6. 저장 위치와 격리

아래 경로는 모두 새 워크스페이스 기준입니다.

| 용도 | 경로 |
|---|---|
| Gazebo/AI Worker 소스 | `src/ai_worker/` |
| 공식 손 모델·메시지 복사본 | `src/robotis_hand/`, `src/robotis_interfaces/` |
| 시뮬레이션 연결 코드 | `src/hx5_simulation/`, `src/hx5_contact_system/` |
| 지도 | `src/ai_worker/ffw_navigation/maps/` |
| Mission/Navigation 저장 데이터 | `simulation/cyclo/navigation/` |
| Action Canvas XML / 자세 Preset | `simulation/cyclo/bt/trees/`, `simulation/cyclo/bt/pose_presets/` |
| 손 curl Preset | `simulation/ai_worker/hand_presets.json` |
| 수집 데이터 | `simulation/cyclo/rosbag2/Task_*_MCAP/` |

전용 컨테이너는 `ai_worker_1044_hx5_sim`, `cyclo_intelligence_1044_hx5_sim`입니다. 물리 리더를 실행한 경우에만 `lg2_leader_1044_hx5_sim`이 추가됩니다. ROS domain **105**, Zenoh **7455**, 별도 Gazebo partition, UI **7380**을 사용합니다. 실제 1044 프로필(domain 104/UI 7280)과 기존 Gazebo 환경(domain 73/UI 7180)도 구분됩니다.

소스는 새 워크스페이스 안의 독립 복사본이고, 실행 중 기존 hero 경로를 마운트하지 않습니다. 호스트의 `.bashrc`, 전역 alias, ROS 설치, 기존 컨테이너는 변경하지 않습니다. 다만 **GPU·CPU·Docker 데몬·호스트 네트워크는 공유**하므로 자원 성능까지 완전 격리된 VM은 아닙니다. Cyclo의 공식 컨테이너 관리 기능 때문에 Docker socket 접근도 남아 있습니다.

## 7. 종료와 초기화

녹화 중이면 먼저 저장 또는 폐기합니다. 실행 터미널에서 Ctrl+C로 종료한 다음 다음 명령으로 **새 시뮬레이션의 컨테이너만** 정리합니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_sim.sh stop
```

월드·로봇·물체 초기화는 `./runtime/hx5_sim.sh reset --check`로 idle 조건을 확인한 뒤 `./runtime/hx5_sim.sh reset`을 사용합니다. 녹화 저장·폐기, Mapping/Navigation 중단, Task Engine 종료, 추론 STOP/CLEAR와 backend·리더 종료가 필요합니다. 이 명령은 Gazebo workcell과 해당 자식 프로세스를 정리·재실행하며 지도/Task/Preset/녹화 데이터와 Cyclo를 유지합니다. 초기화 후 Localization의 현재 위치·방향을 다시 지정합니다. 상세 조건은 [통합 안내의 초기화 절](HX5_SH5_MISSION_CANVAS_INTEGRATION_20260918.md#8-데이터-유지하며-시뮬레이션-초기화)을 참조합니다. 기존 SG2용 `reset_episode.sh`는 이 환경에서 사용하지 않습니다.

화면 없이 실행할 때만 다음 명령을 사용합니다.

```bash
./runtime/hx5_sim.sh gazebo gui:=false rviz:=false
```

이미지 재빌드는 소스 의존성·C++ 또는 이미지에 포함되는 UI/orchestrator 변경을 반영할 때 필요합니다. 준비된 이미지를 매번 다시 빌드하지 않습니다. 아래 `build`는 AI Worker와 Cyclo를 모두 빌드하며 Cyclo만 반영하려면 `build cyclo`를 사용합니다.

```bash
./runtime/hx5_sim.sh build
```

## 8. 원본과 시뮬레이션의 경계

- 공식 [ROBOTIS Hand Gazebo](https://ai.robotis.com/hands/gazebo_hands.html), [robotis_hand](https://github.com/ROBOTIS-GIT/robotis_hand), [Gazebo contact sensor](https://gazebosim.org/docs/harmonic/sensors/) 구조를 기반으로 합니다.
- 손 모델 복사 원본: `robotis_hand` commit `d4f872795abfe6331a924f9fe12b8b69395303e4`. 메시지 복사 원본: `robotis_interfaces` commit `9231cb1005dc03c14bdbf42f1f9b7114af7d3cfb`.
- AI Worker base image는 `robotis/ai-worker@sha256:8fcd3e5305711c7d6a836f1ca17ce3c9ff82e0e31c9b175cdffb3fc67d5de3c2`로 고정했습니다. 새 시뮬레이션 이미지에 새 워크스페이스 소스를 빌드합니다.
- 원본 SH5의 실제 장치 드라이버·손·촉각 학습/추론 구현은 대체하지 않았습니다. 시뮬레이터에 필요한 카메라/LiDAR/contact 센서와 메시지 어댑터는 별도 패키지입니다.
- 손 Preset 서버와 LG2의 gripper→손 변환, 가상 taxel 모델은 **이번 커스텀 구현**입니다. 공식 로봇 손의 보정된 촉각 시뮬레이터로 오해하지 않습니다.
- 실제 1044 손의 revision/장착 보정·촉각 스케일·구동 소스와 일치하는지는 실기기 통합 시 다시 확인해야 합니다. 이 프로필의 성공을 실기기 실행 검증으로 간주하지 않습니다.

### 손·Lift 관절 한계의 시뮬레이션 보완

이 환경의 DART/velocity 기반 제어에서는 손가락이 정확히 관절 한계(완전 열림 0 rad 포함)에 닿은 뒤 다시 움직이지 않는 현상이 재현됐습니다. 단순 재배치나 초기 자세만 바꾸는 방식은 완전히 펼 때 재발하므로 제거했습니다.

ROS의 `robot_description`, 명령 범위, 공식 손 URDF는 그대로 두고, **Gazebo에 넣는 임시 SDF만** 손가락 hard-stop 양쪽에 0.001 rad(약 0.057도)의 수치 여유를 둡니다. 원래 0 rad 명령으로 완전히 펼 수 있고, 기존 목표 범위 내에서 다시 닫을 수 있습니다. 관절을 매번 강제 순간이동시키거나 접촉을 무시하지 않습니다. 이 보완은 실물 로봇의 가동 범위를 넓히는 설정이 아닙니다. 유사한 엔진 제약은 [Gazebo joint-limit 이슈](https://github.com/gazebosim/gz-sim/issues/1684)에도 보고돼 있습니다.

Lift도 초기값 `-0.5 m`가 하한과 정확히 겹쳐 `-0.48 m` 명령을 정상 수신하고도 멈추는 현상을 확인했습니다. Lift의 Gazebo SDF hard-stop에만 같은 방식으로 **1 mm** 여유를 추가합니다. UI/ROS의 명령 범위 `[-0.5, 0.0] m`와 힘·속도 제한, 실제 로봇 설정은 바꾸지 않습니다. 기존에 켜진 Gazebo에는 모델을 다시 로드해야 반영되므로, 진행 중인 매핑·녹화를 저장한 뒤 재시작합니다.

접촉 데이터는 전용 Contact System 하나가 발행합니다. 접촉이 없는 정상 물리 스텝에도 빈 접촉 메시지를 보내므로, `무접촉=0`과 `시뮬레이터/센서 중단`을 구분합니다.

## 9. 이번에 확인한 범위

### Waypoint Task 서비스 타임아웃 재검토 (2026-09-16)

앞선 Rotate 설정·종료 정리만으로는 간헐적인 `/bt/load_and_run` 타임아웃이 해결되지 않았습니다. Task Engine을 새로 시작한 뒤 catalog 조회는 성공하지만 실제 작업 요청이 콜백에 도달하지 않는 경우를 브라우저와 같은 rosbridge 경로 및 직접 ROS 호출에서 재현했습니다. 콜백 실행기 변경은 효과가 없어 원복했습니다.

설치된 공식 `rmw_zenoh_cpp 0.2.10`의 `rmw_service_server_is_available()`은 ROS 그래프에 서비스가 있는지만 확인합니다. 새 클라이언트의 Zenoh querier가 실제 상대와 매칭됐는지는 확인하지 않아, 그래프 확인 직후 전송한 요청이 유실되는 준비 시점 문제가 재현됐습니다. 새 클라이언트 생성 후 100 ms를 기다린 대조 시험에서는 재현되지 않았지만, 고정 sleep을 최종 해결책으로 쓰지는 않습니다.

`runtime/hx5_sim/rmw_zenoh_readiness.patch`는 그래프 확인 **그리고** 해당 querier의 `get_matching_status()`가 모두 준비됐을 때만 서비스 준비 완료를 반환합니다. 기존 `wait_for_service()`가 이를 기다리므로 API·미션 XML·UI 타임아웃은 그대로입니다. 동작 요청을 자동 재전송하지 않으며, 공식 버전 업그레이드나 다른 RMW 전환도 하지 않습니다. 공식 0.2.10 commit `41f316772ba1210a79e715d61951ddf50857b640`을 SHA-256 검증 후 빌드하는 이 워크스페이스 전용 패치입니다. 공식 배포판 자체에 이미 수정됐다는 뜻은 아닙니다.

보완 라이브러리는 `hx5_sim.sh cyclo`와 이 프로필의 s6 ROS 서비스에서만 overlay로 사용합니다. `/opt/ros/jazzy`, 원본 Task Engine/SendCommand, 손·촉각 코드, 학습·추론 버전, 기존 환경은 변경하지 않습니다. 직접 `docker exec`로 ROS를 실행할 때 같은 보완을 쓰려면 기존 ROS setup 뒤에 `source /workspace/ros_transport/install/local_setup.bash`를 추가합니다. 공식 구현 근거: [rmw_zenoh 0.2.10 소스](https://github.com/ros2/rmw_zenoh/tree/41f316772ba1210a79e715d61951ddf50857b640/rmw_zenoh_cpp).

재시작 검증에서 추가로 `behavior_tree.state.mobile.joint_names: []`가 ROS 파라미터의 형식 미지정 배열로 해석돼 Cyclo 본체가 시작하지 못하는 문제를 발견했습니다. 공식 SG2의 Odometry 정의와 같은 `[linear_x, linear_y, angular_z]`로 수정했습니다. 이는 Canvas 전용 이동 상태 필드이며 손·팔 정책 차원은 그대로입니다. 수정 후 Cyclo 본체와 rosbridge의 정상 기동도 확인했습니다.

수정본으로 직접 ROS 호출의 Task Engine 재시작 15회, rosbridge 경유 재시작 24회(임시 포트 12회 + 실제 UI 포트 12회)를 무동작 Wait 작업으로 확인했습니다. 실제 UI 포트에서 잘못된 XML의 오류 응답, 정상 Wait의 로드·완료·중단도 확인했으며 관련 회귀 테스트 14개가 통과했습니다. 이 완료 시험에만 테스트 엔진의 시계를 잠시 wall time으로 설정한 뒤 엔진을 종료했습니다. 저장된 프로필의 `use_sim_time=true`는 유지합니다.

점검 중 Gazebo와 Navigation은 이미 종료되어 있어 이 결과를 실제 전체 미션 주행 성공으로 간주하지 않습니다. 다시 실행할 때는 1절의 Gazebo 실행 후 저장 지도 Localization을 시작하고 현재 위치를 맞춘 뒤 Run을 누릅니다. Cyclo ROS는 보완본으로 재시작했고 시험용 엔진과 임시 rosbridge는 종료했습니다.

### Mission Route의 Rotate 연결 보완 (2026-09-16)

`0914_BGF_test_1`은 경로 연결 후 첫 Waypoint 도착까지 성공했지만, `Rotate`가 요구하는 mobile Twist 명령과 Odometry 피드백이 SH5 Canvas 설정에서 빠져 작업이 실패했습니다. 시뮬레이션 전용 `behavior_tree.action.mobile=/cmd_vel`, `behavior_tree.state.mobile=/odom`을 추가했습니다. 정책의 54차원 observation/action, 실제 로봇 설정, 손·촉각 코드는 변경하지 않습니다.

또한 Task Engine 종료 시 기존 finish 스크립트는 30초를 기다리지만 s6의 기본 finish 제한은 5초여서 정리가 중단되고 ROS 자식 프로세스가 남았습니다. 이 프로필만 사용하는 `ros_service_finish.sh`는 기록된 서비스 프로세스 그룹에 TERM을 보낸 뒤 최대 2초를 기다리고, 남은 프로세스에 KILL을 보내 정리합니다. 다른 컨테이너나 Gazebo/Nav2 프로세스를 검색해 종료하지 않습니다.

수정 후 `Rotate angle_deg="0"` 작업의 실제 `completed` 응답과 엔진 종료 후 잔류 프로세스 0개를 확인했습니다. 관련 테스트 10개 통과. 사용자 미션의 90도 회전과 전체 경로는 자동 재실행하지 않았으며, 현재 지도·Gazebo·Nav2는 유지했습니다. 종료 제한 근거: [s6-supervise 공식 설명](https://skarnet.org/software/s6/s6-supervise.html).

현재 촉각은 기준 Cyclo 저장소에 있던 보정된 Gazebo 센서를 실행한 것이 아닙니다. 공식 손 모델·메시지 규격과 기준 저장소의 촉각 UI/수집 인터페이스에 맞춘 별도 contact-force 어댑터입니다. `8 N → 255`와 `24 × 40 mm` 가상 격자는 커스텀 근사값이며 실제 HX5의 보정값이 아닙니다.

- 전용 이미지 빌드, 새 컨테이너 재생성, Gazebo/RViz 구동, UI 7380 응답.
- 8개 controller 활성화, 양손 40개 관절의 완전 열림 ↔ 닫힘 3회 반복.
- SH5 Task Engine의 `JointControl → ArmStateGate → Wait` 실제 완료.
- 4개 서로 다른 카메라 스트림, LiDAR `/scan`, 54관절 상태 수신.
- 물체 접촉 시 실제 contact force → 촉각 pressure 증가, 물체 제거 후 0 복귀.
- Mapping 지도 생성/저장, 저장 지도 Localization, Nav2 40 cm 목표 주행 성공.
- MCAP 녹화/저장, 네 카메라 영상 처리, 양손 pressure·54관절·시뮬레이션 메타데이터 기록.
- 관련 회귀 테스트 180개 통과. 이번 검증용 Task 데이터와 지도, 임시 접촉 물체는 제거했습니다.

**아직 실제 연결로 검증하지 않은 범위:** USB LG2 리더(현재 장치 없음), 모든 UI 조합/전체 Mission Route의 장시간 실행, 물체별 안정적인 파지·운반 성공률, 실기기 1044/HX5, 학습·모델 추론. 모든 기능을 실로봇 수준으로 검증했다는 뜻은 아닙니다. 특히 리더 트리거→손 프리셋 변환과 손가락 개별 조작의 차이를 3절에서 확인합니다.
