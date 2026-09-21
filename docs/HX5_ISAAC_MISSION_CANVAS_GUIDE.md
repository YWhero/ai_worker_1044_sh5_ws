# Isaac Sim · SH5/HX5 · Mission Canvas 실행 안내

작업 위치는 `/home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws`, 물류 환경은 `/home/robotis-ai/workspaces/isaac_logistics_cell_ws`입니다. Isaac 물류 환경의 기본 로봇과 실행 경로를 SH5/HX5로 전환했습니다. 컨베이어·바구니·물체의 치수와 저장된 시작 배치는 유지합니다. Gazebo는 기존 domain 105 프로필을 계속 사용합니다.

## 실행 CLI

최초 준비는 환경만 담은 `logistics_cell.environment.usda`를 생성하고 공식 SH5 URDF를 가져와 기본 `logistics_cell.usda`에 동적 로봇·센서를 구성합니다. 기존 정적 SG2 모델이나 머리 링크의 수동 회전을 기본 실행에 사용하지 않습니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh prepare
./runtime/hx5_isaac.sh start
```

터미널 1에서 Isaac을 실행합니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh isaac
```

현재 설치된 `/home/robotis-ai/isaac_sim/app/6.1.0`을 사용합니다. 다른 설치 경로는 실행할 때만 `ISAAC_SIM_DIR=/path/to/isaac-sim ./runtime/hx5_isaac.sh isaac`으로 지정합니다. GUI가 필요 없는 실행에는 `isaac --headless`를 사용합니다.

물류 워크스페이스의 기본 실행도 SH5/HX5 씬을 사용합니다.

```bash
/home/robotis-ai/workspaces/isaac_logistics_cell_ws/launch.sh
```

터미널 2에서 Cyclo ROS를 시작합니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh cyclo
```

Cyclo가 실행 중인 상태에서 터미널 3으로 센서 진단을 수행합니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh validate --duration 8
```

브라우저는 **`http://localhost:7880/`**, 로봇 종류는 **FFW SH5 Rev1 (`ffw_sh5_rev1`)**입니다. Isaac timeline이 재생 중이고 센서가 준비된 상태에서 Capture와 Task를 실행합니다.

**7380은 Gazebo, 7880은 Isaac 제어 화면입니다.** 이번 Mobile 무동작 제보는 7380에서 Isaac을 제어하려던 경우였으며, 사용자가 7880에서 정상 동작을 확인했습니다. UI 업데이트 이후에는 7880 페이지를 새로고침합니다. 새 브라우저 프로필에서 로봇 선택값이 비어 있으면 FFW SH5 Rev1을 선택합니다.

| 구성 | Isaac | 기존 Gazebo |
|---|---|---|
| ROS domain | `115` | `105` |
| Zenoh router | `tcp/127.0.0.1:7855` | `tcp/127.0.0.1:7455` |
| Cyclo UI | `7880` | `7380` |
| ROS bridge | `7890` | 기존 프로필 |
| Supervisor API | `7900` | 기존 프로필 |
| 영상·카메라 서비스 | `7882` / `7885` / `7886` | 기존 프로필 |
| Isaac adapter socket | `7866` | 사용하지 않음 |
| AI Worker | `ai_worker_1044_hx5_isaac` | `ai_worker_1044_hx5_sim` |
| Cyclo | `cyclo_intelligence_1044_hx5_isaac` | `cyclo_intelligence_1044_hx5_sim` |
| ViTacFormer | `vitacformer_server_1044_hx5_isaac` | `vitacformer_server_1044_hx5_sim` |
| 운영 데이터 | `simulation/isaac/cyclo/` | `simulation/cyclo/` |

Isaac 프로세스는 host의 ROS 환경을 변경하지 않고 simulator API와 전용 socket을 사용합니다. ROS·Zenoh 연결은 전용 컨테이너에서 처리합니다. 지도·녹화·미션·Preset과 캐시는 Isaac 경로에 저장하며 기존 Gazebo 원본을 수정하지 않습니다. 공유 checkpoint는 `/workspace/model`에 읽기 전용으로 연결합니다. 기존 Mission/Preset의 snapshot은 재사용할 수 있지만 Gazebo 지도와 Isaac 지도 좌표가 같다는 뜻은 아닙니다.

Isaac UI의 ROS WebSocket은 UI와 같은 주소의 `/rosbridge/`를 사용합니다. 로컬에서는 `ws://localhost:7880/rosbridge/`이며 nginx가 내부 7890으로 연결합니다. 따라서 UI용 7880을 포워딩한 주소에서도 ROS 연결에 별도 7890 포워딩이 필요하지 않습니다. 영상과 API 서비스는 위 표의 별도 포트를 계속 사용합니다. Gazebo의 ROS 연결 설정은 유지합니다.

빌드한 UI를 Isaac 프로필에만 다시 적용할 때는 다음 명령을 사용합니다. Isaac 전용 `simulation/isaac/ui/`에 저장하고 현재 프로필의 `cyclo-config.js`를 보존합니다.

```bash
./runtime/hx5_isaac.sh ui-install /absolute/path/to/ui/build
```

## 시작 자세와 물리 구성

2026-09-18 요청 당시의 실제 자세·위치를 Isaac 초기값으로 저장했습니다. spawn은 `(0.3602113128, -0.7280516624, 0.0000001490) m`, yaw `90.0636840638°`이며 전체 quaternion도 보존합니다. 팔·양손·Head·Lift의 57개 실측 목표는 Isaac 전용 `simulation/isaac/config/initial_pose.yaml`을 읽습니다. Head pitch는 `0.6863727570 rad`, yaw는 `0.0000118094 rad`, Lift는 `-0.0055988370 m`입니다. 일반 실행과 reset이 같은 시작값을 사용합니다. [초기값·Record 3D 검증](HX5_ISAAC_INITIAL_POSE_RECORD_GUIDE_20260918.md)에 원본과 실측 결과를 기록했습니다.

기존 `src/hx5_simulation/hx5_simulation/initial_poses/vitacformer_task519_sync.yaml`은 Task519의 고정 추론 정렬 참조로 유지합니다. 새 Isaac 초기값 파일과 구분하며, 입력에 따라 달라지는 모델의 live 초기 정렬과 tracking 검사도 유지합니다.

SH5는 팔 14축·손 40축·Head 2축·Lift 1축 및 swerve 6축을 갖습니다. 정책 state/action은 **54차원**, Canvas가 사용하는 자세는 **57축**입니다. swerve의 steering/wheel은 정책의 54차원 배열에 추가하지 않습니다.

공식 URDF의 articulation·관절 한계·drive를 가져오며 컨베이어 주변의 72개 대상 물체에는 시작 자세를 유지한 동적 물리를 구성합니다. object1 `0.08 kg`, object2 `0.05 kg`은 기존 settling에 사용한 시뮬레이션 가정값입니다. 봉지의 변형, 실제 마찰·질량·파지 정확도를 보정한 결과는 아닙니다. 접촉 force에서 변환한 uint8 촉각 값도 실제 HX5 센서의 보정값과 다릅니다.

시작 직후 object2 비산을 수정하여 물체는 적재 생성과 같은 단일 convex hull, 바구니는 열린 5개 박스 충돌 proxy를 사용합니다. 보이는 scan 외형과 저장 시작 배치는 유지하며 물체는 계속 dynamic입니다. Head 영상의 원형 가림도 렌즈 경로의 visual CAD를 수정하여 제거했습니다. 카메라 optical frame과 로봇 collision은 유지합니다. [카메라 비교 이미지·물체 안정성 검증](HX5_ISAAC_CAMERA_PILE_FIX_20260918.md)에 구현 근거와 실제 측정 결과를 기록합니다.

## Cyclo가 읽고 발행하는 topic

| 기능 | Topic / 메시지 |
|---|---|
| 정책 관절 54축 | `/arm_hand/joint_states` · `sensor_msgs/JointState` |
| Head·Lift 상태 | `/joint_states` · `sensor_msgs/JointState` |
| 양손 촉각 | `/left_hand/finger_pressures`, `/right_hand/finger_pressures` · `robotis_interfaces/HandPressures` |
| 좌우 팔 명령 | `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory`, `..._right/joint_trajectory` |
| 좌우 손 명령 | `/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory`, `..._right_hand/joint_trajectory` |
| Head·Lift 명령 | `/leader/joystick_controller_left/joint_trajectory`, `..._right/joint_trajectory` |
| 주행 명령 | `/cmd_vel` · `geometry_msgs/Twist` |
| 물리 pose 기반 주행 상태 | `/odom` · `nav_msgs/Odometry`, `odom → base_link` TF |
| Navigation 거리 입력 | `/scan` · `sensor_msgs/LaserScan` |
| 시뮬레이션 시간 | `/clock` · `rosgraph_msgs/Clock` |

카메라 이름과 topic은 기존 SH5 계약을 유지합니다.

| 카메라 | JPEG topic | CameraInfo |
|---|---|---|
| 왼쪽 Head | `/zed/zed_node/left/image_rect_color/compressed` | `/zed/zed_node/left/camera_info` |
| 오른쪽 Head | `/zed/zed_node/right/image_rect_color/compressed` | `/zed/zed_node/right/camera_info` |
| 왼쪽 Wrist | `/camera_left/camera_left/color/image_rect_raw/compressed` | `/camera_left/camera_left/color/camera_info` |
| 오른쪽 Wrist | `/camera_right/camera_right/color/image_rect_raw/compressed` | `/camera_right/camera_right/color/camera_info` |

관절·촉각·카메라·scan·TF의 stamp는 같은 simulation clock을 사용해야 합니다. `/clock`과 `/odom` publisher는 각각 하나입니다. TF에서 같은 child에 둘 이상의 parent가 있으면 먼저 중복 발행을 정리합니다. CameraInfo와 영상의 optical frame도 맞아야 합니다.

## Capture · JointControl · Gate

Capture는 SH5의 선택된 관절만 읽습니다. 양팔+Head+Lift는 17축, 손까지 포함하면 57축입니다. ArmStateGate의 팔 target 옵션은 각 7축이고, 손 target 옵션은 각 20축으로 분리됩니다. 기존 저장 Gate의 팔 list에 있던 손가락 목표는 위치 CSV와 함께 손 target으로 옮깁니다. 서로 다른 중복 목표는 값을 보존하고 경고를 표시하므로 실행 전 하나를 선택합니다.

JointControl과 ArmStateGate의 상세 필드·닫힘→열림 이벤트·촉각 조건은 [SH5/HX5 통합 가이드](HX5_SH5_MISSION_CANVAS_INTEGRATION_20260918.md)를 사용합니다. 촉각에서 빈 sensor list는 해당 손의 **다섯 손끝 센서 전체**를 선택하며 센서마다 유효한 9개 값을 요구합니다. `released`는 현재 무접촉 상태만으로도 통과합니다. 이전 파지를 확인하려면 contact Gate→released Gate 또는 손 close→open 이벤트를 구성합니다.

Gate의 contact 성공은 파지 성공 인증이 아닙니다. 기본 `contact_pressure_threshold=30`은 9개 raw 값의 센서별 합계이며 무접촉 baseline에 맞춰 조정합니다. `state_max_age_sec`, `hold_sec`, 유한 `timeout_sec`도 설정합니다.

Isaac에서도 Gazebo와 같은 `hx5_simulation` Hand Preset 어댑터를 실행합니다. `/leader/hand_preset/set`, `custom/save`, `manage/update`, `manage/delete`, `manage/restore` 서비스와 Preset 상태를 제공합니다. 기존 팔 명령은 이 어댑터를 거쳐 Isaac 내부 팔 controller로 전달되며, 손 명령은 기존 20축 hand topic을 사용합니다. Mobile 및 Head/Lift Jog의 연결 실패·명령 실패·확인 시간 초과는 해당 패널에 표시합니다.

## ViTacFormer와 다음 액션

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh policy-start vitacformer
```

SendCommand LOAD는 `target=INFERENCE`, `model=vitacformer:vitacformer`, `policy_path=/workspace/model/vitacformer/task519_pour_h100`를 사용합니다. `inference_mode=robot`은 **domain 115의 Isaac으로 명령을 발행**하고 `simulation`은 명령 없는 preview입니다. 모델 시작 자세 정렬이 필요하면 LOAD의 `initial_pose_sync=true`, duration 5초를 명시합니다.

Task는 **LOAD → RESUME → ArmStateGate → STOP → 다음 액션** 순서로 구성합니다. LOAD는 필요 시 SYNCING을 마친 뒤 PAUSED로 완료하고, RESUME가 INFERENCING을 확인한 뒤 Gate를 시작합니다. Gate는 추론을 중단하지 않으므로 STOP을 이어 붙이며 마지막에는 CLEAR로 READY를 확인합니다.

Gate timeout이나 Task 실패로 Sequence가 실패하면 뒤의 STOP이 실행되지 않습니다. Mission/Task를 중단하고 추론 패널 **Stop → PAUSED → Clear → READY**로 정리합니다. Task Engine을 끄는 것만으로 backend의 추론이 멈췄다고 가정하지 않습니다. API가 응답하지 않으면 `policy-stop vitacformer`로 전용 backend를 종료한 뒤 Cyclo 상태를 다시 정리합니다.

## Mapping · Navigation · 녹화

Isaac의 새 배치에서 **Mapping으로 새 지도**를 만들고 Localization의 현재 위치·방향을 지정한 뒤 Waypoint를 저장합니다. Gazebo 미션의 Task 파일·Preset을 재사용하더라도 저장된 지도·Waypoint 좌표는 Isaac 환경에 맞춰 확인합니다. `/scan`이 로봇 자기 몸을 장애물로 측정하지 않는지, 센서 TF가 실제 mounted laser와 맞는지도 확인합니다.

Recorder는 네 영상과 SH5 54축 상태/action, 양손 촉각 및 구성한 `/clock`·TF·CameraInfo 등의 metadata를 기존 계약에 따라 저장합니다. Head·Lift를 정책 학습 배열에 섞지 않습니다. 저장 Episode의 camera frame 수와 stamp, MCAP state/action/tactile 수 및 변환된 54차원 배열을 확인해야 학습 데이터 연결까지 검증한 것입니다.

물리 LG2를 연결하는 [Skeleton Leader 수집 가이드](HX5_ISAAC_SKELETON_LEADER_GUIDE_20260918.md)에 `leader --check`, `leader`, `leader --mobile`, `leader-stop`과 Record 입력 순서를 안내합니다. Isaac Recorder는 영상·상태와 MCAP action을 같은 simulation time에 저장합니다. Record 3D도 `/joint_states`의 실제 Head/Lift를 함께 구독합니다. Leader를 실행 중이면 Canvas 제어나 reset 전에 `leader-stop`으로 종료합니다.

## 읽기 전용 진단과 종료

```bash
./runtime/hx5_isaac.sh validate --duration 8
./runtime/hx5_isaac.sh status
./runtime/hx5_isaac.sh enter
# 전용 AI Worker 컨테이너의 ROS shell입니다. 빠져나오려면 exit.
```

진단은 publisher나 서비스 명령을 만들지 않습니다. 57축 유한값, 손별 5×9 uint8 촉각의 이름·순서, 실제 decode 가능한 Head 672×376 / Wrist 424×240 RGB JPEG와 CameraInfo (선택한 프로파일은 scene metadata 기준), 정확한 optical frame, timestamp 진행·신선도, 유효한 scan, 유일한 clock/odom publisher, odom/TF pose 일치와 sensor TF 경로를 검사합니다. probe 중 발생한 malformed 메시지는 다음 정상 메시지로 지우지 않습니다. 종료 코드 0은 상태·transport 검사 통과, 1은 계약 위반, 2는 실행 환경·입력 오류입니다. 관절 움직임 확인이 필요하면 외부에서 Task를 실행하는 동안 `--min-joint-change 0.01`을 사용합니다. 특정 목표 비교는 joint→position JSON 파일을 `--expect-pose`로 전달합니다. 진단 통과만으로 모델 수행·파지·Navigation route 성공을 판정하지 않습니다.

초기화 전 녹화 READY, Mission/Navigation OFF, Task Engine OFF, 추론 STOP/CLEAR를 확인합니다.

```bash
./runtime/hx5_isaac.sh policy-stop vitacformer
./runtime/hx5_isaac.sh reset --check
./runtime/hx5_isaac.sh reset
# 이후 Localization의 위치·방향을 다시 지정합니다.

# 시뮬레이터와 전용 컨테이너까지 종료할 때:
./runtime/hx5_isaac.sh sim-stop
./runtime/hx5_isaac.sh stop
```

`reset --check`는 idle 조건만 검사합니다. `reset`은 실행 중인 Isaac의 PhysX world를 초기화하고 로봇 63축의 자세·속도와 72개 물체의 위치·방향·속도를 복원한 뒤 완료 응답을 기다립니다. 이전 trajectory와 주행 명령은 비웁니다. 생성된 Episode·지도·Task·Preset을 삭제하지 않습니다. 초기화 뒤에는 Localization의 위치·방향을 다시 맞춥니다.

`sim-stop`은 같은 idle 조건을 확인한 뒤 Isaac 앱에 정상 종료를 요청하고 전용 ROS·Cyclo 컨테이너는 유지합니다. 직접 실행한 simulator 터미널에서는 `Ctrl+C`도 사용할 수 있습니다. `stop`은 전용 ROS·Cyclo·policy 컨테이너를 내리는 명령이므로 먼저 Isaac을 종료합니다. CLEAR 뒤 backend가 계속 실행 중이지만 READY heartbeat가 없는 경우에도 초기화 상태를 추측하지 않도록 `policy-stop vitacformer`를 거친 뒤 reset합니다.

## 공식 근거와 현재 확인 범위

- [NVIDIA URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/import_urdf.html): articulation·관절 limit와 drive 구성을 가져오는 근거.
- [NVIDIA Camera Sensors](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_camera.html): 실제 렌더 영상과 sensor 갱신 주기.
- [NVIDIA Contact Sensor](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_contact.html): PhysX의 실제 접촉과 force 데이터를 읽는 근거. HX5 9-taxel raw 변환은 이 워크스페이스의 구현입니다.
- [NVIDIA ROS 2 Clock](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_series/tutorial_ros2_clock.html): 시뮬레이션 시간과 sensor stamp 동기화.
- [ROBOTIS SH5 계약](https://github.com/ROBOTIS-GIT/cyclo_intelligence/blob/main/shared/shared/robot_configs/ffw_sh5_rev1_config.yaml), [ROBOTIS Hand](https://github.com/ROBOTIS-GIT/robotis_hand): 관절·영상·촉각·action 이름과 메시지 기반.

2026-09-18 현재 읽기 전용 진단의 malformed 관절·촉각·scan, 센서 이름·순서, JPEG decode·해상도·frame, stale/역행 stamp, TF 중복 부모·cycle·quaternion, ROS NumPy scalar의 JSON 저장 및 기존 domain 106 연결 거부 검증 **12개 테스트를 통과**했습니다. 환경 baseline은 기존 바구니 4개·40/32 물체 배치를 유지하여 실제 생성했습니다. 준비된 실제 SH5 USD에서 **632개 검사 모두 통과**했습니다. frozen 57축과 drive 시작값, 63개 관절, 네 카메라, 열 손끝의 실제 collider, 72개 동적 물체와 가정 질량, 환경 치수·배치·자산 hash·파일 의존성 및 spawn을 검사했습니다.

실제 Isaac PhysX의 직접 smoke 검사에서는 63 DOF·120 Hz state, 구별되는 4개 336×188 부착 카메라·15 Hz 영상, 720방향·10 Hz raycast scan을 확인했습니다. scan은 실제 외부 물체 303개 방향에서 유한 거리를 반환했고 최소 거리는 `1.007 m`였습니다. `0.1 m/s`를 1초 명령한 물리 휠 구동은 `0.0991465 m` 전진했으며 lateral 이동은 약 `5 µm`였습니다. Head `0.08 rad` 명령의 실제 오차는 `0.003446 rad`였습니다. 접촉 probe는 기존 carton과 손끝 2·3의 실제 비영 촉각을 확인했으며 의도한 깊은 겹침에서의 큰 접촉 force는 파지나 센서 보정 결과로 사용하지 않습니다.

같은 smoke 검사에서 초기화 응답과 trajectory·주행 명령 비움을 확인했고 72개 물체 위치의 최대 복원 오차는 `5.96e-8 m`, 로봇 위치는 `9.42e-8 m`였습니다. 증거 파일은 `/tmp/hx5-isaac-smoke.json`, `/tmp/hx5-isaac-physics-smoke.json` 및 대응 로그입니다. 이 수치는 simulator의 직접 명령·물리 검사 결과입니다.

실제 domain 115 ROS 연결의 **15개 topic 계약 진단이 통과**했습니다. 8초 probe에서 joint·clock·odom·각 손 촉각은 281개 메시지, 각 JPEG·CameraInfo는 35개, scan은 24개를 수신했습니다. 네 RGB JPEG decode·해상도·frame, 손별 5×9 raw 값의 순서·신선도, 정적·동적 sensor TF 경로를 통과했고 `/clock`·`/odom` publisher는 각각 하나였습니다. 같은 stamp의 odom/TF 위치·방향 오차는 0이었습니다. 보고서는 `simulation/isaac/ai_worker/diagnostics/isaac-contract.json`입니다. 기존 두 Isaac GUI와 Gazebo가 함께 실행된 이 측정에서 simulation real-time factor는 약 `0.29`였으므로 simulation Hz와 wall-time 수신률을 구분합니다.

실제 Isaac의 54축·양손 45-taxel·네 영상으로 **ViTacFormer preview Task가 9.36초에 완료**했습니다. CUDA의 Task519 step 34000 checkpoint를 LOAD하고 RESUME→Wait→손 target/양손 released contact Gate→STOP→CLEAR를 실행했습니다. `publish_to_robot=False`로 trajectory 발행은 0개였고 실제 관절 drift는 `9e-6` 미만이었습니다. 이 결과는 live 입력·추론·Gate·STOP 연결을 확인하며 실제 로봇 명령이나 PourWater 전체 수행을 확인한 결과는 아닙니다.

Cyclo Recorder의 공식 `/data/recording` START→**3초간 수동 명령 없는 녹화**→FINISH가 성공했고 READY 0으로 돌아왔습니다. 결과는 `simulation/isaac/cyclo/rosbag2/Task_20260918_IsaacIntegrationSmoke20260918_MCAP/0/`입니다. MCAP에서 54축 정책 상태와 Head·Lift를 포함한 전체 63축 feedback, 손별 5×9 raw 촉각·45개 Newton force, clock·odom을 각각 124개, TF 321개, scan 11개 및 실제 tactile model 정보 3개를 읽었습니다. `schema_version=cyclo_isaac_mcap`, `tactile_source=physx_contact_force_projected_grid_v1`, `hardware_calibrated=false`를 확인했습니다.

현재 공식 녹화 format v2는 **JPEG를 MCAP에 넣는 대신 네 MP4와 CameraInfo YAML을 별도로 저장**합니다. 저장된 네 H.264 MP4에서 각각 15개 frame을 decode했고 동일한 15행의 header timestamp parquet이 증가하는지 확인했습니다. Head 영상은 336×188, Wrist는 구성의 270° 회전 후 188×336이며 CameraInfo 원본은 336×188입니다. invalid·queue drop은 모두 0이었습니다. 보고서는 `simulation/isaac/cyclo/diagnostics/recorder-smoke.json`입니다. 이 수동 관찰 녹화에서 팔·손 action 명령은 없었으며 LeRobot 변환이나 학습용 action Episode의 성공을 확인한 것은 아닙니다.

실제 scan으로 Mapping→전용 진단 지도 저장→Navigation 재로딩→초기 위치 지정→`(0.15,0,0)` 목표 실행이 **12.8459초에 SUCCEEDED**, error 0·recovery 0으로 완료했습니다. 목표와의 map XY 오차는 `0.0286 m`, spawn 기준 실제 odom XY 오차는 `0.0854 m`였고 공식 `0.10 m` 허용 범위 안입니다. 짧은 정지 scan으로 만든 진단 지도여서 초기 AMCL Y offset은 약 `-0.148 m`였습니다. 긴 Mission 실행 전에는 작업장 전체 Mapping과 위치·방향 확인이 필요합니다. 지도·실제 이동 및 제한 사항은 [Isaac Navigation smoke 기록](HX5_ISAAC_NAVIGATION_SMOKE_20260918.md)에 있습니다.

실제 Isaac에서 **기본 57축 Gate → JointControl → 변경 자세 57축 Gate → JointControl 복귀 → 기본 57축 Gate**가 **4.86초에 완료**했습니다. 양팔 wrist 목표를 각각 ±0.03 rad, 양손 finger 3을 +0.04 rad, Head pitch/yaw를 −0.04/+0.04 rad, Lift를 −0.015 m 움직였습니다. 실제 feedback에서 각각 약 0.0284 rad, 0.0400 rad, 0.0381/0.0378 rad, 0.0164 m 변화를 확인했고 복귀 뒤 57축 최대 오차는 **0.003363 rad**였습니다. Gate의 기본 position 허용값 0.01과 초기 참조 자세를 유지했습니다. 증거는 `simulation/isaac/cyclo/diagnostics/jointcontrol-gate-smoke.log`입니다.

처음의 왼쪽 엄지 0.01354 rad 오차는 실제 native PD 조건 15개를 비교하여 손의 USD Kp320/Kd8로 조정했습니다. solver는 position64/velocity4를 사용합니다. 자세 오차와 reported velocity를 구분한 결과·단위·제한은 [native PD 검토](HX5_ISAAC_NATIVE_PD_REVIEW_20260918.md)에 기록했습니다. 최종 어댑터 검사 **44개가 통과**, host Python에 USD가 없어 2개가 생략됐으며 실제 USD의 632개 검사는 별도로 모두 통과했습니다.

`inference_mode=robot`에서도 실제 Isaac 입력으로 CUDA LOAD와 5초 초기 정렬 명령까지 연결됐습니다. 다만 새 입력에서 예측한 source action의 `arm_l_joint1` step 4→5 변화량 **0.090673 rad**가 기존 **0.090000 rad** 제한을 넘어 backend가 명령 loop를 중단했습니다. 이 제한을 완화하지 않았습니다. Task Engine 종료 후 명시적인 STOP→UNLOAD 성공을 확인하고 전용 policy 서버를 종료했으며, reset으로 기본 63축과 72개 물체 배치를 복원했습니다. 실패 이유는 `simulation/isaac/cyclo/diagnostics/policy-robot-guard.log`에 있습니다. 모델의 연속 실제 명령·PourWater 전체 수행·파지 성공·긴 Mission·학습 데이터 변환은 확인하지 않았습니다.

복원 후 최종 6초 진단에서도 15개 topic과 57축 기준 자세 비교가 통과했습니다. joint·clock·odom·각 손 촉각 249개, 네 영상·CameraInfo·scan과 sensor TF를 확인했고 clock/odom publisher 각각 하나, 같은 stamp의 odom/TF 위치·방향 오차 0이었습니다. 최종 보고서는 `simulation/isaac/ai_worker/diagnostics/isaac-final-contract.json`입니다. 이 측정의 simulation 진행률은 약 0.34로 실제 시간보다 느렸습니다.

현재 SH5/HX5 Isaac GUI와 Cyclo UI는 실행 중이며 Task Engine·Navigation은 OFF, 모델 서버는 STOP 상태입니다. 기존 Gazebo와 다른 워크스페이스의 컨테이너는 유지했습니다. 재시작은 위 CLI를 사용하고, 실제 모델 동작 전에는 해당 씬 입력에 대한 checkpoint와 명령 제한 검토가 필요합니다.

## Mobile 및 관련 제어 점검 · 2026-09-18

7380/7880 접속 구분 외에 실제로 누락된 Hand Preset 서비스와 swerve 조향 경계 처리를 수정했습니다. 조향은 공식 관절 한계 안의 정방향·역방향 후보 중 실제 이동 거리가 짧은 목표를 선택하며, 경계를 가로질러 아직 정렬되지 않은 wheel에는 구동 속도를 주지 않습니다. 구현 근거는 [ROBOTIS swerve controller](https://github.com/ROBOTIS-GIT/ai_worker/blob/main/ffw_swerve_drive_controller/src/swerve_drive_controller.cpp), [rosbridge 프로토콜](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md), [nginx WebSocket proxy](https://nginx.org/en/docs/http/websocket.html)입니다.

실제 7880 WebSocket으로 Mobile 명령을 보내 약 0.0399 m 이동했고, 정지 명령 뒤 추가 이동은 약 7.5 µm였습니다. Head·Lift·양팔 목표 및 양손 Preset 목표를 실제 PhysX feedback으로 확인했습니다. 사용자 Preset 저장·수정·삭제도 성공했으며 테스트용 Preset은 삭제했습니다. restore 서비스는 제공 여부를 확인했으나 저장된 사용자 Preset 전체를 복원하는 호출은 하지 않았습니다. 증거는 `simulation/isaac/cyclo/diagnostics/teleop-controls-proxy-smoke.json`입니다.

Rotate +10°→−10°와 기본 57축 Gate→JointControl→변경 57축 Gate→복귀 JointControl→기본 57축 Gate가 15.99초에 완료됐습니다. 기본 자세 복귀 후 최대 관절 오차는 0.003363 rad였으며 Gate 허용값 0.01을 유지했습니다. 증거는 `simulation/isaac/cyclo/diagnostics/rotate-jointcontrol-gate-smoke.log`입니다. 이번 점검은 긴 Navigation Mission이나 모델의 연속 추론 수행을 새로 검증한 결과는 아닙니다.

최종 reset 후 15개 topic 계약과 기본 57축 자세 비교가 다시 통과했습니다. 보고서는 `simulation/isaac/ai_worker/diagnostics/isaac-teleop-fixed-contract.json`입니다. Python 관련 검사 50개 통과·USD 없는 host 검사 2개 생략, UI 관련 46개 및 nginx 설정 12개 검사와 production 빌드가 통과했습니다. 실제 Chrome에서도 새 bundle과 `ws://127.0.0.1:7880/rosbridge/` 연결 및 ROS 읽기 서비스 응답을 확인했습니다. 새 프로필의 이 마지막 읽기 전용 접속에서는 로봇을 선택하지 않아 Connected·카메라 표시까지 확인한 것으로 기록하지 않습니다.

GUI와 Isaac Cyclo는 실행 중이며 기본 자세·물체 배치로 복원했습니다. Task Engine·Navigation은 OFF입니다. 기존 Gazebo UI와 컨테이너는 유지했습니다.


2026-09-18 카메라 화질 설정은 [공식 카메라 프로파일 적용 및 원본·미리보기 비교](HX5_ISAAC_CAMERA_QUALITY_20260918.md)를 참고하세요. 현재 기본값은 Head ZED Mini 672×376, Wrist D405 424×240, 모두 30 SIM Hz와 원본 JPEG95이며 Isaac 미리보기도 JPEG95를 사용합니다. 위 이전 smoke 결과의 336×188·15 Hz는 변경 전 실측 기록입니다.
