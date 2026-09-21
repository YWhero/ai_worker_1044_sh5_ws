# Isaac SH5/HX5를 LG2 Skeleton Leader로 조작하고 녹화하기

워크스페이스는 `/home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws`, Isaac ROS domain은 **115**, Zenoh router는 **7855**, Cyclo UI는 **http://localhost:7880/** 이다. Home에서 follower **FFW SH5 Rev1**을 선택한다. `7380`은 별도 Gazebo UI다.

## 시작과 장치 확인

Isaac GUI와 Cyclo가 실행되어 있고, Mapping/Navigation과 Task Engine이 꺼져 있어야 한다. 녹화는 READY, 모델 추론은 STOP/CLEAR 상태로 둔다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh status
./runtime/hx5_isaac.sh leader --check
./runtime/hx5_isaac.sh leader
```

`leader --check`는 장치를 열거나 전원을 켜지 않고 두 USB 장치·중복 사용·서비스 상태·실제 follower feedback·공식 손 mapping 준비 상태만 확인한다. `leader`는 **물리 LG2 Leader**를 시작한다. 기본 장치는 `/dev/left_leader`, `/dev/right_leader`이며, 장치가 없으면 컨테이너를 시작하기 전에 실패한다. 다른 경로를 사용할 경우 실행한 셸에 다음을 지정한다.

시작 검사는 status topic의 마지막 메시지만으로 매핑을 판정하지 않는다. 단일 HandPreset 노드의 실제 `mapping_profile`·`robot_description`·`use_sim_time` parameter와 고정 공식 파일 hash를 읽는다. 중복 publisher와 멈춘 clock·오래된 관절 feedback은 시작을 거부한다. 상태 메시지 수신이 지연되어도 실제 parameter로 확인하며, 오류는 원인을 포함한 짧은 문구로 표시한다.

`leader`는 컨테이너 생성 후 최대 30초 동안 공식 LG2 컨트롤러 5개가 모두 active이고, 16축 실제 feedback이 유효하고 신선하게 진행되는지도 확인한다. 실패하면 로그를 출력하고 리더 컨테이너를 종료한다. 성공 JSON의 `ready: true`와 `controllers`를 확인한다.

```bash
export HX5_ISAAC_LEFT_LEADER_DEVICE=/dev/serial/by-id/왼쪽장치이름
export HX5_ISAAC_RIGHT_LEADER_DEVICE=/dev/serial/by-id/오른쪽장치이름
./runtime/hx5_isaac.sh leader --check
./runtime/hx5_isaac.sh leader
```

Leader 전용 컨테이너 `lg2_leader_1044_hx5_isaac`에는 이 두 장치만 노출한다. 실제 follower의 USB 장치는 노출하지 않는다. 실행 로그는 다음으로 확인한다.

```bash
docker logs -f lg2_leader_1044_hx5_isaac
```

양팔을 Isaac follower의 현재 자세와 비슷하게 맞춘 뒤 양쪽 gripper trigger를 약 **2초** 눌러 arm following을 켠다. 공식 broadcaster의 기본 activation은 off이며, 같은 trigger 동작으로 following을 토글한다.

## 팔·손·머리·lift·이동 입력

| 입력 | Isaac 출력 |
|---|---|
| LG2 왼팔/오른팔 7관절 | 각 SH5 팔 7관절; 실제 가져온 URDF 한계로 clamp |
| 각 LG2 gripper 1값 | 선택된 HandPreset에 따라 각 HX5 손 20관절 release↔grasp 보간 |
| 기본 arm_control 모드 왼쪽 joystick X/Y | `head_joint1`/`head_joint2` |
| 기본 arm_control 모드 오른쪽 joystick X | `lift_joint` |
| swerve 모드 왼쪽 X/Y | base 전후/좌우 이동 |
| swerve 모드 오른쪽 Y / X | base 회전 / lift |

Gripper 하나로 20개의 손 관절을 독립 조작할 수는 없다. UI HandPreset의 active preset와 다섯 finger curl 비율을 적용한 하나의 개폐 정도로 제어한다. Built-in `0`은 공식 1044 release, `1`은 공식 1044 grasp다. `2`는 같은 공식 endpoint에 thumb/index mask를 적용한 이 워크스페이스의 파생 preset이다.

기본 실행은 이동을 끈 **arm_control** 모드다. 이동도 사용할 때는 먼저 Leader를 종료하고 다음처럼 다시 시작한다.

```bash
./runtime/hx5_isaac.sh leader-stop
./runtime/hx5_isaac.sh leader --mobile
```

`--mobile`의 초기 모드는 공식 joystick controller의 **swerve**다. 양쪽 tact 버튼을 함께 눌렀다 놓으면 arm_control↔swerve를 전환한다. Normalize/reverse 적용 후 base 명령은 `vx=-left_x/3`, `vy=left_y/3`, `wz=-right_y/2`이며 최대 약 `0.333 m/s`, `0.5 rad/s`다. 이 워크스페이스는 녹화 중 base를 잠그며, 저장/폐기 후 두 스틱을 중립으로 되돌려야 이동이 다시 허용된다.

## 녹화와 Mission Canvas로 전환

7880 UI에서 Record를 열고 네 camera와 Joint/Tactile topic 상태를 확인한 후 Task Num/Name/Instruction을 입력한다. `Record Start` 후 Leader로 시연하고 `Save Episode` 또는 `Discard Episode`로 끝낸다. 기존 단축 입력은 **왼쪽 tact 짧게: 시작/저장**, **오른쪽 tact 짧게: 폐기**다. 양쪽 tact 동시 입력은 모드 전환용이다.

저장 위치는 호스트의 `simulation/isaac/cyclo/rosbag2/`다. 기존 SH5 데이터 계약은 팔 `7+7`, 손 `20+20`의 **54개 action**이며, gripper 입력을 추가 action으로 저장하지 않는다. 관절 feedback은 `/arm_hand/joint_states` 54개, 네 camera의 원본 JPEG, 양손 tactile은 기존 Recorder 경로를 사용한다. Head/lift/mobile은 이 54-D policy action에 포함되지 않는다. Tactile을 policy observation에 합치는 변환 설정에서는 finger별 압력 feature가 별도로 추가될 수 있다.

Mission Canvas의 Rotate/Navigation/JointControl/Gate/추론으로 전환하거나 Isaac reset/stop을 하기 전에 녹화를 저장/폐기하고 스틱을 중립으로 되돌린 뒤 Leader를 종료한다.

```bash
./runtime/hx5_isaac.sh leader-stop
./runtime/hx5_isaac.sh reset --check
# 다음 episode를 초기 scene에서 시작할 때만:
./runtime/hx5_isaac.sh reset
```

공식 joystick controller는 arm_control 모드에서도 `/cmd_vel` zero를 계속 publish하며 head/lift target도 유지한다. 따라서 스틱 중립 또는 arm following off만으로 Canvas 제어와의 동시 publisher 충돌이 해소되지는 않는다. Leader가 실행 중이면 reset/stop helper가 거부한다.

## 공식 구현 근거와 검증 범위

물리 bringup은 [ROBOTIS LG2 official launch](https://github.com/ROBOTIS-GIT/ai_worker/blob/main/ffw_bringup/launch/ffw_lg2_leader_ai.launch.py)를 그대로 실행한다. [공식 1044 LG2 YAML](https://github.com/ROBOTIS-GIT/ai_worker/blob/a7e047128e730050c6bf21f9b74f6d73139c24bc/ffw_bringup/config/ffw_lg2_leader/ffw_lg2_leader_ai_hardware_controller.yaml)의 팔 7축+gripper, offset `0.3`, trigger `-0.5`/2초, joystick calibration과 spring controllers를 유지한다. Isaac private YAML은 공식 1044처럼 양팔 관절의 추가 부호 반전을 사용하지 않는다. 이전 SG2 공통 설정에서 들어오던 arm_joint2 추가 반전은 Isaac 설정 생성 시 해제하며 공통 소스·Gazebo 설정은 유지한다. Isaac private YAML만 joystick axes를 켜고 실제 exported follower 한계로 bounds를 만든다. 물리 controller는 wall time, follower/Recorder는 simulation time을 사용한다. 공식 trajectory header가 zero인 action은 Recorder 수신 시각으로 동기화한다.

손 pose와 보간은 [ROBOTIS feature-1044-koreamat의 preset_hand_controller.py](https://github.com/ROBOTIS-GIT/ai_worker/blob/a7e047128e730050c6bf21f9b74f6d73139c24bc/ffw_teleop/ffw_teleop/preset_hand_controller.py)를 commit `a7e047128e730050c6bf21f9b74f6d73139c24bc`에 고정했다. [DanielFH1 같은 브랜치](https://github.com/DanielFH1/ai_worker/tree/feature-1044-koreamat)도 점검 시 동일 commit이다. 좌우 release/grasp 80값, gripper normalize range `[-0.1,1.1]`, thumb threshold `0`을 그대로 재사용하고 실제 URDF 한계로 최종 clamp한다. LG2 trajectory를 팔 7개와 손 20개로 나누어 Isaac에 전달하며 Cyclo HandPreset service/status를 유지하는 연결부는 이 워크스페이스의 추가 구현이다. 공식 1044 source의 active subscription은 `/topic_based_joint_commands`이고, LG2 trajectory callback은 주석 예제로 존재하므로 공식 완제품 LG2→Isaac 지원이라고 표현하지 않는다.

LG2 출력 gripper 한계는 기존 `0..1.05`를 유지하므로 공식 normalize range에 대입하면 약 `0.0833..0.9583` curl이다. 물리 장치를 연결한 후 실제 개폐 방향과 endpoint calibration을 확인해야 한다. 검토한 공식 main/fork main에서는 별도 `ffw_lg2_head` launch를 찾지 못했으며, 실제 실행 이름은 `ffw_lg2_leader_ai.launch.py`다. Head/lift는 위 joystick 경로로 연결한다. [ROBOTIS 공식 Isaac SH5 teleoperation 글](https://docs.robotis.com/docs/systems/aiworker/resources/technical_story/isaac_vr_teleoperation/)은 SH5/HX5 외부 ROS trajectory 연결의 근거이며 VR 방식이다.

2026-09-18 최초 구현 검증에서는 `/dev/left_leader`, `/dev/right_leader`, `ttyUSB*`, `ttyACM*`가 없어서 물리 Leader를 실행하지 않았다. 당시 비구동 검사로 공식 endpoint/normalize/limit/preset 선택, 실제 8축 message callback→7+20 split, malformed 입력 차단, USB alias/중복 사용 차단, reset guard, 전용 Compose/domain/device 구성을 검증했다. 데이터 합치기는 실제 Cyclo converter의 관절 이름 필터를 사용해 54-D action임을 별도로 검증했다. 원본 값 비교 receipt는 `simulation/isaac/runtime/official_hand_mapping_review_20260918.json`이다.

합성 공식 LG2 message로 실제 Isaac 양팔·손 움직임과 Recorder MCAP·촉각·네 영상 저장을 확인했고, 기존 공식 Cyclo 변환기로 **LeRobot v3.0 65행×54축 state/action, 양손 각각 45-taxel, 네 최종 영상 각각 65frame**을 생성·검증했다. 원본 episode hash도 보존됐다. Isaac Recorder의 action/state/video 시간을 `/clock`으로 맞췄다. [초기값·Record 3D·저장 및 변환 검증](HX5_ISAAC_INITIAL_POSE_RECORD_GUIDE_20260918.md)에 측정값·실행 범위·시간 품질 경고를 기록했다.

## USB 연결 후 재현된 시작 오류와 수정

같은 shell에서 `leader --check`가 통과한 직후 `leader`가 `official 1044 hand mapping profile` 오류로 실패했다. 실제 ROS 상태를 읽어 손 상태와 각 손 명령에 publisher 5개가 존재하고, 이전 generic mapping과 공식 1044 mapping status가 섞여 있는 것을 확인했다. RSP 자식 프로세스도 여러 개 남아 있었다.

Isaac sidecar supervisor가 `ros2 run` 래퍼만 종료하여 실제 ROS 실행 파일이 남는 문제가 원인이었다. 자식마다 독립 process group을 만들고 그룹 전체에 종료 신호를 보내도록 수정했다. 실패한 spawn도 먼저 실행한 자손까지 정리하며, 종료하지 않는 그룹은 제한 시간 뒤 회수한다. 기존 잔여 프로세스는 Isaac 전용 컨테이너에서 실행 파일과 전용 params 경로가 정확히 일치하는 것만 정리했다. 이후 실제 sidecar 종료·재시작 시 잔여 ROS 자손이 없고 손 status/action publisher가 각각 하나인 것을 확인했다. 씬 위치·자세·Gazebo 프로필은 유지했다.

연결된 `/dev/left_leader → ttyUSB0`, `/dev/right_leader → ttyUSB1`로 수정된 `leader --check`와 `leader`를 연속 실행하여 통과했다. 공식 bringup에서 양쪽 hardware activation, 5개 controller의 active 상태, **16축 실제 JointState**의 유한값·신선도·timestamp 진행을 확인했다. 리더 시작 후 follower 상태·카메라·촉각·TF/odom 검사도 통과했다. 관련 회귀 검사 72개를 통과했다. 이 검사는 실제 USB 통신과 시작 준비 상태이며, 사용자의 trigger activation·수동 시연 결과를 대신하지 않는다.

현재 리더가 이미 실행 중이면 그대로 following을 켜서 사용한다. 재시작할 때는 다음 순서로 실행한다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh leader-stop
./runtime/hx5_isaac.sh leader --check
./runtime/hx5_isaac.sh leader
```

이번 복구·실제 USB 시작 증거는 `/home/robotis-ai/workspaces/isaac_logistics_cell_ws/reports/leader_usb_recovery_20260918/`에 보관했다. 수신 상태와 중복 검사에는 [ROS 2 parameter service](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html), [공식 Transient Local QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html), [공식 rmw_zenoh 설계](https://github.com/ros2/rmw_zenoh/blob/jazzy/docs/design.md)를 따른다. 로보티즈의 물리 LG2 bringup·controller 동작은 재사용하고, 수정한 종료·준비 검사는 이 workspace의 Isaac adapter 구현이다.

## 리더암과 Isaac 움직임 차이 수정

실제 세션 로그에는 following activation 후 약 60초 동안 최초 동기화 완료가 없었다. Isaac adapter는 매 프레임 새 trajectory를 현재 simulation time에서 만들고 동일한 시각에 샘플링했다. 빠른 연속 입력에서는 항상 시작 자세만 drive에 적용되어 팔과 손이 목표를 따라가지 못했다. 이제 다음 물리 계산 시점(`sim_time + 1/120 s`)의 drive 목표를 샘플링한다.

활성 trajectory를 교체할 때는 해당 그룹에서 이미 제어하던 관절의 마지막 drive 목표를 이어받는다. 첫 명령·새 관절·hold/reset/Pause/STOP 이후 명령은 실측값에서 시작한다. 이 동작은 [ROS 2 Jazzy joint_trajectory_controller의 interpolate_from_desired_state](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html)에 근거한다. 실제 `/joint_states`, 촉각, 카메라 feedback은 계속 PhysX·렌더링에서 읽으며 목표값으로 대체하지 않는다. 시간 보간·공식 SH5 한계·초기 정렬의 adaptive delay를 유지한다.

공식 1044 및 DanielFH1 동일 브랜치의 broadcaster는 같은 이름의 arm joint 값을 전달한다. 양쪽 shoulder motor에는 이미 `Drive Mode=1`이 설정되어 있고 공식 broadcaster YAML에는 추가 reverse 목록이 없다. Isaac private 생성기에서 기존 SG2 추가 반전을 해제했다. LG2의 shoulder 모델 한계는 양쪽 `[-6.28, 6.28]`이고 SH5의 한계는 왼쪽 `[0, 3.14]`, 오른쪽 `[-3.14, 0]`이다. LG2 입력이 SH5 한계를 벗어나면 포화된다. 복구 시 읽은 왼쪽 `-0.1902`, 오른쪽 `+0.2730`의 정지 값은 두 SH5 target이 `0`으로 포화되는 경우다. 정지 값만으로 실제 장착 방향·영점 이상을 판정하지 않으며 EEPROM/HomingOffset은 변경하지 않았다. 물리 방향의 마지막 검증은 알려진 개별 어깨 동작으로 확인한다.

같은 세션에서 USB hub가 끊기며 두 장치가 `ttyUSB0/1`에서 `ttyUSB2/3`으로 다시 열렸다. ROS control은 통신 실패로 양쪽 hardware와 controller를 비활성화했다. Docker 장치 바인딩은 이전 장치를 계속 가리킬 수 있으므로 재연결 후 컨테이너를 새로 만든다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh leader-stop
./runtime/hx5_isaac.sh leader --check
./runtime/hx5_isaac.sh leader
```

Isaac GUI의 STOP은 PhysX articulation handle을 해제하므로 기존 코드가 `None` 관절 배열을 읽다가 종료됐다. 이제 STOP/Pause 동안 제어·feedback·clock 진행을 중지하고 대기 명령을 제거한다. Pause→Play는 현재 자세를 유지하고, STOP→사용자의 Play는 [NVIDIA World.reset 및 articulation lifecycle](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/deprecated/isaacsim.core.api/docs/index.html)에 따라 초기 씬과 physics/contact view를 다시 준비한다. STOP은 자동으로 Play되지 않는다. 녹화 중 GUI STOP/reset은 시간 흐름을 끊으므로 episode를 저장한 뒤 사용한다.

자동 검증은 임시 native Isaac 앱에서 수행한다. 원본 USD·초기 자세 파일·사용자 episode는 수정하지 않으며, 다음 명령은 실행 중인 GUI와 물리 리더를 종료한 후 사용한다.

```bash
./runtime/hx5_isaac/run_isaac.sh src/hx5_isaac/tests/timeline_smoke.py \
  --headless \
  --stage /home/robotis-ai/workspaces/isaac_logistics_cell_ws/logistics_cell.usda \
  --metadata /home/robotis-ai/workspaces/isaac_logistics_cell_ws/scene_metadata.json \
  --summary /tmp/hx5-leader-timeline-new-run.json
```

네 그룹의 모든 54관절 목표를 매 물리 프레임 교체하고, 양팔 wrist joint7 및 양손 finger6의 작은 `0.08 rad` 목표 이동을 실제 feedback으로 검사한다. 초기 자세·씬 파일 hash, 네 카메라의 공식 해상도/K/장착 위치, 63축·10개 tactile view·72개 물체의 STOP→Play 복구도 검사한다. 공식 원본 비교와 실제 실행 증거는 `/home/robotis-ai/workspaces/isaac_logistics_cell_ws/reports/leader_motion_fix_20260918/`에 보관한다. 이 자동 시험은 물리 리더암을 사람이 움직이며 확인한 최종 장착·영점 검증을 대신하지 않는다.

최종 회귀 검사 **120개**와 native 검증이 통과했다. 네 그룹의 최대 목표 오차는 왼팔 `0.000553 rad`, 오른팔 `0.000979 rad`, 왼손 `0.003346 rad`, 오른손 `0.002271 rad`였다. 300개의 연속 교체 frame, 총 1,200개 trajectory 명령에서 작은 목표 이동을 실제 측정으로 확인했다. Lifecycle 검사를 포함한 841개 physics frame의 tactile sample 8,410개는 모두 valid였다. 네 카메라와 두 번의 STOP→Play 복구도 통과했으며 초기 pose·layout·USD hash는 변경되지 않았다.

수정된 GUI 실행 뒤 `leader --check`와 실제 `leader`를 검증했다. 현재 `/dev/left_leader → ttyUSB2`, `/dev/right_leader → ttyUSB3`의 host/container character-device 번호가 일치한다. 5개 공식 controller가 모두 active이고 16축 실제 feedback은 약 100 Hz로 진행된다. 실행 중 broadcaster의 ROS parameter를 읽어 추가 reverse가 없는 typed default와 7축 offset `0`·gripper offset `0.3`, follower bounds를 확인했다. ROS는 YAML의 빈 배열 `[]`을 타입 없는 값으로 해석할 수 있으므로 private 생성기는 reverse key를 **공식 YAML처럼 생략**하고 plugin의 typed default를 사용한다.

Following은 공식 startup 기본값 STOPPED다. 양팔을 준비한 뒤 두 gripper trigger를 약 2초 눌러 사용자가 활성화한다. `http://localhost:7880/`에서 수동 팔 동작·손 개폐를 확인한다. SH5의 shoulder 한계를 넘는 LG2 동작은 포화되며, 이 경우 임의의 부호 반전이나 한계 확장을 하지 않는다. 최종 결과와 실제 USB·ROS·공식 원본은 report 폴더의 `review.json`과 `official_sources/source_sha_receipt.json`에서 확인할 수 있다.
