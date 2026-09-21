# 현재 초기 자세와 Record 3D 검증 · 2026-09-18

워크스페이스 `/home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws`의 Isaac 전용 설정이다. 물류 씬은 `/home/robotis-ai/workspaces/isaac_logistics_cell_ws`이며 Cyclo UI는 **http://localhost:7880/** 이다. 기존 Gazebo domain 105, UI 7380과 고정 ViTacFormer 정렬 참조는 유지했다.

## 저장한 현재 초기값

사용자 요청 직후 Isaac `/joint_states`와 `/odom`을 읽어, 당시의 odometry origin으로 world 위치와 방향을 복원했다. 관절은 가져온 공식 URDF의 한계를 검사했다. 팔 14축·양손 40축·Head 2축·Lift 1축의 **57개 실측값**을 `simulation/isaac/config/initial_pose.yaml`에 저장했다. wheel의 누적 회전은 자세 목표에 포함하지 않는다.

| 항목 | 시작 목표 |
|---|---:|
| world X | `0.3602113128 m` |
| world Y | `-0.7280516624 m` |
| world Z | `0.0000001490 m` |
| world yaw | `90.0636840638°` |
| Head pitch (`head_joint1`) | `0.6863727570 rad` |
| Head yaw (`head_joint2`) | `0.0000118094 rad` |
| Lift (`lift_joint`) | `-0.0055988370 m` |

작은 roll/pitch까지 포함한 실측 quaternion을 WXYZ로 저장했다. `config/layout.json`의 로봇 transform과 `initial_pose_source`를 업데이트했고 `build.sh`가 이 Isaac 전용 YAML을 사용한다. 일반 Isaac 실행, 물류 workspace의 `launch.sh`, 실행 중 `reset`이 같은 초기값을 복원한다. 물체·바구니·컨베이어 배치는 유지했다.

물리 drive가 작동하므로 중력 하의 정지 실측값에는 작은 추종 오차가 있다. 새 씬을 실제로 실행하여 57축 모두 저장 목표와 비교해 `0.01 rad/m` 이내였고, 15종 상태·카메라·촉각·scan·clock topic과 TF/odom 일치를 확인했다. 정지 상태의 base는 저장 spawn에 대해 약 `1.35 mm` 차이였다.

측정 원본은 `simulation/isaac/ai_worker/diagnostics/requested_initial_pose_20260918.json`이다. 현재 원점으로 과거 odom을 이중 변환하지 않도록 원본에 당시 `odometry_origin_spawn`을 저장한다. 캡처 helper는 두 출력을 준비한 뒤 적용하며, 중간 실패 시 이전 설정을 복원한다.

이전 씬·배치·빌드 경로와 파일 hash는 물류 workspace의 `revisions/20260918_before_current_initial_pose/`에 보관했다. 새 초기값을 변경하기 위해 모델 추론이나 task를 실행할 필요는 없다.

## Record 3D의 Head/Lift 표시

원인은 3D Viewer에 제공한 상태 topic이 정책용 `/arm_hand/joint_states`의 54축뿐이었다는 것이다. 여기에 Head/Lift가 없어서 URDF의 기본 각도로 표시되었다. `GetRobotInfo.state_joint_topics`에 시각화에 필요한 `/joint_states`를 추가했다. 실제 measured feedback만 표시하며 action command로 자세를 덮어쓰지 않는다. 정책의 state/action 54차원은 그대로 유지한다.

Viewer URDF도 공식 `ai_worker` SH5 follower와 `robotis_hand` HX5 D20 Rev2의 관절 origin·axis·limit에 맞췄다. Head의 +Y 축에 대한 양의 pitch가 아래를 향하므로 부호를 반대로 바꾸는 보정은 사용하지 않는다.

독립 브라우저에서 실제 Record 페이지와 Three URDF 객체를 확인했다. 두 상태 topic의 실제 WebSocket 구독이 존재하고 Head는 아래를 향했다. 표시 Head pitch `0.6845100522 rad`, yaw `0.0000234033 rad`, Lift `-0.0083984919 m`이었다. fresh ROS 실측 대비 바퀴 누적 회전을 제외한 최대 표시 차이는 `4.24e-6`이었다. 스크린샷과 전체 관절 비교는 물류 workspace `reports/initial_pose_viewer/record-3d.png`, `record-3d-review.json`, `record-3d-subscriptions.json`에 저장했다.

7880 페이지를 새로고침하고 Home에서 **FFW SH5 Rev1**을 선택하면 Record에서도 같은 피드백을 사용한다.

## 스켈레톤 리더와 데이터 수집

[스켈레톤 리더 실행·입력·녹화 안내](HX5_ISAAC_SKELETON_LEADER_GUIDE_20260918.md)를 따른다. 물리 LG2의 공식 bringup을 domain 115로 실행하고, 각 팔의 7관절과 gripper 1입력을 SH5 팔 7관절 및 HX5 손 20관절로 분리한다. 손 release/grasp와 보간은 공식 1044 브랜치의 고정 commit을 사용한다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh leader --check
./runtime/hx5_isaac.sh leader
# 7880 Record에서 Task 입력 → Record Start → 시연 → Save Episode
# following 활성화: 양쪽 gripper trigger를 약 2초 누른다.
./runtime/hx5_isaac.sh leader-stop
```

최초 구현 검증에서는 두 리더 USB가 없어 물리 리더를 시작하지 않았다. 이후 USB 연결 후의 중복 노드 수정과 실제 16축 feedback·5개 active controller 검증은 [스켈레톤 리더 가이드의 USB 연결 후 수정](HX5_ISAAC_SKELETON_LEADER_GUIDE_20260918.md#usb-연결-후-재현된-시작-오류와-수정)에 기록했다. 아래의 합성 입력 데이터 수집 검증은 물리 리더의 수동 시연 검증과 구분한다.

Recorder의 기존 RMW source timestamp는 wall time이고, Isaac state와 영상 timestamp는 simulation time이었다. action/state/video 정렬을 위해 공식 rosbag2의 `--use-sim-time` 동작에 맞춰 `record_with_ros_clock=true`를 Isaac launch에서만 켠다. 다른 프로필의 기본값은 유지한다. 새 recorder는 Isaac 전용 `/workspace/ros_transport` overlay에 빌드되며, `prepare_transport.sh`가 소스 hash로 준비 상태를 확인한다.

실제 합성 LG2 입력 → mapper → Isaac PhysX → Cyclo 녹화 검사는 통과했다. 공식 리더와 같이 0인 arm header로 입력했으며 양팔 joint7 실측 변화는 각각 약 `0.02288`, `0.02233 rad`였다. 양손 손가락도 움직였고 MCAP의 팔·손 action은 각각 7·7·20·20으로 저장됐다. state와 action은 54축 계약을 사용한다. MCAP log와 센서 header의 최대 시간 차이는 `8.334 ms`였다.

네 MP4가 각각 69frame으로 decode됐으며 timestamp Parquet 행 수와 일치했다. Head 영상은 `672×376`, wrist 원본은 `424×240`이다. 기존 공식 Cyclo의 wrist 90° recording rotation으로 wrist MP4 크기는 `240×424`다. 네 CameraInfo도 원본 해상도를 유지했다. 녹화 후 Recorder READY와 로봇·물체 reset 완료 응답을 확인했다.

검사용 episode는 `simulation/isaac/cyclo/rosbag2/Task_20260918_IsaacSkeletonContract20260918_MCAP/0`이다. 합성 검사라는 task instruction/tag와 `hardware_calibrated=false`를 기록했다. 보고서는 `simulation/isaac/cyclo/diagnostics/leader-recording-smoke.json`에 있다. 기존 episode는 수정하지 않았다.

이 episode를 기존 공식 Cyclo 변환기로 실제 LeRobot v3.0 데이터셋까지 변환했다. 출력 `simulation/isaac/cyclo/conversion-probes/IsaacSkeletonContract20260918-full-v30/lerobot_v30`의 Parquet에서 **65행×54축 state/action**, 각 손의 **65행×45 raw tactile/baseline**을 확인했다. 네 최종 MP4도 모두 65frame으로 decode됐고, data row와 camera feature metadata에 일치했다. Episode/task/subtask/statistics metadata도 생성됐다. 원본 episode의 모든 파일 hash는 변환 전후 동일했다. 업로드나 학습은 실행하지 않았다.

영상은 동일 SIM grid에 과거 frame을 선택하여 정렬했다. 네 카메라가 각각 65개의 서로 다른 frame을 사용했고, 반복·시작/끝 clamp는 없었다. 영상의 최대 과거 frame 나이는 `25 ms`, state/tactile은 `8.34 ms` 이내였다. 합성 입력을 중단하고 녹화를 끝낸 마지막 두 행에는 action 나이 `100/133.3 ms` 경고가 있다. 실제 리더 수집에서도 저장 시작/끝의 입력 신선도와 변환 quality report를 확인한다. 변환 결과와 품질 보고서는 `simulation/isaac/cyclo/diagnostics/leader-conversion-probe-full-v30-verified.json`에 보관했다.

## 공식 구현과 문서

- [ROBOTIS 스켈레톤 Teleoperation](https://ai.robotis.com/ai_worker/operation_teleoperation_ai_worker): activation trigger·joystick·tact 동작.
- [ROBOTIS LG2 bringup](https://github.com/ROBOTIS-GIT/ai_worker/blob/main/ffw_bringup/launch/ffw_lg2_leader_ai.launch.py): 물리 리더 실행 기반.
- [ROBOTIS 1044 손 구현](https://github.com/ROBOTIS-GIT/ai_worker/blob/a7e047128e730050c6bf21f9b74f6d73139c24bc/ffw_teleop/ffw_teleop/preset_hand_controller.py): 좌우 release/grasp 80개 값과 normalize·보간 기준. DanielFH1 동일 브랜치도 같은 commit이었다.
- [ROBOTIS SH5 URDF](https://github.com/ROBOTIS-GIT/ai_worker/blob/main/ffw_description/urdf/ffw_sh5_rev1_follower/ffw_sh5_follower.urdf), [HX5 Rev2 URDF](https://github.com/ROBOTIS-GIT/robotis_hand/tree/main/robotis_hand_description/urdf/hx5_d20_rev2): 관절 이름·기구 transform·한계 기준.
- [ROBOTIS Isaac SH5 Teleoperation](https://docs.robotis.com/docs/systems/aiworker/resources/technical_story/isaac_vr_teleoperation/): 외부 ROS 명령을 SH5/HX5 Isaac articulation에 연결하는 근거.
- [ROS 2 rosbag2 Jazzy simulation time](https://github.com/ros2/rosbag2/blob/jazzy/README.md#simulation-time): 저장 timestamp를 `/clock`에 맞추고 첫 clock 이전 기록을 기다리는 기준.

공식 물리 리더 실행·기구·메시지·손 목표를 재사용하되, Zenoh domain 115 연결과 Isaac 명령 분리·초기값 저장·시뮬레이션 recorder 설정은 이 workspace의 추가 구현이다.
