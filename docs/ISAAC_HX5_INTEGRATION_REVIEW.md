# HX5 · Cyclo · Mission Canvas의 Isaac Sim 연동 검토

상태: 2026-09-16 기준 검토 문서. **새 HX5 Isaac 프로필은 아직 구현·실행하지 않았습니다.**

## 결론

연동할 수 있는 구조입니다. Cyclo/Mission Canvas는 Gazebo 화면 자체가 아니라 ROS 2 상태·명령·센서·Navigation 서비스를 이용합니다. Isaac Sim 쪽에서 같은 인터페이스를 제공하면 UI와 데이터 수집, Mission/Action Canvas 코드를 대부분 유지할 수 있습니다. 단순히 Gazebo 실행 명령을 Isaac 명령으로 바꾸는 것만으로 완성되지는 않습니다.

## 이 PC에서 확인한 기존 작업

- `/home/robotis-ai/isaac_sim/app/6.1.0/VERSION`: `6.1.0-rc.26+release.49347.2d230af4.gl`.
- `/home/robotis-ai/isaac_sim/integration_ws/README.md`: 기존 **SG2** ROS/카메라/LiDAR/Mission 연동 작업이 있음.
- `integration_ws/scripts/build_ffw_scene.py`: Lift를 비활성화하고 fixed joint로 교체하는 코드가 있음. HX5용으로 그대로 가져오면 Lift 조절 요구를 충족하지 못함.
- `integration_ws/src/ffw_isaac_sim/ffw_isaac_sim/command_adapter.py`: 궤적 마지막 점을 단순 JointState 목표로 변환함. 완전한 trajectory 시간 처리/ros2_control 인터페이스를 구현한 것과 다름.
- 기존 시작 스크립트의 기본 ROS domain 73, Zenoh 7447, UI 7180은 이전 Gazebo 환경과 겹침.

위 외부 경로는 이번에 **읽기만 했으며 수정하거나 실행하지 않았습니다.** 기존 SG2 작업을 HX5 완성본 또는 모든 기능이 검증된 것으로 간주하지 않습니다.

## 권장 구성

```text
Cyclo UI / Mission·Action Canvas / 데이터 수집
                     ↕ ROS 2 Jazzy + Zenoh
Task Engine + Nav2/AMCL/SLAM + 기존 관절 제어기
                     ↕
Isaac ROS 2 Bridge / ros2_control 하드웨어 연결
                     ↕
Isaac: AI Worker + HX5 USD, 카메라, LiDAR, 물체, 접촉 물리
```

NVIDIA 공식 ROS 2 bridge는 Linux/Jazzy의 Zenoh 연동을 설명합니다. Isaac은 Zenoh 라이브러리를 자체 포함하지 않는 구성도 있으므로 선택한 실행 패키지와 ROS/Python ABI에 맞춰 준비해야 합니다. [공식 ROS 2 설치](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros.html)

가능하면 공식 `isaacsim.ros2.control`로 원래 joint trajectory controller/명령 토픽/액션을 유지합니다. AI Worker의 사용자 정의 swerve controller도 해당 ROS 환경에서 빌드·로드 가능한지 확인해야 합니다. [공식 ros2_control](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/ros2_tutorials/robot_control/tutorial_ros2_control.html)

## 반드시 연결할 항목

| 기능 | Isaac 쪽에서 필요한 작업 |
|---|---|
| Mapping/지도 저장/Localization/Nav2 | `/scan`, `/odom`, `/tf`, `/tf_static`, `/clock`, `/cmd_vel`; 기존 SLAM/AMCL/Nav2와 시뮬레이션 시계 사용 |
| Mobile Teleop/횡이동 | AI Worker의 3개 swerve 모듈 구동·조향; differential-drive 예제를 그대로 쓰지 않음 |
| Head/Lift/양팔/양손 | 실제 가동 관절/drive/관절명/단위/한계, 원래 JointTrajectory 토픽 및 필요한 FollowJointTrajectory 액션 |
| Capture Pose/ArmStateGate | 실제 측정 `/joint_states`, 팔 14+손 40 순서의 `/arm_hand/joint_states`; 명령값을 측정값으로 대신 발행하지 않음 |
| 영상/데이터 수집 | SH5 설정과 동일한 4개 JPEG CompressedImage 토픽, camera_info, 명령/상태/센서 MCAP |
| 손 Preset/LG2 | 시뮬레이션 Preset 서비스 계약과 LG2 gripper→20관절 변환 재사용; USB 장치는 한 환경만 점유 |
| 촉각 | 손가락 끝 접촉 위치·힘을 5×9셀/손으로 변환해 `robotis_interfaces/msg/HandPressures` 발행 |
| 초기화 | Isaac 전용 로봇·물체 상태 복원; Gazebo set_pose 서비스나 SG2 reset 스크립트를 호출하지 않음 |

Nav2 연동 자체는 NVIDIA 공식 예제가 제공하지만 AI Worker의 frame·footprint·swerve 설정은 별도 이식 대상입니다. 지도 생성 방식을 바꿀 필요 없이 현재처럼 센서를 읽는 SLAM 경로를 유지할 수 있습니다. [공식 ROS/Navigation 구성](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/ros2_tutorials/tutorial_series/tutorial_ros2_putting_it_all_together.html)

## HX5 촉각의 범위

Isaac Contact Sensor는 접촉 여부·힘과 원시 contact 정보를 제공하므로 현재와 같은 가상 taxel 어댑터를 만들 수 있습니다. 원시 impulse를 사용할 경우 물리 시간 간격으로 힘을 환산하고, 유효하지 않은 센서 값을 새 관측으로 재사용하지 않아야 합니다. [공식 Contact Sensor API](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.sensors.experimental.physics/docs/index.html)

이것도 실물 HX5의 보정된 압력 센서/고무 변형/노이즈/미끄러짐을 자동 재현하는 것은 아닙니다. URDF→USD 가져오기 이후 관성·충돌 형상·마찰·drive gain을 파지 물체에 맞게 검증해야 합니다. Gazebo 전용 hard-stop 수치 여유를 Isaac에 무조건 복사하지 않습니다.

## 격리와 진행 순서

1. 새 워크스페이스 아래 Isaac 전용 소스/설정/자산/저장 경로를 마련합니다. 기존 공유 SG2 소스를 런타임 의존성으로 연결하지 않습니다.
2. Gazebo와 다른 ROS domain, Zenoh router, 컨테이너 이름, UI 포트를 사용합니다. GPU/CPU 자원 공유까지 사라지는 것은 아닙니다.
3. HX5 USD와 모든 가동 관절부터 확인하고, 센서·시계·TF를 연결합니다.
4. Mapping→Save→Localization→Waypoint→Task 완료를 검증합니다.
5. 촉각과 리더 데이터 수집, 영상/MCAP 동기화와 초기화를 검증합니다.
6. Isaac MCAP는 예를 들어 `schema_version=cyclo_isaac_mcap`로 별도 구분합니다. 이는 제안이며 기존 `cyclo_gazebo_mcap` 정책을 변경하지 않습니다.

모델 학습·추론용 SendCommand는 시뮬레이터 연결과 별개로 정책 백엔드/checkpoint가 있어야 동작합니다. 전체 Mission Canvas 지원 가능성과 모든 실행 조합이 이미 검증되었다는 주장은 구분합니다.
