# SH5/HX5 Isaac 카메라 시야와 적재 물체 비산 수정

## 카메라 화면의 의미

Cyclo의 Head L/R 및 Wrist L/R 네 영상 구성은 실제 AI Worker와 같습니다. Head는 ZED Mini 스테레오이고 Wrist는 좌우 D405입니다. 공식 SH5 URDF의 Head optical frame 간격은 63 mm이므로 먼 물체를 보는 좌우 화면은 비슷할 수 있습니다. UI는 영상 비율을 유지하여 카드의 좌우에 빈 공간을 표시합니다. [ROBOTIS 카메라 구성](https://ai.robotis.com/ai_worker/hardware_ai_worker)

제보 이미지의 원형 회색 테두리와 검은 가림은 **머리 CAD의 불투명한 visual mesh가 시뮬레이션 카메라를 가린 것**입니다. Cyclo가 원형으로 crop한 영상이 아닙니다. 카메라는 공식 optical frame에 있지만 CAD는 투명 렌즈와 그 광학 통로를 표현하지 않습니다.

카메라 위치·방향·63 mm 간격을 유지하고, 생성하는 USD overlay에서 머리 visual mesh의 두 렌즈 시야에 광학 통로를 만들었습니다. 원본 CAD, 로봇 collision mesh, 관절과 다른 링크를 수정하지 않습니다. 원본 머리 삼각형 8114개 중 716개가 통로와 교차하여 해당 부분만 잘랐습니다. 전체 씬의 로봇 실루엣과 나머지 표면은 유지합니다. 이는 상세 렌즈 CAD를 대신하는 시뮬레이션 근사입니다.

수정 전:

![수정 전 Head L](/home/robotis-ai/workspaces/isaac_logistics_cell_ws/reports/camera_review/baseline-head_left.png)

수정 후:

![수정 후 Head L](/home/robotis-ai/workspaces/isaac_logistics_cell_ws/reports/camera_review/aperture-head_left.png)

실제 RTX 렌더 검사에서 두 Head 영상의 가림 제거와 Overview의 로봇 외형을 확인했습니다. collision prim, 카메라 local 위치, 원본 stage 파일 보존도 확인했습니다. `purpose=proxy` 및 RenderProduct `includedPurposes` 조합은 실제 renderer에서 효과가 없어 적용하지 않았습니다. 검사 증거는 물류 워크스페이스 `reports/camera_review/receipt.json`과 같은 폴더의 Before/After PNG입니다. [NVIDIA 카메라 설명](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_camera.html)

이 검사를 수행할 당시 336×188 해상도와 focal length/aperture는 기존 시뮬레이션 설정이었습니다. 이후 해상도·모델별 공칭 intrinsics·JPEG 설정은 [카메라 화질 적용 기록](HX5_ISAAC_CAMERA_QUALITY_20260918.md)에 따라 변경했습니다. 실제 장비의 CameraInfo로 보정한 영상과 완전히 같다고 판정하지 않습니다. UI의 약 4 Hz 표시는 현재 수신률이며, 실제 ZED/D405 장비의 최대 FPS를 나타내지 않습니다. 생성 URDF의 카메라 CAD 경로 일부가 상대 경로여서 현재 import 자산의 일부 카메라 visual이 비어 있는 점도 관찰했습니다. 이번 원형 가림의 직접 원인은 실제 렌더되는 `head_link2` visual mesh이며, importer 자산 전체를 재생성한 검사는 아닙니다.

## object2 비산 원인과 수정

저장된 적재 자세는 단일 convex hull 물체와 열린 5개 박스 바구니로 안정화하여 만들었습니다. 실행 씬은 물체를 convex decomposition으로 다시 계산하고 바구니에는 거친 scan 삼각형 충돌을 사용했습니다. 일부 물체와 scan 바구니 충돌면은 시작부터 겹쳤으며, 과거의 정적 적재 검증은 물체·바구니 교차를 검사하지 않았습니다.

수정 전 실제 native PhysX 10초 검사에서 object2의 최대 상승 속도는 **40.03 m/s**, 최대 상승량은 **3.64 m**, 바구니 이탈은 **23/32개**였습니다. 초기 collider 겹침이 강한 반발력을 만드는 것은 공식 PhysX 문서의 설명과 일치합니다. [NVIDIA Collision Behavior Guide](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/guides/collision_tuning.html)

실행 충돌 모델을 적재 생성 방식에 맞췄습니다.

- 물체는 단일 `convexHull`, hull vertex limit 256입니다. 72개 모두 dynamic이고 잡거나 밀 수 있습니다.
- 바구니 Scan은 보이는 외형을 유지하고, 충돌은 열린 5개 박스 proxy를 사용합니다. packing의 내부 half width/depth 0.143/0.225 m와 floor 0.032 m를 그대로 사용합니다.
- 물체 solver는 position/velocity 32/4, damping은 0.3/0.5, CCD를 켜고 overlap 회복 속도를 0.2 m/s로 제한합니다. 정상 운동 속도를 강제로 잘라내는 처리는 없습니다.
- 물체·바구니 contact/rest offset은 적재 생성과 같은 0.003/0.0015 m입니다. TGS와 pile stabilization을 실행에도 적용합니다.
- Isaac `World()`의 기본 설정과 legacy manager 호출만으로는 startup/reset 뒤 stage의 stabilization이 꺼져 있었습니다. 실제 PhysicsScene을 대상으로 공식 `PhysxScene.set_enabled_stabilization(True)`를 startup/reset 전후에 적용합니다. 두 과정 모두 실제 flag가 True인 것을 검사했습니다. SH5 articulation은 threshold 0으로 제외하여 로봇에 pile stabilization을 적용하지 않습니다.

접촉 안정화와 관절 로봇의 stabilization 제외는 [공식 PhysX Best Practices](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/BestPractices.html)에 근거합니다. proxy 형상, 질량, 마찰과 damping은 시뮬레이션 가정이며 실제 포장재의 변형·마찰을 보정한 값은 아닙니다.

컨베이어·바구니·물체의 크기와 저장 시작 배치, 로봇 spawn, frozen 57축 목표를 유지합니다. `layout.json`, `settled_piles.json`, 환경 USD 및 원본 CAD를 수정하지 않습니다. 수정 전 씬과 설정은 물류 워크스페이스 `revisions/20260918_before_camera_pile_fix/`에 보존했습니다.

## 재실행과 검증

기존 CLI를 사용합니다. Cyclo는 Isaac용 **7880**입니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh isaac
```

코드 변경을 새 씬에 재생성할 때는 물류 워크스페이스의 `build.sh`를 사용합니다. 광학 통로와 충돌 proxy가 매번 같은 builder에서 생성됩니다.

```bash
/home/robotis-ai/workspaces/isaac_logistics_cell_ws/build.sh
```

`src/hx5_isaac/tests/pile_stability.py`는 외부 로봇 명령 없이 native 물체 pose/velocity를 매 physics frame 측정합니다. startup과 reset을 구분하여 상승량·상승 속도·바구니 이탈·마지막 2초 drift와 종료 속도를 검사합니다. 기본 판정은 최대 상승 0.25 m 미만, 상승 속도 2 m/s 미만, 이탈 0개, 마지막 drift 0.02 m 미만, 종료 속도 0.05 m/s 미만입니다. 물체의 미세한 초기 중력 정착은 허용합니다.

최종 씬에서 startup과 reset을 각각 **10초 검사하여 모두 통과**했습니다. object2 최대 상승량은 startup **7.13 mm**, reset **9.47 mm**였으며, **이탈은 두 검사 모두 0/32개**였습니다. object1도 이탈 0/40개입니다. reset 뒤 마지막 2초 위치 drift는 양쪽 종류 모두 0이었고, 종료 시 object2 최대 속도는 0.00760 m/s였습니다. 모든 관절 로봇을 freeze하거나 물체 운동을 강제로 제거하지 않았습니다. 실제 flag는 두 과정 모두 True, 로봇 articulation stabilization threshold는 0입니다. 최종 증거는 물류 워크스페이스 `reports/pile_stability_final.json`, 수정 전은 `reports/pile_stability_before.json`입니다.

구조 검사는 **636/636 통과**했고, Isaac의 실제 USD/PhysxSchema를 사용한 scene·payload·camera 검사는 **30개 통과**했습니다. host의 관련 Python 검사는 **54개 통과**, USD 없는 host에서 10개 생략됐습니다. 생략된 payload USD 검사와 기존 scene USD 검사는 위 30개에서 실제 provider로 검증했습니다. 별도의 비교에서 로봇 collider Mesh 64개의 정점 628,352개와 face 1,267,806개 및 local transform·활성 상태·offset이 수정 전과 동일함을 확인했습니다. 원본 URDF, 환경 USD, saved pile, layout도 유지했습니다.

수정 씬으로 실제 Isaac GUI를 재실행하여 Cyclo의 `/snapshot`에서 두 Head JPEG를 HTTP 200으로 수신했습니다. live 이미지는 물류 워크스페이스 `reports/camera_review/live-head_left.jpg`, `live-head_right.jpg`에 있습니다. 재실행 뒤 6초 ROS 진단은 **15개 topic, 기본 57축 자세 오차 0.01 이내, 관절·촉각·네 JPEG와 CameraInfo·scan·clock/odom/TF 검사 모두 통과**했습니다. clock/odom publisher는 각각 하나, 동일 stamp의 odom/TF 오차는 0입니다. 보고서는 `simulation/isaac/ai_worker/diagnostics/isaac-camera-pile-final-contract.json`입니다. 현재 Isaac GUI와 7880 Cyclo는 실행 중이며 Gazebo는 유지했습니다.
