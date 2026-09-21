# Isaac SH5/HX5 기본 자세의 native PD 검토 (2026-09-18)

SH5/HX5의 57개 자세 관절을 실제 PhysX 상태로 검사했을 때, 왼쪽 엄지 `finger_l_joint1`의 목표 대비 약 0.01354 rad 오차가 기본 StateGate 허용값 0.01 rad를 초과했다. Native controller의 적용 목표는 초기 자세와 일치했으며, 외부 접촉은 없었다. 목표를 다시 쓰거나 피드백을 보정하는 방법 대신 native drive와 solver 설정을 비교했다.

## 검사 방법

- `src/hx5_isaac/tests/pd_sweep.py`를 단일 headless Isaac 6.1 프로세스로 실행했다. 각 조건에서 로봇과 동적 물체를 초기화하고 1/120 s 간격으로 120 physics frame을 진행했다.
- 마지막 60 frame의 실제 57개 joint position과 velocity를 수집했다. 아래 position 오차는 해당 구간의 최대 절댓값이다.
- 원본 stage를 `/tmp`로 복사하고 설정을 session layer 및 native controller에만 적용했다. 원본이나 사용자 stage를 저장하지 않았다.
- 모든 조건에서 position iteration은 64였다. 외부 접촉의 실제 taxel force 최댓값은 모두 0 N이었다.
- 15개 조건 모두 완료됐으며 프로세스 종료 코드는 0이다. 원본 stage SHA256은 실행 전후 동일했다: `3224c3c3d702b67cb7d477b7816ea34e569c8fde8fcb7ed46f3ef0bd587a0d54`.

## 측정 결과

Kp/Kd는 USD angular drive 단위다. Native tensor의 값은 설치된 Isaac의 단위 변환으로 `180/pi` 배이며, 320/8 설정에서 native Kp는 18334.6504, Kd는 458.3662였다.

| Finger Kp | Finger Kd | Velocity iterations | External forces every iteration | 왼쪽 엄지 최대 position 오차 (rad) | 왼쪽 엄지 velocity RMS (rad/s) |
|---:|---:|---:|:---:|---:|---:|
| 320 | 32 | 8 | false | 0.013541251 | 0.055923937 |
| 320 | 8 | 8 | false | 0.003365636 | 0.055017470 |
| 320 | 2 | 8 | false | 0.000806063 | 0.048648685 |
| 320 | 32 | 4 | false | 0.013541400 | 0.055583461 |
| **320** | **8** | **4** | **false** | **0.003365844** | **0.054971080** |
| 320 | 2 | 4 | false | 0.000806183 | 0.048898559 |
| 320 | 32 | 1 | false | 0.013540328 | 0.055079408 |
| 320 | 8 | 1 | false | 0.003365368 | 0.054230005 |
| 320 | 2 | 1 | false | 0.000806093 | 0.048053197 |
| 80 | 8 | 1 | false | 0.013541669 | 0.055549737 |
| 160 | 16 | 1 | false | 0.013540328 | 0.054913987 |
| 320 | 32 | 0 | false | 0.013540328 | 0.055079408 |
| 320 | 32 | 1 | true | 0.013602614 | 0.002443363 |
| 80 | 8 | 1 | true | 0.013603985 | 0.002220447 |
| 320 | 32 | 0 | true | 0.013602614 | 0.002443363 |

Kp와 Kd를 같은 비율로 올린 80/8, 160/16, 320/32에서는 position bias가 그대로였다. Kp320에서 Kd를 8로 낮추자 약 0.003366 rad로 줄었다. 이 결과는 damping/Kp 비율이 이번 초기 자세 오차에 직접 영향을 준다는 근거다.

채택 조건인 **finger Kp320/Kd8, position64/velocity4**에서는 모든 57개 자세 관절이 기본 position Gate 허용값 안에 있었다. 최악의 angular 오차는 왼쪽 엄지 0.003365844 rad, lift 최대 오차는 0.002799758 m였다. Base의 마지막 실제 위치는 `(0.360001385, -1.249826908, 0.000000298)` m로 안정적이었다. Velocity iteration을 8에서 4로 낮추는 목적은 설치된 PhysX의 4 초과 경고를 제거하는 것이며, 이것만으로 position bias가 해결되지는 않았다.

## 해석과 범위

이 기록은 실제 position Gate의 여유를 확인한다. Reported velocity가 0이라고 주장하지 않는다. 채택 조건에서 왼쪽 엄지 position 오차의 측정 범위는 `[-0.003365844, -0.003361255]` rad로 거의 일정했지만, native reported velocity RMS는 약 0.05497 rad/s였다.

공식 PhysX 문서는 TGS의 split impulse 처리에서 reported velocity가 position의 유한 차분과 다를 수 있다고 설명하며, 외력의 substep 적용을 위한 scene flag를 제공한다. 이번 측정에서도 해당 flag는 reported velocity를 줄였지만 position bias를 해결하지 않았다. 따라서 최종 설정에는 적용하지 않았다. **Kp320/Kd8과 해당 flag를 함께 켠 조합은 검사하지 않았다.** [PhysX Simulation 문서](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/Simulation.html), [Omniverse Physics 제한 사항](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/guides/current_limitations.html)

정적 초기 자세의 1초 검사이며, 장시간 grasp 안정성이나 tactile 압력의 하드웨어 보정 결과는 아니다. 실제 GUI/ROS StateGate 확인은 이 설정을 production stage에 재구성한 후 별도로 수행한다. Gazebo 설정과 사용자 map/mission에는 변경이 없다.

전체 57개 관절별 position/velocity와 15개 조건은 `/tmp/hx5-isaac-pd-sweep.json`, 실행 로그는 `/tmp/hx5-isaac-pd-sweep.log`에 보존했다.
