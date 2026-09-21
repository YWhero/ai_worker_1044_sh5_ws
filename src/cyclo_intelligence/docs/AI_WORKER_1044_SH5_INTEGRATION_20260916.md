# 1044 SH5 통합 보완 — 2026-09-16

## 범위

대상은 `ai_worker_1044_sh5_ws/src/cyclo_intelligence`다.
기준 feature는 `7bcc876`, 공식 main 비교 기준은 `e648425`다.
`hero_gazebo_ws` 및 기존 실행 컨테이너는 수정·재시작하지 않았다.
새 이미지는 `cyclo-1044-sh5/*` 태그로만 빌드한다.

## 수정 사항

### 추론 타이밍 전달

UI/Action Canvas가 LOAD에 넣은 `control_hz`, `inference_hz`,
`chunk_align_window_s`를 실제 ControlLoop와 ActionChunkProcessor에 전달한다.
ViTacFormer temporal ensemble의 source rate도 함께 갱신하고 이전 이력을 비운다.
원본의 tactile 스케줄링, horizon, smoothing, 보호값은 변경하지 않았다.

생략·0·유효하지 않은 값은 공식 main과 같이 컨테이너 환경변수의 기본값을 쓴다.
다른 모델을 LOAD할 때 앞 모델의 명시적 rate가 기본값으로 남지 않는다.
UI의 Dataset FPS는 해당 모델이 학습한 데이터의 주기로 맞춘다. 이 설정은
추론 계산이 반드시 같은 초당 횟수로 완료된다는 뜻이 아니다.

### 학습 목록 복원

LeRobot `c8ce413`(0.5.2)의 원본 학습 목록을 복원했다. SAC를 다시 제공하고,
이 버전에 없는 Gaussian Actor 및 원본 학습 목록에 없던 FastWAM 항목은 제외했다.
ACT/Tactile ACT/ViTacFormer/T-Rex/FastWAM의 기존 추론 선택·라우팅은 유지한다.

### Head/Lift Jog 정지 확인

- `/odom` 미수신·만료·잘못된 속도값이면 Jog를 허용하지 않는다.
- 정지 명령 발행 후 새로 수신한 정지 odom 2개를 확인한 뒤 관절 명령을 보낸다.
- 중간 이동이 관측되면 정지 표본을 다시 모은다. 확인 시간이 초과되면 명령 없이 재시도를 안내한다.
- 대기 중 연결/관절 상태 소실, 패널 비활성화·종료, 대상 관절의 유의한 이동은 대기를 취소한다.
- Head와 Lift의 상태 요구는 독립적이며, 기존 관절 범위·증분·이동 시간은 유지한다.

이것은 이 UI의 명령 전 확인이다. 다른 publisher를 전역적으로 잠그는 제어권
관리자는 아니다. 이미 발행된 짧은 JointTrajectory를 action cancel로 중단하는
구조도 아니므로, 리더·추론 등 같은 관절을 제어하는 명령과 동시에 사용하지 않는다.

### FastWAM 소스 복원

기준 문서가 명시한 공식 LeRobot `240b4a0`의 FastWAM 패키지를
`cyclo_brain/policy/lerobot/backports/`에 출처·해시와 함께 보관한다.
LeRobot 서브모듈은 `c8ce413` 그대로이며 변경하지 않았다.

Docker 빌드에서만 새 패키지와 factory/processor/extra 등록 패치를 적용한다.
기존 정책 패키지와 의존성 범위는 그대로 유지하고 FastWAM 설정에만
`pretrained_revision` 호환 필드를 추가했다. 체크포인트나 원본 파일을 고쳐서
검사를 우회하지 않는다. 빌드 시 FastWAM policy/processor import도 검사한다.

이것은 원본 문서에 기록된 공식 이식 범위의 재구현이다. 확보하지 못한 1044의
로컬 수정 전체와 바이트 동일하다는 뜻은 아니다. 실제 checkpoint 로딩·GPU
offload·학습·추론 성공은 별도 검증 대상이다. native T-Rex 소스를 대체하지 않는다.

### 이미지 빌드 보완

ViTacFormer AMD64 빌드에서 OpenCV 4.12가 NumPy 2를 설치해 NGC 기본 이미지의
NumPy 1.x 기반 확장과 충돌하는 문제를 재현했다. AMD64는 NumPy 1.26.4,
OpenCV 패키지는 4.12 미만으로 제한했다. ARM64의 CUDA 라이브러리·NumPy를
임의 교체하지 않고 OpenCV 패키지의 상한만 동일하게 적용했다.

정책 Docker 컨텍스트에서는 호스트 서브모듈 `.git` 포인터와 Python 캐시를
제외한다. 이미지 내부 `git apply`가 호스트의 Git 경로를 찾다가 실패하는
문제를 예방하며 호스트 서브모듈 자체는 변경하지 않는다.

## 준비 명령 — 실행과 분리

호스트에서:

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws/src/cyclo_intelligence
./docker/container_1044.sh build main vitacformer lerobot
./docker/container_1044.sh check
```

`build`는 선택한 이미지만 준비하고 컨테이너를 시작·중지·교체하지 않는다.
인자가 없으면 main만 빌드한다. 필요할 때 `groot`도 선택할 수 있다.
`prepare`는 서브모듈 초기화만 한다.

`check`는 모든 백엔드와 navigation의 준비 현황을 표시한다. 사용하지 않는
T-Rex·GR00T가 미준비여도 다른 backend나 UI의 실행을 전역 차단하지 않는다.
FastWAM의 build-time source 준비와 실제 정책 이미지 준비를 구분해서 표시한다.
기존 `start*`는 빌드뿐 아니라 실행도 하므로 비구동 준비 단계에서는 사용하지 않는다.

## 검증 및 남은 조건

- UI: 88 suites / 876 tests 통과.
- supervisor + 공용 control loop/service handler: 220 tests 통과.
- ViTacFormer: 90 passed / 2 skipped. 학습 가중치가 아닌 합성 입력·참고 소스 비교를 포함한다.
- Docker/출처/빌드 패치/학습 목록 회귀 검사: 47 tests 통과.
- 위 회귀 검사의 합계는 1,233 passed / 2 skipped다. 전체 실기기 기능의 성공을 뜻하지 않는다.
- main, ViTacFormer, LeRobot AMD64 전용 이미지 빌드 완료.
  `check`에서도 세 이미지의 존재를 확인했다. GR00T와 native T-Rex 이미지는 빌드하지 않았다.
- ViTacFormer 이미지에서 설정을 read-only mount하고 네트워크·장치 없이
  모델 및 타이밍 API import를 확인했다. NumPy 1.26.4 / cv2 4.10.0이다.
- LeRobot 이미지에서 ACT/SAC/FastWAM policy 및 Tactile ACT 어댑터 import,
  FastWAM 54차원 설정 저장·재로딩, 전·후처리 파이프라인 생성을 확인했다.
  checkpoint를 로딩하거나 로봇에 명령을 보내지 않았다.
- 이번 LeRobot 빌드의 설치 버전은 LeRobot 0.5.2 / torch 2.10.0 /
  torchvision 0.25.0 / transformers 5.5.4 / diffusers 0.35.2 / NumPy 2.2.6이다.
  원본의 의존성 범위를 유지했지만 그 범위 안의 실제 설치 버전은 빌드 시 결정된다.
  따라서 원래 1044의 설치 환경과 완전히 같다고 보증하지 않는다. 당시 lockfile이나
  이미지 digest를 확보하면 추가 비교할 수 있다. ViTacFormer는 별도 이미지 환경이다.
- 기존 손·촉각·녹화·변환·모델 전환·SH5 URDF의 비교 대상 15개 파일은 원본과 동일하다.
  주요 tactile 및 real-action 보호 함수도 AST 비교상 동일하다.

native T-Rex 구현, 실제 1044의 AI Worker/HX5 driver 및 장치 설정,
hand-preset/navigation 컨테이너, 실제 checkpoint 검증은 여전히 필요하다.
1044 실기기 연결·LiDAR·TF·mapping·localization·Nav2 주행이나 ARM64 이미지
빌드는 수행하지 않았다. tactile grasp/FK 높이 Gate도 아직 구현하지 않았다.
Upright ViTacFormer 모델 카드의 연구용·실기기 배포 미승인 상태는 이번 소스
호환 작업으로 바뀌지 않는다. ROS domain을 실제 로봇과 맞추기 전 장치 구성부터 확인한다.

Docker 이름·포트·데이터·ROS 도메인은 분리하지만 호스트 GPU/IPC/네트워크와
Docker 데몬은 공유한다. 완전한 자원·보안 격리로 간주하지 않는다.
