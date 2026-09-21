# SH5 ViTacFormer 수동 모델 전환

## 확인 결과

BT Manager는 UI(`btSupport.js`), supervisor API, launch, BT 노드에서
`ffw_sg2_rev1`만 허용한다. BT의 `SendCommand`는 LOAD/RESUME/STOP/CLEAR를
트리로 조합할 수 있지만, 기존 SG2 예제는 한 모델을 반복 실행하며
사람이 버튼으로 두 모델 사이를 넘기는 전용 기능은 없다.

이번 기능은 **Inference 탭에서 독립적으로 동작**한다. SH5의 정책 전환에
BT 노드를 실행할 필요가 없으며, BT의 SG2 전용 JointControl/Rotate 기능을
SH5에도 지원한다고 표시하지 않는다.

## 사용법

1. 로봇을 `ffw_sh5_rev1`, 모델을 `ViTacFormer`로 선택한다.
2. 기존 **Policy Path**에 전반부 모델을 지정하고 Start한다.
3. 그 아래 **Manual model switching → Next model path**에 후반부 모델
   폴더를 입력하거나 폴더 버튼으로 선택한다. 실행 전에도 지정할 수 있다.
4. 전환할 시점에 **Switch to next model**을 누른다.
5. 출력 일시정지 → 다음 모델 로드 → 기존 실기기 첫 동작 검사 → 재개 순서로
   실행된다. 현재 Sim/Real 모드, 센서 연결, 최초 tactile 영점은 유지한다.
   Cycle Home/초기 자세 복귀는 호출하지 않는다.
6. 로딩 중 **Stop**을 누르면 자동 재개를 취소한다. 로드는 완료될 수 있지만
   다음 모델은 PAUSED 상태로 남는다. 이후 Start로 명시적으로 재개할 수 있다.

호스트의 `docker/workspace/model/...` 폴더는 UI에서 `/workspace/model/...`로
선택한다. 질문에 제시된 기존 모델은
`/workspace/model/lerobot/Dongkkka/Task000519_000608_ViTacFormer_400k_Hand_Intern`이다.
전반부/후반부로 별도 학습한 체크포인트는 아직 생성하지 않았다.

기본 전환은 가중치를 버튼 클릭 후 읽는다. 아래의 preload 옵션을 사용하면
모델 로딩과 워밍업을 전환 전에 완료할 수 있다. 두 모델을 위한 메모리가
필요하다. 로드 실패 시 자동 재개하지 않으며, 로드가 성공하고
첫 동작 검사가 실패했다면 UI에는 새 모델 경로와 PAUSED 오류가 표시된다.
응답이 확인되지 않은 전환은 일반 Start/Resume을 차단하고 재시도 또는
Clear를 요구한다. 연결이 끊겨 UI가 모델 상태를 확인할 수 없으면 재연결 후
Stop, Clear로 종료하고 새 실행을 준비한다.

## 사전 로딩으로 빠르게 전환하기

1. Start 전에 모델 1의 **Policy Path**와 모델 2의 **Next model path**를 지정한다.
2. **Preload next model for fast switching**을 체크하고 **Preload next model**을 누른다.
3. **Preload ready · warmup complete**가 표시되면 모델 1을 Start한다.
4. 원하는 시점에 **Switch to next model**을 누른다. 이 경로는 GPU에
   준비된 모델로 교체하며 체크포인트를 다시 읽거나 GPU로 전송하지 않는다.

이미 추론 중이라면 먼저 **Stop**으로 일시정지한 뒤 preload하고 Start로
모델 1을 재개할 수 있다. 추론 중 GPU 로딩·워밍업 부하로 기존 모델이
멈추는 것을 막기 위해 preload/release는 READY 또는 PAUSED에서만 허용한다.
준비 중에는 Start, Clear, Cycle Home, 추가 모델 전환을 차단한다.

모델 2는 실제 추론과 동일한 크기·정밀도의 합성 입력으로 워밍업하며,
합성 입력의 temporal history를 초기화한 뒤 준비 완료를 알린다. 이 과정은
RobotClient를 생성하거나 tactile 영점을 다시 측정하지 않고, 동작을 출력하지 않는다.
모델 1을 처음 로드해도 준비된 모델 2는 유지된다.

전환 시에는 이전 action 큐를 폐기하고 모델 2로 현재 관측에 대한 새 chunk를
계산한 뒤 기존 Real 첫 동작 검사를 거쳐 재개한다. **가중치 로딩 대기는
없지만 새 동작 계산·검사·서비스 왕복 시간까지 0 ms라는 의미는 아니다.**
준비한 모델이 사라졌거나 경로가 다르면 디스크 로딩으로 대체하지 않고
PAUSED로 남아 다시 preload하도록 안내한다.

빠른 전환 후 이전 모델은 spare로 남아 두 모델의 가중치가 계속 메모리에
유지된다. PAUSED/READY에서 **Release preload**를 누르면 spare만 해제한다.
일반 **Clear**는 실행 모델과 spare를 함께 해제한다. 다음 경로를 변경하면
새 경로가 준비될 때까지 빠른 전환은 비활성화된다. 메모리 부족이나 워밍업
실패 시 현재 모델과 센서 영점은 유지되고 자동으로 추론을 시작하지 않는다.

사용자 UI 명령은 PRELOAD=27, RELEASE=28, SWITCH_PRELOADED=29,
Main 명령은 9/10/11, Engine 명령은 5/6/7이다. 기존 26/8/4 경로는
preload 옵션을 끈 일반 전환에 사용한다. 모든 Engine 작업은 기존 명령
잠금으로 직렬화하며, 전환은 여전히 PAUSE → 교체 → 별도 RESUME 순서다.

preload 추가 검증: 공통 런타임 123개, orchestrator 38개, ViTacFormer
21개, React/ROSLIB 73개로 총 **255개**가 통과했다. Python 테스트는
서로 다른 프로젝트의 전역 로깅 설정이 간섭하지 않도록 분리 실행했다.

## 데이터 분할 시 맞춰야 할 조건

- 영상만 자르지 말고 camera, state, action, tactile의 시간 구간을 함께
  분할한다. 제거한 중간 구간을 가로질러 action chunk가 만들어지지 않아야 한다.
- 2번 모델의 학습 시작 자세·물병 기울기·손 접촉 상태를 실제 수동 전환 시점과
  맞춘다. 첫 구간을 일찍 끝내고 두 번째 구간을 늦게 시작했다면, 그 사이의
  상태 변화는 이 버튼 자체가 만들어 주지 않는다.
- 이번 런타임은 동일한 원본 에피소드의 tactile 영점을 유지한다. 후반부만
  잘라 놓고 물병을 잡은 상태를 새 영점으로 계산하면 학습/추론 입력이 달라진다.
  원본 기준 영점을 유지하는 전처리와 학습 데이터가 필요하다.
- 후반부 시작 프레임에 필요한 state 6프레임(10 Hz), tactile 18프레임(30 Hz)
  이력도 원본에서 확보한다. 첫 프레임 반복 패딩 여부까지 런타임과 맞춘다.
- 현재 백엔드는 기존 SH5 ViTacFormer 모델의 이미지 크기, 54차원 action,
  센서 표현과 architecture 계약을 검사한다. 분할 모델도 이 계약을 따라야 한다.

## 구현과 검증 범위

`SendCommand.SWITCH_INFERENCE_MODEL=26`을 새 진입점으로 사용한다.
orchestrator가 PAUSE → `InferenceCommand.SWITCH_POLICY=8` → RESUME을
직렬 실행한다. 엔진에는 `EngineCommand.SWITCH_POLICY=4`로 전달된다.
EngineWorker의 명령 잠금은 아직 끝나지 않은 GET_ACTION과 가중치 교체가
동시에 실행되지 않게 한다. Main의 PAUSE는 action 큐와 이전 응답 세대를
무효화한다. ViTacFormer는 새 모델의 normalization과 temporal ensemble을
교체하며 RobotClient와 tactile baseline을 다시 만들지 않는다.

검증은 모의 서비스·모델을 사용하는 Python/React 테스트와 별도 경로의
ROS 인터페이스 및 프로덕션 UI 빌드로 수행한다. 실제 로봇의 자세 유지,
모델 로드 시간, 물 따르기 성공률은 실기기 검증 대상이다.

2026-09-07 검증: 공통 런타임·orchestrator 116개, ViTacFormer 14개,
React/ROSLIB 63개로 총 193개 테스트가 통과했다. ROS 인터페이스 빌드도
통과했다. 기존 ViTacFormer prediction 테스트 대역에는 현재 코드가 호출하는
`parameters()`를 추가하여 CPU 테스트 계약을 맞췄다.

## 최초 수동 전환 배포 (2026-09-07)

사용자 승인 후 ViTacFormer STOP과 inference Clear를 실행하고,
`interfaces`·`orchestrator`를 현재 ROS workspace에 빌드·설치했다.
UI를 nginx에 반영했으며 기존 `cyclo-config.js` 포트 설정은 보존했다.
기존 수동 bringup을 종료하고 통합 `cyclo_intelligence` s6 서비스로
orchestrator·rosbridge·cyclo_data를 재시작했다. ViTacFormer 컨테이너에는
전환 관련 Python 파일 6개를 반영하고 main-runtime·engine-process를
재시작했다. SH5 설정도 복구했다.

현재 상태는 **READY, 모델 미로드, 추론 출력 정지**다. 모델이나 실제
로봇 동작을 자동으로 다시 시작하지 않았다.

- nginx에서 제공하는 index와 검증된 빌드의 SHA-256 일치를 확인했다.
  당시 메인 번들은 `/static/js/main.b25b7c8b.js`였다.
- 실제 rosbridge에서 새 SendCommand 응답 필드와 미실행 상태의 전환
  거절을 확인했다. Main/Engine의 새 명령도 각각 올바르게 인식한다.
- supervisor API에서 ViTacFormer의 두 프로세스가 `up`임을 확인했다.
- 설치 중 `setup.py`가 BT template의 `__pycache__` 디렉터리까지 파일로
  복사하던 문제를 발견해 일반 파일만 포함하도록 수정하고 빌드를 완료했다.

백업과 배포 기록은 호스트의
`docker/workspace/deployments/manual-model-switch-20260907-1644/`에 있다.
검증용 빌드는 cyclo_intelligence 컨테이너의
`/tmp/cyclo-model-switch-check/install`과 `/tmp/cyclo-model-switch-ui`에 있다.

이번 반영은 현재 컨테이너의 설치 파일을 갱신했으므로 해당 컨테이너를
restart해도 유지된다. 컨테이너를 삭제·재생성할 때는 수정된 소스로 이미지를
다시 빌드하거나 동일한 배포를 적용해야 한다. `SendCommand` 응답 형식이
변경되었으므로 이전 인터페이스로 실행 중인 노드와 새 UI를 섞지 않는다.

## Preload 추가 배포 (2026-09-07)

현재 UI 번들은 `/static/js/main.10b4921d.js`다. ROS의 `interfaces`와
`orchestrator` 빌드·설치, nginx UI 반영, ViTacFormer 런타임 7개 파일
반영 및 두 프로세스 재시작을 완료했다. `cyclo-config.js` 포트를 보존했다.
배포 확인 중 남아 있던 이전 rosbridge 고아 프로세스를 종료하고,
orchestrator·rosbridge가 각각 한 프로세스로 실행되는 것을 확인했다.

실제 rosbridge → orchestrator → Main → Engine 경로에서 제시된 기존
체크포인트를 CUDA에 preload하고 워밍업 완료 응답을 받았다. 동일 경로를
다시 준비하면 캐시를 재사용하며, Release 후에도 READY 상태가 유지됐다.
검증용 모델은 모두 해제했으며, **현재 READY·모델 미로드·추론 출력 정지**다.

별도 GPU 검증은 기존 체크포인트 두 사본과 합성 입력으로 수행했다.
RobotClient와 로봇 출력 publisher는 생성하지 않았다. 이 검증에서
preload+워밍업은 약 1.93 s, 메모리 내 정책 교체는 약 0.033 ms,
교체 후 첫 100×54 동작 chunk 계산은 약 108 ms였다. 이는 실기기 서비스
왕복·PAUSE·첫 동작 검사까지 포함한 버튼 응답 지연 측정이 아니다.
두 모델의 PyTorch 할당 메모리는 약 730 MiB, 예약 메모리는 약 902 MiB였다.
두 정책 객체의 해제도 확인했으며 CUDA 내부 작업공간 약 8 MiB는 남았다.
서로 다른 전반부/후반부 체크포인트의 실제 성공률은 별도 실행으로 검증해야 한다.

백업, 소스 해시, 실제 서비스 응답, GPU 검증 스크립트와 측정 결과는
`docker/workspace/deployments/model-preload-20260907/`에 보관했다.
