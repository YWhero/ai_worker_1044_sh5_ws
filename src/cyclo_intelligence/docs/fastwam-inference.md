# FastWAM inference 준비

현재 `wip/seungwoo-20260824` 브랜치에 FastWAM 관련 코드만 이식했다.
UI의 **Inference → Model → LeRobot → FastWAM**에서 선택할 수 있으며,
선택하면 Task Instruction 입력란이 표시된다. 선택 값은
`serviceType=lerobot`, `policyType=fastwam`이다.

## 모델이 준비되면

현재 작업은 UI 준비와 소스 이식까지다. 실행 중인 LeRobot 이미지에는
새 정책을 설치하지 않았으므로, 처음 모델을 사용할 때 로컬 소스로
일반 LeRobot 백엔드를 재빌드해야 한다.

```bash
cd /home/robotis/cyclo_intelligence
CYCLO_LEROBOT_POLICY_FLAVOR=default bash docker/container.sh start-lerobot --build
```

재빌드는 LeRobot 컨테이너를 재생성하므로 진행 중인 추론이 끝난 뒤 실행한다.
FastWAM은 일반 `default` 환경을 사용한다. T-Rex 전용 환경은 별도 의존성을 사용한다.

체크포인트를 `docker/workspace/model/lerobot/` 아래에 놓고 UI에서 선택한다.
컨테이너에서는 `/workspace/model/lerobot/` 경로로 보인다. 체크포인트에는
`type: fastwam`인 `config.json`, 가중치, 학습 시 저장한 전·후처리기 파일이
필요하다. Task Instruction을 입력한 후 모델 로딩을 확인한다.
실제 호환성은 향후 체크포인트의 카메라·상태·행동 정의를 기준으로 검증해야 한다.

## 이식 범위

- Cyclo 원본: `upstream/main`의 `e648425`.
- LeRobot 원본: 위 main이 고정한 `240b4a0314ae0879cdd928c7f4bdc1eee9a01b3b`.
- FastWAM 모델 패키지, 정책·프로세서 등록, FastWAM 의존성 extra.
- CPU 우선 로딩, 텍스트 인코더 CPU 유지, 지시문 임베딩 캐시 및 갱신.
- Inference/BT 모델 선택 및 LeRobot 라우팅, 카메라 이름 alias.

기존 LeRobot submodule 커밋과 버전은 유지했다. FastWAM 설정에만
0.6.1의 `pretrained_revision` 필드를 수용하고, 선택하지 않은 정책에
추가 모델 의존성을 강제하지 않도록 FastWAM의 모델 import를 지연한다.
저장된 FastWAM 프로세서 복원 전에 전용 processor를 등록한다.

## 검증

- UI 선택·정책 변경·제어 관련 테스트: 37개 통과.
- FastWAM 로딩·지시문 캐시 테스트: 4개 통과.
- 카메라 매핑·촉각 설정 테스트: 6개 통과.
- 현재 LeRobot 이미지에서 이식된 소스를 사용한 FastWAM 클래스/Wan VAE import 확인.
- 가중치 없이 FastWAM 설정 및 저장된 전·후처리기 복원 확인.
- 실제 모델 로딩 및 로봇 추론은 체크포인트 준비 후 검증할 항목이다.
