# 1044 ViTacFormer Upright H100 소스 호환 확장

## 범위

대상은 `ai_worker_1044_sh5_ws/src/cyclo_intelligence`뿐이다.
`hero_gazebo_ws`, 다른 저장소, 실행 중인 컨테이너는 변경하지 않았다.

참고 모델:
[Task000608 Upright ViTacFormer](https://huggingface.co/Dongkkka/Task000608_Upright_ViTacFormer_H100_B512_Hand_Intern).
확인한 리비전은 `71ac7bf6cce3460192a13386b6268c4849fa09a9`다.
소스와 작은 설정 파일만 검토했으며, 학습 가중치를 다운로드하거나 실행하지 않았다.

이 저장소는 ViTacFormer 모델·학습·전처리·추론 참고 소스다.
native T-Rex, FastWAM의 LeRobot 이식분이나 1044의 HX5 드라이버를
대체하는 소스는 아니다. AI Worker/HX5 공식 패키지는 별도로 존재하며,
1044가 사용한 정확한 버전·장치 설정·추가 커스텀은 별도 대조 대상이다.

## 발견한 호환 차이와 수정

| 항목 | 기존 | 이번 확장 |
|---|---|---|
| 모델 형식 | legacy H100, H200, Pour H100 | Upright H100 형식 추가 |
| Upright architecture | 지원 목록에 없음 | `vitacformer_sh5_upright_h100_v2` |
| Upright recipe | 지원 목록에 없음 | `task608_upright159_gt75_residual_unpenalized_r1` |
| 기본 가중치 | `best_model.pt` 계열 | Upright에 한해 `checkpoints/best_validation.pt` |
| 촉각 출력 | 모델별 residual 처리 | Upright는 원본대로 양손 learned residual 사용 |

추가한 형식의 입출력은 관절 54차원, 촉각 표현 180차원, action 100행이다.
전처리와 디코더가 기존 Pour H100 경로와 일치함을 소스·설정·출력 비교로
확인했으므로 별도 모델 엔진을 복제하지 않고 기존 ViTacFormer loader를 확장했다.

기존 모델의 기본 가중치 선택, tactile baseline, preload/switch, 녹화 토픽,
관절 순서, 관절 제한값, warm-start 범위 및 실행 타이밍은 변경하지 않았다.
원본의 관절 순서/단위/차원 검증, stats/config 해시 바인딩 및 strict weight
loading도 유지한다. artifact 파일을 고쳐서 검사를 통과시키지 않는다.

Upright 폴더에 기본 가중치가 없으면 오류를 표시한다. 다른 모델을 자동 선택하지
않는다. `latest_model.pt`를 시험하려면 해당 `.pt` 경로를 명시해야 한다.

## 검증

- 참고 저장소의 `inference_config.json`을 리비전과 SHA256으로 고정한 fixture 추가.
- 신규 형식·파일 선택·hash/recipe/전처리 오류·양손 residual·strict loading 회귀 검사.
- 원본 소스와 동일한 무작위 가중치를 사용한 CPU 출력 비교:
  접촉/비접촉 합성 입력에서 관절 action과 future tactile 모두
  `atol=1e-5`, `rtol=1e-5` 범위로 일치. 학습된 checkpoint 결과는 아니다.
- 위 신규 테스트 21개 통과. 네트워크를 끈 일회용 컨테이너에서 실행했으며
  Docker socket, 장치, ROS 연결을 제공하지 않았다.
- 실제 GPU checkpoint 로딩, 센서 동기화, 작업 성공률, 실기기 실행은 미검증이다.

## 모델 패키지를 준비해 사용할 때

아래 다운로드는 아직 실행하지 않았다. 가중치까지 준비하려는 시점에 사용한다.
1044 main 컨테이너에서 실행해야 `/workspace`가 이 워크스페이스의
`docker/workspace`를 가리킨다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws/src/cyclo_intelligence
./docker/container_1044.sh enter
```

컨테이너 안:

```bash
python3 - <<'PY'
from huggingface_hub import snapshot_download

snapshot_download(
    repo_id="Dongkkka/Task000608_Upright_ViTacFormer_H100_B512_Hand_Intern",
    revision="71ac7bf6cce3460192a13386b6268c4849fa09a9",
    local_dir="/workspace/model/vitacformer/task608_upright_h100",
    ignore_patterns=["**/__pycache__/**", "*.pyc"],
)
PY
```

이후 오프라인 검증 환경을 준비하려면 호스트에서
`./docker/container_1044.sh start-vitacformer`를 사용한다. 전용 이미지로 빌드하며,
이미 실행 중인 이 전용 backend가 있으면 재생성하므로 해당 추론을 먼저 중지한다.
기존 이미지의 단순 restart로는 변경한 엔진이 반영되지 않는다.

Cyclo에서 지정하는 모델 경로는
`/workspace/model/vitacformer/task608_upright_h100`이며,
모델 선택은 **ViTacFormer**, robot type은 `ffw_sh5_rev1`이다.
이 경로는 모델 형식 준비 안내이지 실기기 Start/Resume 실행 승인이 아니다.
실시간 UI LOAD도 로봇 센서 초기화를 요구하므로 비구동 파일 로딩 검사와 구분한다.

## 실기기 사용 주의

모델 카드에는 미완성 연구 checkpoint이며 powered SH5 deployment 승인을 받지
않았다고 명시돼 있다. 호환 loader 구현은 이 판정을 바꾸지 않는다.
체크포인트와 실센서를 이용한 비구동 검증, 1044 장치/제어기 대조가 먼저 필요하다.
이번 작업에서 새로운 전역 잠금을 추가하거나 기존 보호값을 완화하지 않았다.
