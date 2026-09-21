# SH5 네 카메라 녹화와 Record 카메라 선택

2026-09-11 현재 장비에 소스 수정, ROS 패키지 빌드, UI 빌드·배포 및 Cyclo 재시작까지 적용했다. 로봇 유형은 `ffw_sh5_rev1`, 녹화 카메라는 아래 네 개 모두 ON이다.

- Head L: `cam_left_head`
- Head R: `cam_right_head`
- Wrist L: `cam_left_wrist`
- Wrist R: `cam_right_wrist`

## 지금 사용하는 방법

1. Cyclo Intelligence UI에서 **Ctrl+Shift+R**로 새로고침한다.
2. **Record** 탭으로 이동한다. Robot Type이 비어 있으면 `ffw_sh5_rev1`을 선택한다.
3. Cameras가 **4/4**이고 Head L/R, Wrist L/R이 모두 **ON**인지 확인한다. 필요하면 **All ON**을 누른다.
4. 네 미리보기와 각 카메라의 Hz를 확인하고 평소 방식으로 녹화한다.

카메라 이름 버튼을 누르면 개별 ON/OFF가 즉시 적용된다. **All ON / All OFF**는 전체 선택이다. 선택은 서버에서 로봇별로 자동 저장되며 새로고침이나 Cyclo 재시작 후에도 유지된다. 녹화·저장 중에는 버튼이 잠긴다.

ON/OFF는 **미리보기와 실제 영상·camera_info 저장 여부**를 함께 바꾼다. 카메라 하드웨어 전원을 끄는 기능은 아니다. All OFF 상태에서도 관절·촉각 등 카메라 외 데이터는 녹화된다. `Waiting for frames`는 해당 카메라의 프레임 수신을 아직 확인하지 못했다는 뜻이다.

## 원인과 수정 범위

`ffw_sh5_follower_ai.launch.py`는 이미 ZED 좌우 및 RealSense 양쪽 손목 카메라를 실행하고 있었다. 기존 Cyclo 녹화 서비스는 `/ffw/head_camera_profile`의 추론용 head 선택을 녹화에도 적용해 카메라 하나만 구독했다. 기존 공용 미리보기도 3칸으로 제한돼 있었다.

녹화 서비스의 head profile 종속을 제거하고, `/data/recording/cameras` (`interfaces/srv/RecordingCameras`)에 실제 녹화 카메라 목록의 조회·변경 기능을 추가했다. Record는 전용 2×2 미리보기를 사용하며, 손목 회전과 영상 전체 표시를 지원한다. 추론의 head profile 선택은 기존대로 유지한다.

설정 파일:

- 컨테이너: `/workspace/config/recording_cameras.json`
- 호스트: `~/cyclo_intelligence/docker/workspace/config/recording_cameras.json`

설정이 없는 로봇은 YAML에 등록된 모든 카메라를 기본 사용한다. 명시적인 빈 목록 `[]`은 전체 OFF다. 새 서비스는 기존 RecordingCommand와 별도 인터페이스이므로 기존 녹화 명령 형식을 바꾸지 않는다.

## 나중에 수정본을 다시 적용할 때

현재 장비에는 이미 적용했으므로 지금은 아래 빌드가 필요 없다. 컨테이너를 새로 만들었거나 소스를 다시 수정했다면, 녹화를 종료하고 Cyclo 실행을 중지한 상태에서 다음을 수행한다.

```bash
docker exec cyclo_intelligence bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /root/ros2_ws
  colcon build --symlink-install --packages-select interfaces cyclo_data
'

cd ~/cyclo_intelligence
./docker/container.sh build-ui
```

그 후 Cyclo를 평소 방식으로 다시 실행한다. 직접 실행하는 경우 컨테이너 안에서:

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch orchestrator cyclo_intelligence_bringup.launch.py
```

UI는 Ctrl+Shift+R로 새로고침한다. `ai_worker` 카메라 드라이버는 이번 수정으로 재시작하거나 재빌드할 필요가 없다. 현재 Cyclo는 s6 관리 서비스로 실행 중이며 로그는 컨테이너의 `/var/log/cyclo_intelligence/current`에 있다.

## 검증

- 백엔드 테스트 26개 통과: 기본 4개, 개별 선택, 전체 OFF, 영구 저장, 기록·저장 중 변경 차단, 잘못된 카메라 거부, 실패 시 복원.
- UI 관련 테스트 총 12개 통과: 네 카메라, 실제 서비스 요청, 전체 선택, 잠금, 실패 표시, 회전 매핑.
- ROS `interfaces`, `cyclo_data` 빌드 및 React 배포 빌드 성공.
- 실제 rosbridge를 통한 0개 → 2개 → 4개 선택 검증. 최종 4개 ON.
- 네 카메라 각각 약 30Hz 수신 확인.
- 별도 3초 시험 녹화: Head L 90, Head R 89, Wrist L 90, Wrist R 90프레임 저장. 큐·잘못된 프레임 손실 0. MP4 4개와 camera_info YAML 4개 생성.
- 시험 파일은 컨테이너 `/tmp/four-camera-recording-check-gj5q6v28`에 있으며 사용자 작업 데이터셋에는 추가하지 않았다.

작업 시작 전 기존 수정 파일의 백업: `~/camera_record_backup_20260911/`. 기존의 다른 수정사항은 유지했다.

## UI 미리보기 지연 개선 (2026-09-11)

Record 미리보기는 이제 녹화와 별도의 `camera_preview_node`를 사용한다.

- 원본 카메라 토픽, 녹화 해상도와 프레임률, 저장 모듈은 변경하지 않는다.
- 미리보기: 가로 최대 424px, JPEG quality 45, 최대 12fps. 작은 손목 영상은 확대하지 않는다.
- 서버는 원본 토픽을 QoS depth=1, BEST_EFFORT로 계속 구독하면서 최신 프레임 하나만 보관한다. HTTP 요청마다 원본 ROS 구독을 새로 만들지 않는다. 동일 카메라를 여러 브라우저에서 보아도 재압축 결과를 공유한다.
- 브라우저는 한 장을 받고 디코딩·표시한 뒤 다음 장을 요청한다. 느린 연결에서는 표시 FPS가 낮아지며, 과거 영상이 줄지어 쌓이지 않는다. 750ms를 넘긴 요청은 취소하고 늦게 도착한 프레임은 버린다.
- 오래되었거나 끊어진 영상은 `Waiting for live preview…`로 표시한다. 다른 탭으로 이동하거나 브라우저가 숨겨지면 미리보기 요청을 중단한다.
- HTTP 기본 포트는 7086이다. 서버 `CYCLO_CAMERA_PREVIEW_PORT`와 UI runtime config의 `cameraPreviewPort`로 변경할 수 있다.

적용 당시 중복된 Cyclo launch 3개를 녹화 대기 상태에서 정상 종료했다. 이제 s6의 `cyclo_intelligence` 서비스 하나로 실행한다. 같은 bringup을 추가 실행하면 파일 잠금으로 차단하고 `already running` 오류를 표시한다.

**현재 장비는 적용 완료. 접속 중인 모든 브라우저에서 Ctrl+Shift+R로 새로고침해야 기존 원본 MJPEG 연결이 새 미리보기 방식으로 바뀐다.** 카메라와 `ai_worker` 재시작은 필요 없다.

나중에 이 수정본을 다시 빌드할 경우 ROS 패키지 선택에 `orchestrator`도 포함한다:

```bash
docker exec cyclo_intelligence bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /root/ros2_ws
  colcon build --symlink-install --packages-select interfaces cyclo_data orchestrator
'
cd ~/cyclo_intelligence
./docker/container.sh build-ui
```

현재 관리 서비스의 재시작이 필요할 때는 녹화·저장을 끝낸 후 컨테이너 안에서 다음을 사용한다. 이미 실행 중일 때 별도 터미널에서 bringup을 추가 실행하지 않는다.

```bash
/command/s6-rc -d change cyclo_intelligence
/command/s6-rc -u change cyclo_intelligence
```

검증: 새 백엔드 테스트 6개와 관련 UI 테스트 11개 통과. 두 브라우저에 해당하는 8개 미리보기 요청과 원본 4카메라 녹화를 동시에 시험했다. 미리보기 한 프레임은 평균 약 10~11KB, 브라우저 한 대의 4카메라 전송량은 약 4.2Mbps로 원본 미리보기 대비 약 90% 이상 감소했다. 로컬 HTTP 응답 p95는 카메라·클라이언트별 약 10~50ms였다. 원본 녹화는 Head L/R 88프레임, Wrist L/R 90프레임을 각각 수신한 그대로 전부 저장했으며 큐·잘못된 프레임 손실은 0이었다. 실제 원격 화면 지연에는 네트워크와 브라우저 표시 시간도 추가된다.
