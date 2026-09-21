# SH5/HX5 Isaac 카메라 화질과 공식 프로파일 적용

## 확인 결과

화질 제한은 UI에만 있지 않았습니다. 변경 전 실제 ROS 원본 네 개는 모두 336×188, JPEG 품질 75였고, Cyclo 미리보기에서 JPEG 품질 45로 다시 압축했습니다. 이는 실제 AI Worker의 기본 RGB 프로파일보다 낮은 해상도였습니다.

공식 SH5 카메라는 Head의 ZED Mini 스테레오와 좌우 Wrist의 D405입니다. 공식 `zedm.yaml`은 VGA·30 FPS이며, `common_stereo.yaml`은 NATIVE publication·downscale factor 1.0입니다. VGA의 실제 크기는 제조사 표의 672×376입니다. Wrist RGB 설정은 `depth_module.color_profile1/2=424,240,30`입니다. 480×270은 해당 launch의 **depth** 프로파일이므로 RGB 해상도와 구분해야 합니다.

| 경로 | 변경 전 실측 | 적용 후 실측 |
| --- | --- | --- |
| Head L/R ROS 원본 | 336×188, JPEG75 | ZED Mini 672×376, JPEG95, 4:4:4 |
| Wrist L/R ROS 원본 | 336×188, JPEG75 | D405 424×240, JPEG95, 4:4:4 |
| Isaac UI 미리보기 | JPEG45, 최대 폭 424 | JPEG95, 최대 폭 1280; 기본 원본 크기 유지 |
| 원본 frame 주기 | 15 SIM Hz | 30 SIM Hz |

JPEG95는 시뮬레이션의 영상 전송 설정입니다. 하드웨어 자체의 해상도나 광학 품질을 뜻하지 않습니다. 공식 ROS `compressed_image_transport`의 JPEG 기본 품질도 95입니다.

Head 명목 rectified intrinsics는 제조사 WVGA 표의 `fx=fy=367`, 중심 `(336,188)`입니다. FOV는 약 84.95°×54.25°입니다. Wrist는 제조사 공칭 87°×58°로부터 `fx≈223.4014`, `fy≈216.4857`, 중심 `(212,120)`을 계산했습니다. 실물의 serial별 factory calibration을 확보한 값이 아니며, 프로파일 metadata에도 nominal로 표시합니다. Wrist의 기존 raw topic 이름은 유지하되 현재 renderer는 왜곡 없는 nominal pinhole 근사입니다.

Isaac의 공식 OpenCV pinhole lens schema에 해상도와 fx/fy를 직접 적용했습니다. 생성자와 initialize가 기본 aperture를 square pixels로 변경하므로, 기본 aperture로 K를 다시 계산하지 않습니다. runtime의 `get_opencv_pinhole_properties()`에서 실제 lens 값을 읽어 ROS `CameraInfo`에 반영합니다. 왜곡 계수는 0이며 fStop=0으로 보정 근거 없는 depth-of-field 흐림을 적용하지 않습니다. Head의 실제 mount와 63 mm baseline은 유지하고, 오른쪽 stereo projection `P[3]=-367×0.063=-23.121`도 제공합니다.

카메라 물리 mm와 USD의 tenths-of-stage-unit을 구분합니다. meter stage에서 WVGA focal 2.936 mm는 USD 0.02936이고 aperture 5.376×3.008 mm는 USD 0.05376×0.03008입니다. 투영과 단위를 실제 생성 USD 검사에 포함했습니다.

## 원본과 UI를 직접 비교하기

현재 Isaac UI는 **http://localhost:7880/** 입니다. UI 미리보기는 최대 12 FPS로 제한하지만 ROS 원본·녹화 경로에 미리보기 JPEG를 공급하지 않습니다. 녹화는 기존 Recorder의 별도 MP4 인코딩 경로입니다.

- [현재 Head L 미리보기](http://localhost:7886/snapshot?topic=%2Fzed%2Fzed_node%2Fleft%2Fimage_rect_color%2Fcompressed)
- [현재 Head L 원본 ROS JPEG](http://localhost:7886/snapshot?topic=%2Fzed%2Fzed_node%2Fleft%2Fimage_rect_color%2Fcompressed&quality=original)

`quality=original`은 수신한 ROS JPEG bytes를 decode·resize·재압축 없이 반환합니다. 센서의 무손실 RAW를 뜻하지 않습니다. 첫 구독은 새 frame을 기다리면서 잠시 HTTP503을 반환할 수 있습니다. 최신 frame의 TTL 0.75초, no-store와 X-Frame-Age-Ms를 유지합니다.

변경 전 미리보기:

![변경 전 Head L](/home/robotis-ai/workspaces/isaac_logistics_cell_ws/reports/camera_quality/before/preview_head_left.jpg)

공식 프로파일 적용 후 미리보기:

![적용 후 Head L](/home/robotis-ai/workspaces/isaac_logistics_cell_ws/reports/camera_quality/after/preview_head_left.jpg)

## 실행과 선택 프로파일

현재 공식 기본 프로파일을 적용하고 GUI와 Cyclo를 재실행했습니다. 이후 기본 실행은 기존 CLI입니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh isaac
./runtime/hx5_isaac.sh cyclo  # 별도 터미널; 서비스가 내려가 있을 때
```

Head에 제조사가 지원하는 HD720 모드를 선택하려면 유휴 상태에서 아래 순서로 새 씬을 만듭니다. Wrist는 424×240 그대로이며 Head만 1280×720, nominal fx=fy=736으로 변경됩니다. **공식 launch 기본값은 VGA**이며, 이번 live 검증은 기본 프로파일에서 수행했습니다. HD720은 더 많은 렌더링·전송 비용이 필요합니다.

```bash
cd /home/robotis-ai/workspaces/hero/ai_worker_1044_sh5_ws
./runtime/hx5_isaac.sh sim-stop
HX5_ISAAC_CAMERA_PROFILE=hd720 /home/robotis-ai/workspaces/isaac_logistics_cell_ws/build.sh
docker exec ai_worker_1044_hx5_isaac /command/s6-rc -d change hx5_isaac_sidecar
docker exec ai_worker_1044_hx5_isaac /command/s6-rc -u change hx5_isaac_sidecar
./runtime/hx5_isaac.sh isaac
```

`HX5_ISAAC_CAMERA_PROFILE=official`로 build하면 기본 설정으로 돌아갑니다. 새 build 후 sidecar 재시작은 CameraInfo metadata를 다시 읽기 위한 단계입니다. 일반 `validate`는 scene metadata의 선택 해상도를 사용합니다. 아래의 품질 probe는 공식 기본 프로파일을 엄격하게 검사합니다.

```bash
./runtime/hx5_isaac.sh validate --duration 6
docker exec ai_worker_1044_hx5_isaac bash --noprofile --norc -c \
  'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; python3 /hx5_isaac_runtime/camera_quality_probe.py --output-dir /workspace/diagnostics/camera_quality_after --expect-official'
```

## 검증 근거

실제 GUI에서 네 ROS 원본과 CameraInfo를 8초 수신했습니다. 네 카메라 모두 해상도·frame ID·K/P·JPEG95 quantization table 및 약 30.000 SIM Hz 검사가 통과했습니다. **실시간 수신은 약 12.0 FPS**였으므로 시뮬레이션 30 Hz를 실시간 30 FPS로 판정하지 않습니다. UI 네 JPEG도 원본과 같은 해상도와 품질 95였고, frame age는 약 63–73 ms였습니다. 원본 HTTP 경로는 672×376 JPEG를 HTTP200으로 반환했습니다.

원본·미리보기·JSON 비교는 물류 워크스페이스 `reports/camera_quality/before/` 및 `after/`에 있습니다. AI 측 원본은 `simulation/isaac/ai_worker/diagnostics/camera_quality_before/`, `camera_quality_after/`입니다.

Scene 구조 636/636, matching USD의 scene·payload·camera geometry·camera profile 검사 52개, ROS/OpenCV의 preview 검사 24개가 통과했습니다. 실제 6초 연결 진단도 15개 topic의 관절·촉각·영상·CameraInfo·scan·clock/odom/TF 모두 통과했습니다. 보고서는 `simulation/isaac/ai_worker/diagnostics/isaac-camera-quality-final-contract.json`입니다.

수정 전 quality backup은 물류 워크스페이스 `revisions/20260918_before_camera_quality_fix/`입니다. 비교에서 로봇 collider 64개, 63축 설정·57축 초기 target, 72개 물건과 60개 바구니 proxy의 물리 속성, 카메라 mount, 환경·layout·saved pile이 유지됨을 확인했습니다. 이전 비산 안정화 설정을 유지했고 Gazebo 프로파일도 유지했습니다. Preview 기본값 424/45/12를 보존하고 Isaac만 환경 변수로 1280/95/12를 선택합니다.

## 공식 출처

- [ROBOTIS AI Worker 카메라 하드웨어 구성](https://ai.robotis.com/ai_worker/hardware_ai_worker)
- [ROBOTIS ZED Mini 기본 설정](https://github.com/ROBOTIS-GIT/ai_worker/blob/main/ffw_bringup/config/common/zedm.yaml)
- [ROBOTIS stereo publication 설정](https://github.com/ROBOTIS-GIT/ai_worker/blob/main/ffw_bringup/config/common/common_stereo.yaml)
- [ROBOTIS D405 RGB/depth launch 설정](https://github.com/ROBOTIS-GIT/ai_worker/blob/main/ffw_bringup/launch/camera_realsense.launch.py)
- [Stereolabs 모드별 focal length/FOV 표](https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view)
- [RealSense D405 시야각](https://www.realsenseai.com/product-family/d405-series/)
- [NVIDIA OpenCV pinhole 카메라](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_camera.html#opencv-pinhole)
- [ROS compressed image publisher JPEG 기본값](https://github.com/ros-perception/image_transport_plugins/blob/rolling/compressed_image_transport/src/compressed_publisher.cpp)
