# Isaac Mission Canvas의 실제 브라우저 Teleop 확인 (2026-09-18)

사용자가 **Gazebo UI 포트 7380에 접속했던 것**이 Isaac 로봇의 “움직임 없음” 원인으로 확인됐다. 사용자는 **Isaac UI 포트 7880에서는 현재 정상 동작한다**고 답했다. 두 UI는 서로 다른 시뮬레이션에 연결되므로 Isaac 제어에는 `http://127.0.0.1:7880`을 사용한다.

독립 Chrome 프로필로 7880의 실제 React 화면도 확인했다. 13:43 KST의 실행에서는 Mobile Teleop 명령이 전송됐고 실제 로봇 위치도 변했다. 아래 기록은 이 경로의 동작 증거이며, 브라우저 장애를 원인으로 판단한 자료가 아니다. 사용자 브라우저의 캐시와 세션 자체는 직접 조사하지 않았다.

## 실제 화면과 연결

- 독립 프로필: `/tmp/hx5-isaac-ui-browser`, 임시 CDP 포트: `9227`. 사용자 Chrome 프로필은 사용하지 않았다.
- 초기 화면에서 `ffw_sh5_rev1`, `Connected`, Cameras 4/4, 양손 Tactile Live를 관측했다. Joint/clock은 약 39 Hz였다.
- 실제 mouse 입력으로 Autonomy Studio → Mission Canvas → Mobile Teleop Activate를 눌렀다. Activate 후 W 버튼과 속도 range control이 활성화됐다.
- 실제 range control에 Home 키를 보내 Linear 최소값 `0.05`를 확인한 후, W 키를 1006 ms 누르고 key-up 및 Space로 정지했다. Start Mapping, Navigation, Run, Recording은 실행하지 않았다.
- Chrome이 연결한 실제 WebSocket은 `ws://127.0.0.1:7890/`이었다. 브라우저에서 `/cmd_vel`의 `linear.x=0.05` 명령 6개, 정지 명령 2개를 관측했다. 이후 Deactivate로 정지 명령을 1개 더 보냈다.
- Runtime exception, console error, WebSocket frame error는 0개였다.

## 실제 이동과 정지

W 입력 직후 `/odom` position은 x 약 0.14431 m에서 0.17988 m로 변했다. 다만 key-up 직후 1.2초 시점의 reported `linear.x`는 약 0.35082 m/s였고, 이후 13:43:42에는 x 약 0.58779 m에 도달했다. 이는 1초간 0.05 m/s 명령으로 예상되는 이동보다 크다. 브라우저는 위의 6개 이동 및 3개 정지 명령 외 추가 이동 명령을 보내지 않았지만, **별도 클라이언트의 동시 조작 여부를 관측할 수 없었다.** 다른 실행 경로의 명령, 당시 새로 시작된 CELL runtime, 물리 접촉 등을 이 브라우저 로그만으로 구분할 수 없으므로 이 관측의 원인은 미확정이다. 이 기록만으로 Teleop 정지 결함을 확정하거나, 이미 확인된 UI 포트 선택 문제와 연결하지 않는다.

Deactivate 후 13:44:13.162–13:44:16.268 KST에 3초간 명령 없이 다시 관측했다. 실제 planar drift는 **0.00001297 m**, 마지막 linear velocity는 약 `(-0.0000010, -0.0000050)` m/s였다. 이 구간에는 로봇이 정지해 있었고, 화면도 Mobile Teleop Inactive 및 Upper Body Jog Inactive였다.

검사가 끝난 뒤 독립 Chrome만 `Browser.close`로 종료했다. 종료 코드 0과 9227 포트가 비어 있음을 확인했다. Isaac 재시작/reset, scene 변경, joint 제어는 수행하지 않았다.

이 확인은 실제 UI의 Activate 및 keyboard W 경로에 한정된다. Pointer로 버튼을 계속 누르는 경우나 Rotate/Nav Mission 전체 실행의 성공을 이 검사로 주장하지 않는다.

## 보존한 근거

- 요약 및 command/pose 수치: `/tmp/hx5-isaac-ui-browser.summary.json`
- DOM, screenshot, CDP console/WebSocket 로그: `/tmp/hx5-isaac-ui-browser-artifacts/`
- 임시 CDP 검사 도구: `/tmp/hx5-isaac-ui-browser.cjs`
- 독립 Chrome 실행 로그: `/tmp/hx5-isaac-ui-browser.chrome.log`

Steering 관절 한계에서 실제 긴 회전을 wrap된 작은 오차로 잘못 판단하던 별도 코드 문제는 `control_math.py`에서 수정했고, 순수 회귀 테스트 14개가 통과했다. 사용자의 “움직임 없음”은 Gazebo/Isaac UI 포트 선택 문제로 해결됐으며, 이 경계 수정은 별도 개선이다.
