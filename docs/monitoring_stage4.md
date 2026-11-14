# 4번: 카메라·제어 상태 공통 모니터

`pl_viz`의 `monitor`가 원본 카메라 해상도를 유지하며 영상 위에 직접 정보를 겹쳐 표시합니다.
별도 상단·측면 패널은 없습니다.

- 좌상단: 대문자로 크게 착륙 상태. 비활성 상태는 INACTIVE.
- 상태 글자와 영상 가장자리 테두리: 대기(INACTIVE/IDLE)는 회색, 진행(SEARCH/APPROACH/DESCEND)은 노란색, 완료(FINISHED)는 초록색. 활성 중 타겟 유실 또는 제어 상태 미수신·갱신 지연이면 빨간색이 우선합니다.
- 우상단: GPS fix, 위성 수, 수평·수직 속도(m/s). GPS는 3D/DGPS/RTK fix일 때 초록색, 그 외(미수신·지연·추정 포함)는 빨간색. 수직 속도는 NED 아래쪽이 양수.
- 하단 가운데: 배터리 전압(V)과 잔량(%)을 나란히 표시. 미수신·지연·무효 값은 `--`.
- 중앙: 십자선. 영상 중심에서 검출한 타겟 마커 중심으로 굵은 노란 화살표.
- 마커: 굵은 초록 윤곽선과 ID. 위치·거리 패널과 3차원 축은 표시하지 않습니다.

화살표는 인식기가 해당 프레임의 코너 좌표로 그립니다. 최신 상태 메시지와 영상의 시각 차이로
화살표가 다른 프레임의 마커를 가리키지 않도록 했습니다. FSM 상태 글자는 테두리 없이 표시하고, 나머지 텍스트에는 검은 테두리를 사용합니다.

## 실행

```bash
cd ~/precision-landing
export PX4_DIR="$HOME/PX4-Autopilot"
./scripts/run.sh
```

Gazebo와 공통 모니터 창이 열립니다. 다른 터미널에서:

```bash
cd ~/precision-landing
source scripts/env.sh
python3 scripts/test_flight.py           # 이동 0.3 m/s
# python3 scripts/test_flight.py --speed 0  # 정지 (새 시뮬에서 실행)
```

모니터 창을 닫아도 인식·제어·시뮬은 계속 실행됩니다. 전체 종료는 run.sh 터미널에서 Ctrl+C입니다.
창을 다시 열려면:

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=42
ros2 run rqt_image_view rqt_image_view /pl/monitor/image
```

영상 아래가 아니라 rqt의 토픽 선택 목록에서 `/pl/monitor/image/compressed`를 선택하면
JPEG 전송을 사용합니다. 영상 없이 상태만 보려면:

```bash
ros2 topic echo /pl/monitor/status std_msgs/msg/String --qos-reliability best_effort
```

## 데이터와 지연 표시

| 입력 | 내용 |
|---|---|
| `/image_proc` 또는 `/image_proc/compressed` | 인식기가 마커 윤곽·ID·중심 화살표를 그린 영상 |
| `/pl/detection/status` | 프레임별 검출 여부, pose 유효 여부, ID, optical 위치·거리, 촬영 timestamp |
| `/pl/control/status` | 실제 FSM state, active, 제어기와 동일한 target_lost 판단, 등록된 mode_id |
| `/fmu/out/vehicle_local_position_v1` | 유효성이 확인된 수평·수직 속도 |
| `/fmu/out/vehicle_gps_position` (`SensorGps`) | fix type, satellites_used |
| `/fmu/out/battery_status_v1` | 연결 여부, voltage_v, remaining |

PX4 토픽 suffix는 설치된 메시지의 MESSAGE_VERSION을 사용합니다. 위 세 텔레메트리 토픽은
선택한 PX4 ee99864808의 dds_topics.yaml에 포함돼 있습니다. PX4 소스 수정은 하지 않습니다.
상태 메시지는 작은 `std_msgs/String` JSON이며 상태 값은 화면 출력을 위한 데이터입니다.
제어 경로는 이 메시지에 의존하지 않습니다.

`NO DATA`는 수신 없음, `STALE`은 마지막 수신 후 기본 2초 초과, `INVALID`는 PX4 유효성
조건 불충족을 뜻합니다. 화면은 마지막 영상을 유지하며 CAMERA: STALE 경고를 표시하고,
다른 상태는 계속 갱신합니다. 시간 경과는 monotonic clock을 사용하므로 시뮬 시간이 멈춰도
영상 유실 경고가 작동합니다. 상태와 영상은 각각 최신 수신값을 표시하며 정밀 시각 동기화나
제어용 보간은 수행하지 않습니다. 영상 메시지 header는 마지막 카메라 촬영 시각을 유지합니다.

## 구독과 부하

- `/pl/monitor/image`: 비압축 BGR8 합성 화면.
- `/pl/monitor/image/compressed`: JPEG 합성 화면. 해당 구독자가 있을 때만 JPEG 인코딩.
- `/pl/monitor/status`: 최신 상태·입력별 health/age를 담은 JSON. 해당 구독자가 있을 때만 발행.

모니터 출력 구독이 모두 사라지면 내부 구독도 해제합니다. 상태만 구독하면 카메라 디버그
영상을 구독하지 않습니다. 압축과 비압축 화면을 동시에 구독하면 합성은 한 번 하고 두 형식으로
발행합니다. 카메라 입력 전송 방식은 `viz.transport`로 별도 선택합니다.

인식기는 제어에 필요한 검출과 `/target_pose`를 계속 처리합니다. `/image_proc`와 압축 출력에
구독자가 없으면 마커 그리기·영상 발행·JPEG 인코딩을 생략합니다. 디버그 출력 속도를 제한해도
검출·pose 발행 주기는 바뀌지 않습니다. Gazebo의 `/image_proc` 브리지도 lazy 설정이므로
Gazebo 영상 소비자가 없을 때 ROS 영상을 구독하지 않습니다.

`sim.yaml`은 raw 입력, `real.yaml`은 compressed 입력이며 둘 다 기본 10 fps 모니터 출력을
사용합니다. `viz.fps`, `viz.jpeg_quality`, `viz.stale_after`로 출력 설정을 바꾸고,
`detection.debug_fps`, `detection.jpeg_quality`로 상류 디버그 영상 설정을 바꿉니다.

`./scripts/build.sh real`는 공통 모니터를 빌드하지만 Gazebo 마커 실행 파일은 제외합니다.
기존 `tag_pose_visualizer`는 `PL_VIZ_GAZEBO=ON`인 sim 빌드에만 포함됩니다. 이 선택 기능에는
시스템 `gz-transport13`/`gz-msgs10` 개발 라이브러리가 필요하며 공통 모니터는 Gazebo를 import/link하지 않습니다.
실기체 프로파일의 카메라 교정·장착 확인 가드는 유지합니다. CSI·실기체 화면 검증은 보류입니다.

## 자동 검증

```bash
source scripts/env.sh
ROS_DOMAIN_ID=85 python3 -m pytest -q src/pl_detection/test src/pl_viz/test
```

실제 ROS 노드 간 통신으로 raw/compressed 입력 각각에 대해 무구독, 상태만, raw만,
JPEG만, 동시 출력, 재구독, 카메라 유실 중 제어 상태 갱신을 검사합니다.
그리기/JPEG 인코딩 호출을 관측해 무구독 시 실행되지 않는지 확인하고,
출력 구독을 닫은 뒤에도 `/target_pose`가 계속 나오는지 확인합니다.

## 현재 검증 결과 (2026-11-10)

- sim 관련 5개 패키지 빌드 통과, Gazebo 옵션 OFF인 별도 pl_viz 빌드 통과.
- 기존 인식기 테스트와 raw/compressed ROS 통합 테스트 21개 통과.
- Gazebo 디버그 브리지의 실제 ROS 구독 수 `0 → 1 → 0` 확인. 해당 ros_gz 버전에서
  전역 CLI lazy 플래그가 적용되지 않아 별도 YAML 브리지를 사용하고, best-effort 영상
  발행자와 연결되도록 구독 QoS를 명시했습니다.
- 생성한 HUD 이미지를 직접 확인했습니다. 이 화면 검사는 합성 카메라/텔레메트리 테스트입니다.
- 새 구성의 실제 SITL 착륙은 별도 ARMS PX4가 실행 중이어서 대기 중입니다.
  이전 3단계의 성공 결과를 4단계 비행 검증 결과로 간주하지 않습니다.
- CSI·실기체 화면 검증은 기존 보류 범위입니다.
