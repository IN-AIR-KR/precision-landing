# 3번: 기존 PX4 체크아웃 호환과 Gazebo 실행 분리

사용자 선택에 따라 공식 릴리스 대신 기존 `~/PX4-Autopilot`의 현재 커밋을 기준으로 검증합니다.

| 구성 | 검증 커밋 |
|---|---|
| PX4 | `ee99864808e03c164975fc38c450fa22179fe0cb` |
| px4_msgs | `d3673f41976f04c64a5e559a61a441f348501913` |
| px4_ros2_cpp | `7338c1df755c20902dc65d5b844f347d877b5268` |

PX4의 사용자 차량·모델 변경사항은 유지했습니다. `setup.sh`는 기존 PX4를 checkout/reset하지 않습니다.
ROS 서브모듈은 위 조합으로 맞췄습니다. 사용 중인 상태·명령·등록·setpoint 설정 메시지 정의를
PX4 소스와 비교하고 인터페이스 라이브러리의 런타임 호환성 검사도 통과시킵니다.
`vehicle_status`는 `_v4`, `vehicle_command_ack`는 `_v1`로 변경되었습니다.
기존 bag의 카메라/포즈 재생은 가능하지만, 구버전 PX4 메시지를 새 타입으로 역직렬화해서는 안 됩니다.
기존 전체 비행 검증 결과는 당시 결과 파일로 보존합니다.

## 실행 구성

`run.sh`가 Gazebo 서버를 먼저 시작한 뒤 `PX4_GZ_STANDALONE=1`로 PX4를 연결합니다.
움직이는 월드는 이 저장소의 `simulation/gz/worlds/aruco_moving.sdf`에서 직접 읽습니다.
정지 `aruco` 월드와 기체 모델·플러그인은 PX4의 기존 리소스를 사용합니다.
PX4 폴더로 프로젝트 월드를 복사하지 않습니다.

카메라 센서 이름은 선택한 PX4에서 `camera`이므로 `sim.yaml`의 `bridge.link`, `bridge.sensor`로 지정합니다.
카메라 출력의 ROS 토픽은 `/camera/image_raw`, `/camera/camera_info`를 유지합니다.
PX4 파라미터·로그는 `$RUN_LOG_DIR/px4-rootfs`에 저장하여 기존 PX4 rootfs와 분리합니다.
프로세스 종료 시 run.sh가 자신이 시작한 프로세스 그룹만 종료합니다.

```bash
cd ~/precision-landing
export PX4_DIR="$HOME/PX4-Autopilot"
./scripts/run.sh
```

다른 터미널:

```bash
cd ~/precision-landing
export PX4_DIR="$HOME/PX4-Autopilot"
source scripts/env.sh
python3 scripts/test_flight.py             # 이동 0.3m/s, ROS 서비스로 시작
# python3 scripts/test_flight.py --speed 0 # 정지
```

기존 터미널에 예전 PX4_DIR가 남아 있을 수 있으므로 위처럼 경로를 명시합니다.
시험마다 run.sh를 재시작합니다. `test_flight.py`는 실행 파일과 프로젝트 안의 전용 작업 디렉터리를 확인합니다.
원래 PX4로 실행한 다른 SITL에 착륙 명령을 보내지 않도록 검사합니다.

## ROS 시작 서비스

이미 무장·이륙한 상태에서:

```bash
ros2 service call /pl/start_landing std_srvs/srv/Trigger '{}'
```

제어기는 등록 과정에서 받은 모드 ID를 사용합니다. 등록·최신 기체 상태·유효 위치·무장·비행 여부를
확인하고 모드 전환 명령을 보냅니다. 응답을 기다리는 동안 executor를 막지 않으며,
PX4가 실제 활성 상태를 보고하면 성공을 반환합니다. PX4 거절과 5초 활성화 확인 시간 초과를 구분합니다.
이미 활성화됐으면 FSM을 재시작하지 않습니다. 동시에 들어온 요청은 같은 전환 결과를 받습니다.

서비스는 무장·이륙을 수행하지 않습니다. QGC의 `PrecisionLandCustom` 선택도 같은 FSM을 실행합니다.
MAVLink 경로 자동 검증은 `test_flight.py --start-via mavlink`를 사용합니다.
이는 QGC가 사용하는 PX4 custom AUTO 모드 전환 프로토콜 검증이며 QGC 화면 직접 조작 시험은 아닙니다.

## 재현 검증

다른 SITL을 종료하고:

```bash
source scripts/env.sh
python3 scripts/record_baseline.py static
python3 scripts/record_baseline.py moving
python3 scripts/record_baseline.py moving --start-via mavlink
python3 scripts/test_mode_safety.py
```

안전 동작 시험은 별도 설정에서 하강 속도를 0.1m/s로 낮춥니다. 자신이 띄운 시뮬의 인식기를 잠시
멈춰 기본 5초 타겟 유실을 유발하고, 인식기를 복구한 다음 MAVLink Hold와 Land 전환을 확인합니다.
QGC 동작은 사용자 직접 시험으로 확인했습니다. 실기체 비행과 실물 RC 조작 검증은 보류입니다.

참고: [PX4 standalone Gazebo](https://docs.px4.io/main/en/sim_gazebo_gz/).

## 결과 (2026-11-10)

| 시험 | 결과 | 최종 수평 오차 |
|---|---|---|
| 정지 / ROS 서비스 | 착륙·자동 disarm 통과 | 0.006953 m |
| 이동 0.3m/s / ROS 서비스 | 착륙·자동 disarm 통과 | 0.045352 m |
| 이동 0.3m/s / MAVLink | 착륙·자동 disarm 통과 | 0.062857 m |

기록은 각각 `baselines/20261110-212322-static`, `212409-moving`, `212513-moving`에 있습니다.
세 bag 모두 접근 시 XY 속도+Z 위치 혼합 setpoint와 하강 속도·yaw 명령을 확인했습니다.
지상 요청 거부, 활성화 확인 응답, 활성 상태 반복 요청 및 FSM 재시작 없음도 통과했습니다.
카메라 장착 변환 단위 테스트 3개가 새 라이브러리 조합에서 통과했습니다.

타겟 유실·Hold/Land 회수 시험: `.run/mode-safety-20261110-212646/`.
기본 유실 타임아웃 뒤 Descend → Idle을 확인하고 외부 모드에서 Hold로 전환한 뒤 착륙했습니다.
별도 시작 검사에서 실제 PX4 cwd가 프로젝트의 px4-rootfs임을 확인했습니다.
요약은 `baselines/stage3-results.json`에 보존합니다.

이전 센서명 `imager`를 사용해 영상이 없었던 `212210-static`은 중단한 진단 실행이며 성공 결과에서 제외합니다.
