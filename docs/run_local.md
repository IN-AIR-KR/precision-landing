# 로컬 실행 (Ubuntu 22.04 / ROS 2 Humble / Gazebo Harmonic)

기존 `~/PX4-Autopilot` 수정사항은 보존했습니다. 예제 전용 `~/PX4-precision-landing`
worktree는 README 지정 커밋 `86f2fdfd7de7f5e5d29ab61f60d81e892a557376`입니다.
OpenCV 4.10.0과 추가 gps_msgs는 `.deps` 아래에 설치했습니다.

## 실행

```bash
cd ~/precision-landing
./scripts/run.sh
```

`aruco_moving` 월드, Gazebo GUI, `/image_proc` 영상 창, DDS Agent(pl_bringup launch 소유), ROS 노드를 함께
실행합니다. 기존 `aruco_moving_target` 패드를 원점에서 X 방향 1.5m 떨어진 곳에 배치합니다.
마커 크기는 모델과 같은 1.0m입니다. 처음에 패드는 정지해 있습니다.

QGroundControl은 별도 터미널에서 실행합니다:

```bash
~/Applications/QGroundControl-*.AppImage
```

QGC에서 약 2.5m까지 이륙한 뒤 `PrecisionLandCustom` 모드를 선택합니다.
마커를 인식하고 접근하기 시작하면 다른 터미널에서 패드를 움직입니다:

```bash
cd ~/precision-landing
./scripts/move_target.sh 0.3
```

정지:

```bash
./scripts/move_target.sh 0 0
```

두 인자는 월드 좌표계 X/Y 방향 속도(m/s)입니다. 타겟이 탐색 범위 밖으로 나가지 않도록
인식 후 이동시키세요. 전체 종료는 실행 터미널에서 Ctrl+C입니다.
새 시험은 종료 후 `./scripts/run.sh`를 다시 실행하여 시작합니다.

## 월드와 모델

- `simulation/gz/worlds/aruco_moving.sdf`: 누락된 월드를 기존 문서의 배치 조건으로 추가했습니다.
  PX4 기본 `aruco.sdf`의 지면·조명·시뮬레이션 플러그인을 사용합니다.
- `simulation/gz/models/aruco_moving_target`: 저장소에 있던 움직이는 패드 모델을 그대로 사용합니다.
- `run.sh`가 월드를 전용 PX4 checkout의 `Tools/simulation/gz/worlds`로 복사합니다.
  기존 `~/PX4-Autopilot`은 수정하지 않습니다.

## 명령 환경과 재빌드

```bash
cd ~/precision-landing
source scripts/env.sh
export ROS_DOMAIN_ID=42 GZ_PARTITION=precision_landing
ros2 topic list
```

소스 수정 후에는 `./scripts/build.sh`로 재빌드합니다.
ROS Humble, Gazebo Harmonic, PX4 빌드 도구 및 MicroXRCEAgent가 설치된 환경에서
로컬 의존성과 checkout을 준비하려면 `./scripts/setup.sh`를 사용합니다.
`PX4_DIR`로 전용 checkout 경로를 바꿀 수 있습니다.
GUI 없이 실행하려면 `HEADLESS=1 ./scripts/run.sh`를 사용합니다.

## 자동 비행 시험

`run.sh`를 실행한 상태에서, 다른 터미널에서 실행합니다.
이 명령은 전용 로컬 SITL을 확인한 뒤 이륙·모드 전환·패드 이동을 자동 수행합니다.

```bash
cd ~/precision-landing
source scripts/env.sh
python3 scripts/test_flight.py
```

시험 로그는 `.run/moving-flight.jsonl`, ROS 로그는 `.run/ros.log`,
PX4 로그는 `.run/px4.log`에 기록됩니다.

## 검증 결과 (2026-11-10)

- PX4 SITL 및 필요한 ROS 패키지 빌드 완료, OpenCV 4.10 로딩 확인.
- 새 월드 SDF 검증 통과, 카메라 영상과 `PrecisionLandCustom` 모드 등록 확인.
- 0.3m/s 이동 명령을 유지한 상태에서 추적·착륙·자동 disarm 성공.
- `Search → Approach → Descend → Finished`, 마커 인식 145회 확인.
- 패드가 X=1.5m에서 약 4.20m로 이동. 착륙 후 Gazebo 실제 좌표 기준
  드론과 패드 중심의 수평 거리는 약 7.9cm. 이 결과는 이번 0.3m/s 시험 기준입니다.
- 시험 종료 시 패드를 정지시키고, 전체 시뮬레이터 프로세스도 종료했습니다.

## 0번 기준 데이터 기록

정적·이동 비행의 ROS bag 기록과 격리 재생은 [0번 실행 안내](baseline_stage0.md)를 참고하세요.

## 1번 공통 카메라 입력

입력 토픽과 카메라 장착 설정, CSI 보류 범위는 [1번 안내](camera_inputs_stage1.md)를 참고하세요.
