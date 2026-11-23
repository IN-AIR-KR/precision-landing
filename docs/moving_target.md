# 움직이는 ArUco 타겟 (Moving Target)

정적인 ArUco 마커 대신, 평평한 패드 위에 더 큰 마커를 올려두고, 시뮬레이션이 실행 중일 때 명령어로 속도를 넣어줘야만 움직이게 만든 구성입니다. "자동차"가 아니어도 되고, 핵심은 마커가 월드 좌표계에서 이동할 수 있다는 점입니다.

## 왜 이런 구조인가 (히스토리)

처음에는 PX4 공식 `MovingPlatformController` 플러그인(배/트럭 갑판 이착륙 데모용, 힘(wrench)으로 중력을 상쇄해서 특정 높이에 "띄워놓는" 방식)을 재사용했습니다. 두 가지 문제가 있었습니다:

1. **드론과 겹쳐서 스폰됨**: 플랫폼과 드론이 둘 다 월드 원점(0,0,0) 근처에서 스폰되면서 물리적으로 겹쳐(interpenetration) ODE가 다리처럼 늘어진 이상한 형태를 만들어냈습니다.
2. **공중에 떠 있는 느낌**: `MovingPlatformController`는 바닥과 접촉하는 게 아니라 PD 제어로 특정 높이를 "유지"하는 방식이라, 바닥과 패드 사이에 뜬 간격이 보이고, 시작하자마자(환경변수로 설정한) 속도로 바로 움직이기 시작했습니다. 실행 중에 속도를 바꾸거나 멈추는 것도 안 됐습니다(설정은 시작 시점에 한 번 고정).

그래서 지금은 **Gazebo 표준 `VelocityControl` 플러그인**으로 바꿨습니다. 커스텀 PX4 플러그인이 전혀 필요 없고, 동작이 훨씬 단순/직관적입니다.

## 구성 요소

### 모델: [`simulation/gz/models/aruco_moving_target`](../simulation/gz/models/aruco_moving_target)

- `marker_link` 하나짜리 단순 모델. 1.5 x 1.5 x 0.1m 평평한 패드, 질량 50kg, 중력/바닥 접촉 정상 작동 (물리적으로 바닥에 그냥 놓여 있는 상자입니다. 뜨는 게 아닙니다).
- 패드 위에 **0.30 × 0.30 m** ArUco 마커 평면이 붙어 있습니다. **크기를 바꾸려면 `src/pl_bringup/config/sim.yaml`의 `detection.marker_size`도 반드시 같이 바꿔야 합니다** — PnP 포즈 추정의 스케일 기준이기 때문입니다.
- `gz-sim-velocity-control-system` (`gz::sim::systems::VelocityControl`) 플러그인이 붙어 있습니다. 이건 Gazebo에 기본 내장된 플러그인이라 별도 빌드가 필요 없습니다.
  - **커맨드를 아무것도 안 보내면 속도 0, 즉 가만히 있습니다.** (처음 준비 상태 = 정지)
  - 실행 중 아무 때나 아래 명령으로 속도를 바꿀 수 있습니다:
    ```bash
    # 동쪽으로 1 m/s
    gz topic -t "/model/aruco_moving_target/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}"

    # 대각선으로
    gz topic -t "/model/aruco_moving_target/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.7, y: 0.7}"

    # 제자리 회전까지 테스트하고 싶다면 (yaw 추적 확인용)
    gz topic -t "/model/aruco_moving_target/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.2}"

    # 정지
    gz topic -t "/model/aruco_moving_target/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0, y: 0}"
    ```
  - 속도는 world frame(ENU) 기준입니다.

### 월드: `simulation/gz/worlds/aruco_moving.sdf`

- `aruco_moving_target`을 원점에서 월드 +X 방향으로 1.5 m 떨어진 곳에 배치합니다. 기체는 원점에서 이륙합니다. 패드는 기체 이륙과 정밀착륙 모드 활성화 및 마커 재검출을 확인할 때까지 정지합니다.
- `scripts/run.sh`는 저장소에 있는 월드 파일을 직접 사용합니다.
- 모델(텍스처 포함)은 `precision-landing` 저장소 안에 두고 `GZ_SIM_RESOURCE_PATH`로 참조합니다.

## 실행 방법

이 컴퓨터에서는 `./scripts/run.sh`로 전체 실행하고, `./scripts/move_target.sh 0.3`으로 패드를 움직일 수 있습니다. [로컬 실행 안내](run_local.md)를 참고하세요. 아래 수동 명령은 월드 파일을 PX4의 `Tools/simulation/gz/worlds/`에 먼저 복사한 경우에 사용합니다.

```bash
# PX4-Autopilot 디렉터리에서
GZ_SIM_RESOURCE_PATH=/path/to/precision-landing/simulation/gz/models:$GZ_SIM_RESOURCE_PATH \
PX4_GZ_WORLD=aruco_moving \
make px4_sitl gz_x500_mono_cam_down
```

시뮬레이션이 뜨면 패드는 가만히 있습니다. 이륙하고 Precision Land 모드를 활성화한 뒤, 다른 터미널에서 `gz topic` 명령으로 패드를 움직이세요.

Gazebo와 PX4를 별도로 실행할 때는 [README의 스택 분리 실행](../README.md#62-gazebopx4ros-스택-분리-실행)을 따릅니다. ROS 스택은 `ros2 launch pl_bringup sim.launch.py world:=aruco_moving`으로 실행합니다.

## 타겟 추정과 추종

- `/pl/target_observation`의 마커 관측을 카메라 장착 변환과 기체 pose로 NED 월드 좌표로 변환합니다.
- Kalman 필터로 타겟 위치·속도를 추정하고, 제어 주기마다 예측합니다.
- PID 제어기는 `ALIGN`에서 고도를 유지하며 타겟을 추종하고, 위치 오차와 상대 속도가 허용 범위에 들면 `APPROACH`에서 하강합니다.
- 수평 속도 명령에 추정 타겟 속도를 feedforward로 더합니다. 상태 전이와 제어식은 [README](../README.md#4-착륙-fsm)를 참고하세요.

## 파라미터 튜닝

- 수평 추종 게인은 `src/pl_bringup/config/sim.yaml`의 `control.horizontal_tracking`에서 설정합니다.
- 위치·속도 추정의 노이즈 값과 마커 유실 허용 시간은 `control.target_estimation`에서 설정합니다.
- 마커 크기를 바꾸면 모델의 `aruco_tag_visual` 평면 크기와 `detection.marker_size`를 함께 맞춥니다. 패드 크기는 마커가 들어갈 수 있도록 정합니다.

## 알려진 한계

- `VelocityControl`은 **명령으로 넣은 선속도/각속도**만큼만 움직입니다. 경로 추종이나 자동 가속/감속 같은 건 없고, 사람이 `gz topic`으로 직접 넣어줘야 합니다 (원하면 셸 스크립트나 ROS2 노드로 주기적으로 publish해서 자동화할 수 있습니다).
- PID 제어기의 `IDLE`은 현재 위치를 유지하며 마커 검출을 기다립니다. 마커를 카메라 시야 안에 배치해야 합니다.
- 실제 자동차 모델을 쓰고 싶다면 `marker_link`를 원하는 차량 메시로 바꾸고, 그 위에 `aruco_tag_visual`만 유지하면 됩니다. `VelocityControl` 플러그인은 모델 전체에 적용되므로 링크 구조를 바꿔도 그대로 동작합니다.
