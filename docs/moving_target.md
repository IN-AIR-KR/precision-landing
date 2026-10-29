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
- 패드 위에 **1.0 x 1.0m** ArUco 마커 평면이 붙어 있습니다 (기존 0.5m에서 2배로 키움). **크기를 바꾸려면 `src/aruco_tracker/cfg/params.yaml`의 `marker_size`도 반드시 같이 바꿔야 합니다** — PnP 포즈 추정의 스케일 기준이기 때문입니다.
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

### 월드: `PX4-Autopilot/Tools/simulation/gz/worlds/aruco_moving.sdf`

- 기존 `aruco.sdf`(지면 + 조명만 있는 월드)를 복사하고, `arucotag` 모델 대신 `aruco_moving_target`을 `<pose>1.5 0 0 0 0 0</pose>`로 원점에서 1.5m 떨어진 곳에 include합니다.
  - 드론도 원점 근처에서 스폰되므로, 겹치지 않게 떨어뜨려 놓은 것입니다. 그러면서도 `PrecisionLand.cpp`의 나선 탐색 반경(2.0m) 안에는 들어옵니다.
- PX4 벤더 디렉터리에 새 파일 하나만 추가했을 뿐, 기존 PX4 파일은 건드리지 않았습니다. `PX4_GZ_WORLDS` 환경변수가 이 경로로 고정돼 있어서 월드 파일 자체는 이 위치에 있어야 합니다.
- 모델(텍스처 포함)은 `precision-landing` 저장소 안에 두고 `GZ_SIM_RESOURCE_PATH`로 참조합니다.

## 실행 방법

```bash
# PX4-Autopilot 디렉터리에서
GZ_SIM_RESOURCE_PATH=/path/to/precision-landing/simulation/gz/models:$GZ_SIM_RESOURCE_PATH \
PX4_GZ_WORLD=aruco_moving \
make px4_sitl gz_x500_mono_cam_down
```

시뮬레이션이 뜨면 패드는 가만히 있습니다. 드론을 Precision Land 모드로 보내 탐색을 시작한 뒤(또는 그 전에), 다른 터미널에서 `gz topic` 명령으로 속도를 넣어서 움직이게 하세요.

나머지(MicroXRCEAgent, `ros2 launch precision_land precision_landing_system.launch.py`)는 정적 마커일 때와 동일합니다.

## PrecisionLand.cpp 변경 사항

기존 코드는 **정적 타겟을 가정**하고 있어서 그대로는 움직이는 마커에 착륙할 수 없었습니다. 수정한 부분:

1. **타겟 속도 추정** ([`PrecisionLand.hpp`](../src/precision_land/PrecisionLand.hpp)의 `ArucoTag::velocity`, [`targetPoseCallback`](../src/precision_land/PrecisionLand.cpp))
   - 매 `/target_pose` 콜백마다 이전/현재 월드 좌표 차분으로 타겟의 월드 속도를 추정하고, 노이즈를 줄이기 위해 저역통과 필터(alpha=0.3)를 적용합니다.

2. **`positionReached()`의 전환 조건 수정**
   - 기존: `드론 위치 - 타겟 위치 < 0.25m` **그리고** `드론 자체 속도 < 0.25m/s`.
   - 문제: 타겟이 계속 움직이면 드론도 계속 따라 움직여야 하므로, 드론의 "절대 속도"는 결코 0 근처로 떨어지지 않습니다 → `Approach` 상태에서 `Descend`로 영원히 전환되지 않습니다.
   - 수정: 드론 속도에서 **타겟 속도를 뺀 상대 속도**를 기준으로 판정합니다. 타겟이 감지되기 전(Search 단계)에는 `_tag.velocity`가 0이므로 기존 동작과 동일하게 작동합니다.

3. **Descend 단계의 속도 feedforward** ([`calculateVelocitySetpointXY`](../src/precision_land/PrecisionLand.cpp))
   - 기존 PI 제어기는 `vx = -(Kp*e + Ki*I)`만 계산했습니다. 움직이는 타겟을 순수 P 제어로 쫓으면 정상상태 오차가 `타겟 속도 / Kp`만큼 항상 남습니다.
   - 추정한 타겟 속도를 feedforward로 더해 `vx = -(Kp*e + Ki*I) + v_tag_x` 로 바꿔서 추적 지연을 크게 줄였습니다.

## 파라미터 튜닝

- 타겟 속도가 빨라지면 `src/precision_land/cfg/params.yaml`의 `vel_p_gain`을 높이거나(응답성 ↑, 오버슈트 위험 ↑), `target_timeout`을 넉넉히(5초 이상) 주는 것이 안전합니다.
- 속도 추정 저역통과 필터 계수(`alpha = 0.3`, `PrecisionLand.cpp`의 `targetPoseCallback`)는 하드코딩되어 있습니다. 탐지 노이즈가 심하면 더 작게(예: 0.15), 반응이 느리게 느껴지면 더 크게 조정하세요.
- 마커/패드 크기를 더 키우고 싶다면 `model.sdf`의 두 치수(패드 box, 마커 plane)와 `aruco_tracker/cfg/params.yaml`의 `marker_size`를 같이 맞추세요.

## 알려진 한계

- `VelocityControl`은 **명령으로 넣은 선속도/각속도**만큼만 움직입니다. 경로 추종이나 자동 가속/감속 같은 건 없고, 사람이 `gz topic`으로 직접 넣어줘야 합니다 (원하면 셸 스크립트나 ROS2 노드로 주기적으로 publish해서 자동화할 수 있습니다).
- `Search` 단계의 나선형 탐색 패턴은 여전히 정적 waypoint 기준입니다. 타겟이 탐색 범위(반경 2m) 밖으로 많이 벗어나면 탐색 중에 못 찾을 수 있습니다.
- 실제 자동차 모델을 쓰고 싶다면 `marker_link`를 원하는 차량 메시로 바꾸고, 그 위에 `aruco_tag_visual`만 유지하면 됩니다. `VelocityControl` 플러그인은 모델 전체에 적용되므로 링크 구조를 바꿔도 그대로 동작합니다.
