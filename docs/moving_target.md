# 움직이는 ArUco 타겟 (Moving Target)

정적인 ArUco 마커 대신, 바퀴 달린 차량 위에 더 큰 마커를 올려두고, 시뮬레이션이 실행 중일 때 명령어로 속도를 넣어줘야만 움직이게 만든 구성입니다.

## 구성 요소

### 모델: [`simulation/gz/models/aruco_moving_target`](../simulation/gz/models/aruco_moving_target)

- `chassis`(1.5 x 1.5 x 0.15m 박스) + 바퀴 4개(실린더, 반지름 0.2m)로 이루어진 차량. 바퀴는 `revolute` 조인트로 `chassis`에 붙어있고, 중력/바닥 접촉이 정상 작동합니다(물리적으로 바닥에 바퀴로 서 있는 차량입니다 — 떠 있지 않습니다).
- 바퀴 4개와 `ground_plane`(`aruco_moving.sdf`)의 마찰계수를 `mu: 0.8`로 맞춰뒀습니다 — 건조한 아스팔트+고무 타이어의 현실적인 마찰계수(대략 0.7~0.9) 범위를 대표하는 값입니다 (Gazebo 기본값 1.0은 실제 노면보다 그립이 더 좋은 비현실적인 값). ODE는 접촉하는 두 표면 중 낮은 쪽을 따라가므로, 바퀴/바닥 둘 다 같이 맞춰야 의미가 있습니다.
- `chassis` 위에 **1.0 x 1.0m** ArUco 마커 평면이 붙어 있습니다. **크기를 바꾸려면 `src/aruco_tracker/cfg/params.yaml`의 `marker_size`도 반드시 같이 바꿔야 합니다** — PnP 포즈 추정의 스케일 기준이기 때문입니다.
- `gz-sim-diff-drive-system`(`gz::sim::systems::DiffDrive`) 플러그인이 붙어 있습니다. Gazebo에 기본 내장된 플러그인이라 별도 빌드가 필요 없습니다. 힘/속도를 바디에 직접 꽂는 게 아니라, **좌/우 바퀴 조인트의 각속도를 계산해서 돌리고, 바퀴-바닥 마찰로 실제로 굴러서 움직입니다** — 바퀴가 눈에 보이게 회전합니다.
  - **커맨드를 아무것도 안 보내면 속도 0, 즉 가만히 있습니다.** (처음 준비 상태 = 정지)
  - 실행 중 아무 때나 아래 명령으로 속도를 바꿀 수 있습니다:
    ```bash
    # 동쪽으로 1 m/s
    gz topic -t "/model/aruco_moving_target/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}"

    # 회전하면서 전진 (좌/우 바퀴 각속도가 달라짐)
    gz topic -t "/model/aruco_moving_target/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.2}"

    # 정지
    gz topic -t "/model/aruco_moving_target/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0, y: 0}"
    ```
  - 차동 구동(differential drive) 방식이라 좌우 평행이동(`linear.y`)은 지원하지 않습니다 — 실제 자동차처럼 `linear.x`(전진/후진) + `angular.z`(회전)만 됩니다.
  - 플러그인에 가감속 한계(`max_linear_acceleration: 3 m/s²`)를 걸어뒀습니다. 이전 `VelocityControl` 방식은 명령을 주는 즉시 그 속도로 순간이동하듯 바뀌었는데, 지금은 실제 바퀴 마찰로 가속하는 데 시간이 걸립니다(더 현실적인 동작).

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

시뮬레이션이 뜨면 차량은 가만히 있습니다. 드론을 Precision Land 모드로 보내 탐색을 시작한 뒤(또는 그 전에), 다른 터미널에서 `gz topic` 명령으로 속도를 넣어서 움직이게 하세요.

ROS2 쪽 노드는 `world:=aruco_moving`을 반드시 붙여서 띄워야 카메라 토픽이 맞습니다 (월드 이름이 Gazebo 센서 토픽 경로에 포함되기 때문):

```bash
ros2 launch precision_land precision_landing_system.launch.py world:=aruco_moving
```

## PrecisionLand.cpp 변경 사항

기존 코드는 **정적 타겟을 가정**하고 있어서 그대로는 움직이는 마커에 착륙할 수 없었습니다. 누적된 수정 사항:

1. **타겟 칼만 필터** (`PrecisionLand.hpp`의 `_target_position`/`_target_velocity`/`_target_P`, `predictTarget()`/`correctTarget()`)
   - 등속(constant-velocity) 모델의 6상태(위치+속도) 칼만 필터. 매 제어 틱마다 `predictTarget()`으로 예측을 진행하고(탐지 여부 무관), `/target_pose`가 들어올 때만 `correctTarget()`으로 보정합니다.
   - 탐지가 잠깐 끊겨도(마커가 카메라 시야 밖으로 나가는 등) 예측으로 계속 추정을 이어가다가, `target_timeout` 동안 보정이 아예 없으면 실패 처리합니다.
   - 튜닝 파라미터: `target_accel_noise`(프로세스 노이즈, 타겟이 얼마나 급하게 가속/회전할 수 있다고 볼지), `target_pos_noise`(측정 노이즈, 탐지 흔들림 정도).

2. **`positionReached()`의 전환 조건 수정**
   - 기존: `드론 위치 - 타겟 위치 < delta_position` **그리고** `드론 자체 속도 < delta_velocity`.
   - 문제: 타겟이 계속 움직이면 드론도 계속 따라 움직여야 하므로, 드론의 "절대 속도"는 결코 0 근처로 떨어지지 않습니다 → `Approach` 상태에서 `Descend`로 영원히 전환되지 않습니다.
   - 수정: 드론 속도에서 **타겟 속도를 뺀 상대 속도**를 기준으로 판정합니다. 타겟이 감지되기 전(Search 단계)에는 타겟 속도가 0이므로 기존 동작과 동일하게 작동합니다.

3. **Approach 단계도 속도 제어로 전환**
   - 기존엔 `Approach`가 일반 position setpoint(`updatePosition()`)를 썼는데, PX4 내부 position controller는 움직이는 setpoint에 대한 feedforward가 없어서 타겟 속도가 빨라지면 구조적으로 못 따라잡았습니다(계속 등속 추적만 하고 `Descend`로 전환이 안 됨).
   - `withHorizontalVelocity() + withPositionZ()`로, Descend와 동일한 feedforward 포함 속도 제어기를 재사용하도록 변경. 고도만 `_approach_altitude`로 고정합니다.

4. **Descend 단계의 속도 feedforward** (`calculateVelocitySetpointXY()`)
   - 기존 PI 제어기는 `vx = -(Kp*e + Ki*I)`만 계산했습니다. 움직이는 타겟을 순수 PI로 쫓으면 정상상태 오차가 `타겟 속도 / Kp`만큼 항상 남습니다.
   - 추정한 타겟 속도를 feedforward로 더해 `vx = -(Kp*e + Ki*I) + v_target_x` 로 바꿔서 추적 지연을 크게 줄였습니다.

5. **`max_velocity`와 적분 와인드업 한계 분리**
   - 원래 적분항(I) 와인드업 방지 클램프가 `max_velocity`(출력 속도 상한)를 그대로 재사용하고 있었습니다. 빠른 타겟을 쫓으려고 `max_velocity`를 크게 올리면, 그 영향으로 와인드업 한계도 같이 풀려버리는 의도치 않은 결합이 있었습니다.
   - 별도 파라미터 `integral_limit`(기본 3.0)로 분리해서, 출력 속도 상한과 적분 누적 한계가 독립적으로 조절되게 했습니다.

## 파라미터 튜닝

- 타겟 속도가 빨라지면 `src/precision_land/cfg/params.yaml`의 `vel_p_gain`을 높이거나(응답성 ↑, 오버슈트 위험 ↑), `target_timeout`을 넉넉히(5초 이상) 주는 것이 안전합니다.
- `max_velocity`는 드론이 낼 수 있는 최종 속도 상한, `integral_limit`은 적분항 와인드업 한계 — 서로 독립적으로 조정하세요. `vel_i_gain`을 0이 아닌 값으로 켤 계획이면 `integral_limit`을 같이 봐야 합니다.
- `target_accel_noise`/`target_pos_noise`로 칼만 필터가 모델(등속 가정)과 측정 중 뭘 더 믿을지 조절합니다. 급가속/급회전하는 타겟을 쫓을 땐 `target_accel_noise`를 높이는 쪽이 반응성에 도움이 됩니다.
- 마커/차체 크기를 더 키우고 싶다면 `model.sdf`의 치수(`chassis` box, 마커 plane, 바퀴 위치)와 `aruco_tracker/cfg/params.yaml`의 `marker_size`를 같이 맞추세요.
- `descent_vel`이 낮고 접근 고도가 높을수록, 하강하는 동안 타겟을 쫓아 수평으로 이동하는 거리가 길어집니다(대략 `타겟속도 × 접근고도 / descent_vel`). 타겟이 빠를 때 착륙 지점이 너무 멀어지는 게 싫으면 `descent_vel`을 올리세요.

## 알려진 한계

- `DiffDrive`는 **명령으로 넣은 선속도/각속도**만큼만 움직입니다. 경로 추종이나 자동 가속/감속 같은 건 없고, 사람이 `gz topic`으로 직접 넣어줘야 합니다 (원하면 셸 스크립트나 ROS2 노드로 주기적으로 publish해서 자동화할 수 있습니다).
- `Search` 단계의 나선형 탐색 패턴은 여전히 정적 waypoint 기준입니다. 타겟이 탐색 범위(반경 2m) 밖으로 많이 벗어나면 탐색 중에 못 찾을 수 있습니다.
- 칼만 필터에 이상치(outlier) 게이팅이 없습니다 — 탐지가 잘못된 값을 내놓아도(모션 블러로 코너를 잘못 읽는 등) 그대로 보정에 반영됩니다. 실기체 테스트 전에 보강하면 좋은 부분입니다.
- 시뮬레이션은 실제보다 여러 면에서 관대합니다(카메라 노이즈/블러 없음, 바람 없음, 드론 상태 추정이 매우 정확함 등). 시뮬에서 빠른 속도로 착륙이 잘 된다고 해서 실기체에서 같은 속도가 바로 될 거라고 기대하면 안 됩니다.
