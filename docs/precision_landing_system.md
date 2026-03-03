# 정밀 착륙 시스템 (Precision Landing System)

## 목차

1. [시스템 개요](#시스템-개요)
2. [시스템 아키텍처](#시스템-아키텍처)
3. [ROS 노드 구성](#ros-노드-구성)
4. [정밀 착륙 알고리즘](#정밀-착륙-알고리즘)
5. [좌표계 변환](#좌표계-변환)
6. [파라미터 설정](#파라미터-설정)
7. [메시지 흐름도](#메시지-흐름도)
8. [실행 방법](#실행-방법)
9. [디버깅 및 모니터링](#디버깅-및-모니터링)
10. [트러블슈팅](#트러블슈팅)

---

## 시스템 개요

정밀 착륙 시스템은 드론이 ArUco 마커를 인식하여 목표 지점에 정확하게 착륙할 수 있도록 하는 자동 착륙 시스템입니다. 카메라로 지상의 ArUco 마커를 감지하고, 마커의 위치를 추적하면서 안전하게 착륙합니다.

### 주요 기능

- **자동 마커 탐색**: 나선형 패턴으로 비행하며 ArUco 마커 탐색
- **정밀 접근**: 마커 위치로 수평 이동
- **추적 하강**: 마커를 실시간으로 추적하며 안전하게 하강
- **시각화**: Gazebo 시뮬레이터에서 실시간 시각화 지원

---

## 시스템 아키텍처

```
┌────────────────────────────────────────────────────────────────────┐
│                           Gazebo Simulator                         │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐      │
│  │  Camera      │      │  ArUco       │      │  Drone       │      │
│  │  (Downward)  │      │  Marker      │      │  (x500)      │      │
│  └──────┬───────┘      └──────────────┘      └───────┬──────┘      │
└─────────┼────────────────────────────────────────────┼─────────────┘
          │                                            │
          │ Image + CameraInfo                         │ Odometry
          │                                            │
┌─────────▼────────────────────────────────────────────▼──────────────┐
│                          ROS 2 Network                              │
└─────────┬──────────────────────────┬──────────────────┬─────────────┘
          │                          │                  │
┌─────────▼─────────┐     ┌──────────▼──────────┐       │
│  aruco_tracker    │     │  precision_land     │       │
│  ┌──────────────┐ │     │  ┌───────────────┐  │       │
│  │ ArUco        │ │     │  │ State Machine │  │       │
│  │ Detection    │ │     │  │ - Search      │  │       │
│  └──────┬───────┘ │     │  │ - Approach    │  │       │
│         │         │     │  │ - Descend     │  │       │
│   /target_pose    │     │  └───────┬───────┘  │       │
│         │         │     │          │          │       │
│         └─────────┼─────┼──────────┘          │       │
└───────────────────┘     │                     │       │
                          │ /trajectory_setpoint│       │
                          └─────────────────────┼───────┘
                                                │
                          ┌─────────────────────▼─────────────┐
                          │         PX4 Autopilot             │
                          │  (Flight Controller)              │
                          └───────────────────────────────────┘
```

---

## ROS 노드 구성

### 1. **ros_gz_bridge** (3개 인스턴스)

Gazebo 시뮬레이터와 ROS 2 간의 메시지 브릿지 역할을 수행합니다.

#### 1.1 image_bridge

- **역할**: 카메라 영상을 Gazebo에서 ROS 2로 전송
- **토픽**:
  - Gazebo → ROS 2: `/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image`
  - 메시지 타입: `sensor_msgs/msg/Image`

#### 1.2 camera_info_bridge

- **역할**: 카메라 보정 정보를 Gazebo에서 ROS 2로 전송
- **토픽**:
  - Gazebo → ROS 2: `/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info`
  - 메시지 타입: `sensor_msgs/msg/CameraInfo`

#### 1.3 image_proc_bridge

- **역할**: 처리된 영상을 ROS 2에서 Gazebo로 전송 (시각화용)
- **토픽**:
  - ROS 2 → Gazebo: `/image_proc`
  - 메시지 타입: `sensor_msgs/msg/Image`
- **QoS**: Best Effort (실시간 처리 우선)

### 2. **aruco_tracker**

ArUco 마커를 감지하고 위치를 추정하는 노드입니다.

#### 입력 (Subscriptions)

- `/world/aruco/.../image`: 카메라 영상
- `/world/aruco/.../camera_info`: 카메라 보정 정보

#### 출력 (Publications)

- `/target_pose` (`geometry_msgs/msg/PoseStamped`)
  - **frame_id**: `camera_optical_frame` (카메라 광학 좌표계)
  - **내용**: ArUco 마커의 3D 위치 및 방향
  - **좌표계**: 카메라 기준 좌표 (X: 우, Y: 하, Z: 전방)

#### 설정 파일

- `aruco_tracker/cfg/params.yaml`

### 3. **precision_land**

정밀 착륙을 수행하는 메인 노드입니다. PX4의 커스텀 비행 모드로 구현되어 있습니다.

#### 입력 (Subscriptions)

1. `/target_pose` (`geometry_msgs/msg/PoseStamped`)
   - aruco_tracker로부터 마커 위치 수신
   - QoS: Best Effort
2. `/fmu/out/vehicle_land_detected` (`px4_msgs/msg/VehicleLandDetected`)
   - PX4로부터 착륙 감지 상태 수신
   - 착륙 완료 판단에 사용

#### 출력 (Publications)

1. `/target_pose_world` (`geometry_msgs/msg/PoseStamped`)
   - **frame_id**: `map` (NED 월드 좌표계)
   - **내용**: 월드 좌표계로 변환된 마커 위치
   - **용도**: 시각화 및 디버깅

#### PX4 인터페이스

1. **TrajectorySetpointType** (출력)
   - 위치/속도 명령을 PX4에 전송
   - Position mode 또는 Velocity mode로 제어
2. **OdometryLocalPosition** (입력)
   - 드론의 현재 위치 (NED 좌표계)
   - 드론의 현재 속도
3. **OdometryAttitude** (입력)
   - 드론의 현재 자세 (쿼터니언)

#### 설정 파일

- `precision_land/cfg/params.yaml`

### 4. **tag_pose_visualizer** (선택적)

시뮬레이션 환경에서 마커 위치를 시각화하는 노드입니다.

#### 입력

- `/target_pose_world`: 월드 좌표계의 마커 위치

#### 출력

- Gazebo 마커 메시지 (시각화용)

#### 활성화 조건

- `enable_gazebo_viz:=true` 일 때만 실행

---

## 정밀 착륙 알고리즘

### 상태 머신 (State Machine)

정밀 착륙은 5개의 상태로 구성된 유한 상태 머신으로 동작합니다.

- 모드 시작 시 바로 `Search`로 진입합니다.
- `Search`에서 모든 waypoint를 순회한 경우 처음부터 다시 순회합니다.
- `Approach`, `Descend`, 상태에서 `tag_lost` 발생 시 실패로 모드 종류 후 `Idle` 상태로 진입합니다.

```
    ┌──────────┐
    │   Idle   │
    └────┬─────┘
         │ onActivate()
         ▼
    ┌──────────┐
    │  Search  │
    └────┬─────┘
         │ Tag Detected
         ▼
    ┌──────────┐
    │ Approach │
    └────┬─────┘
         │ Position Reached
         ▼
    ┌──────────┐
    │ Descend  │
    └────┬─────┘
         │ Landed
         ▼
    ┌──────────┐
    │ Finished │
    └──────────┘
```

### 1. **Idle 상태**

- **목적**: 대기 상태
- **동작**: 아무 작업 수행 안 함
- **전환 조건**: 모드 활성화 시 Search로 전환

### 2. **Search 상태** (탐색)

ArUco 마커를 찾기 위해 나선형 패턴으로 비행합니다.

#### Waypoint 생성 알고리즘

```
초기 위치: (0, 0, z_current)
최대 반경: r_max = 2.0m
레이어 간격: Δz = 0.5m
레이어당 포인트 수: N = 16
```

나선형 패턴:

- **바깥쪽 나선**: 반경을 0에서 $r_{max}$까지 증가시키며 회전
- **안쪽 나선**: 반경을 $r_{max}$에서 0으로 감소시키며 회전

각 레이어 $l$에 대해:

$$
\begin{align}
\text{반경:} \quad r_i &= \frac{r_{max}}{N} \cdot i, \quad i = 0, 1, ..., N \\
\text{각도:} \quad \theta_i &= \frac{2\pi i}{N} \\
\text{위치:} \quad x_i &= r_i \cos(\theta_i) \\
y_i &= r_i \sin(\theta_i) \\
z_{\text{out}} &= z_{\text{current}} + l \cdot 2\Delta z \\
z_{\text{in}} &= z_{\text{current}} + (l \cdot 2 + 1) \cdot \Delta z
\end{align}
$$

#### 제어 방식

- **제어 모드**: Position Setpoint
- **목표**: 각 waypoint 순차적으로 방문
- **완료 조건**:
  - 위치 도달: $\|\mathbf{p}_{\text{target}} - \mathbf{p}_{\text{vehicle}}\| < \delta_p$ (기본값: 0.25m)
  - 속도 안정: $\|\mathbf{v}_{\text{vehicle}}\| < \delta_v$ (기본값: 0.25m/s)

#### 전환 조건

- **→ Approach**: ArUco 마커 감지 시
- waypoint 순환: 모든 waypoint 방문 후 처음부터 반복

### 3. **Approach 상태** (접근)

마커 바로 위로 수평 이동합니다.

#### 목표 위치 계산

$$
\mathbf{p}_{\text{target}} = \begin{bmatrix} x_{\text{tag}} \\ y_{\text{tag}} \\ z_{\text{approach}} \end{bmatrix}
$$

여기서:

- $(x_{\text{tag}}, y_{\text{tag}})$: 마커의 월드 좌표 (NED)
- $z_{\text{approach}}$: Search에서 마커를 발견했을 때의 고도

#### 제어 방식

- **제어 모드**: Position Setpoint
- **고도 유지**: 마커 발견 시점의 고도 유지

#### 전환 조건

- **→ Descend**: 목표 위치 도달 시 ($\delta_p < 0.25\text{m}$, $\delta_v < 0.25\text{m/s}$)
- **→ Idle (실패)**: 마커 추적 실패 시 (타임아웃 > 3초)

### 4. **Descend 상태** (하강)

마커를 추적하며 천천히 하강합니다. 가장 중요한 단계입니다.

#### 제어 알고리즘: PI 제어기

수평 방향(X, Y)에 대해 PI 제어기를 사용하여 속도 명령을 생성합니다.

##### 위치 오차 (Position Error)

$$
\begin{align}
e_x &= x_{\text{vehicle}} - x_{\text{tag}} \\
e_y &= y_{\text{vehicle}} - y_{\text{tag}}
\end{align}
$$

##### 적분항 (Integral Term)

$$
\begin{align}
I_x(t) &= I_x(t-1) + e_x \\
I_y(t) &= I_y(t-1) + e_y
\end{align}
$$

적분 와인드업 방지 (Anti-windup):

$$
I_x, I_y \in [-v_{\max}, v_{\max}]
$$

##### 속도 명령 계산

$$
\begin{align}
v_x &= -(K_p \cdot e_x + K_i \cdot I_x) \\
v_y &= -(K_p \cdot e_y + K_i \cdot I_y) \\
v_z &= v_{\text{descent}}
\end{align}
$$

파라미터:

- $K_p$: 비례 게인 (기본값: 1.5)
- $K_i$: 적분 게인 (기본값: 0.0)
- $v_{\text{descent}}$: 하강 속도 (기본값: 1.0 m/s, 양수 = 하강)

##### 속도 제한 (Saturation)

$$
v_x, v_y \in [-v_{\max}, v_{\max}]
$$

여기서 $v_{\max} = 3.0$ m/s (기본값)

#### Yaw 제어

드론의 방향을 마커의 방향과 정렬:

$$
\psi_{\text{setpoint}} = \text{yaw}(q_{\text{tag}})
$$

여기서 $q_{\text{tag}}$는 마커의 쿼터니언

#### 제어 블록 다이어그램

```
                                  ┌──────────────────┐
Target Position ────┐             │                  │
(x_tag, y_tag)      │             │   PI Controller  │
                    ├─► (-) ─────▶│   Kp = 1.5       │───► Velocity
Vehicle Position ───┘             │   Ki = 0.0       │     Command
(x_vehicle, y_vehicle)            │                  │     (vx, vy)
                                  └──────────────────┘
                                           │
                                           ▼
                                  ┌──────────────────┐
                                  │   Saturation     │
                                  │  [-3.0, 3.0] m/s │
                                  └──────────────────┘
                                           │
                                           ▼
Descent Velocity ──────────────────────────┴─────────────► (vx, vy, vz)
(vz = 1.0 m/s)                                              to PX4
```

#### 전환 조건

- **→ Finished**: `vehicle_land_detected` == true
- **→ Idle (실패)**: 마커 추적 실패 시 (타임아웃 > 3초)

### 5. **Finished 상태**

착륙이 성공적으로 완료되었음을 PX4에 보고합니다.

```cpp
ModeBase::completed(px4_ros2::Result::Success);
```

---

## 좌표계 변환

정밀 착륙 시스템은 3개의 좌표계를 사용합니다.

### 좌표계 정의

#### 1. **Optical Frame** (카메라 광학 좌표계)

```
  Y (Down)
  │
  │
  └───── X (Right)
 ╱
Z (Forward, away from lens)
```

- aruco_tracker가 출력하는 좌표계
- OpenCV/ROS 표준 카메라 좌표계
- z 방향: ⊗

#### 2. **Body Frame** (드론 본체 좌표계)

```
  X (Forward)
  │
  │
  └───── Y (Right)
 ╱
Z (Down)
```

- FRD (Front-Right-Down)
- 드론 중심 기준
- z 방향: ⊗

#### 3. **NED Frame** (월드 좌표계)

```
  N (North)
  │
  │
  └───── E (East)
 ╱
D (Down)
```

- PX4가 사용하는 글로벌 좌표계
- 위치 제어에 사용

### 변환 과정

ArUco 마커의 위치를 카메라 좌표계에서 월드 좌표계로 변환:

#### Step 1: Optical → Camera NED

회전 행렬:

$$
R_{\text{opt→ned}} = \begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

위 행렬은 카메라가 드론 하단을 향해 수직으로 장착된 경우를 고려한 변환입니다.

$$
\begin{bmatrix} x_{ned} \\ y_{ned} \\ z_{ned} \end{bmatrix} = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_{opt} \\ y_{opt} \\ z_{opt} \end{bmatrix} = \begin{bmatrix} -y_{opt} \\ x_{opt} \\ z_{opt} \end{bmatrix}
$$

이 변환이 의미하는 바는 다음과 같습니다:

- $x_{ned} = -y_{opt}$: Optical의 '아래'를 반전시켜 NED의 '앞'으로 보냄 (카메라가 바닥을 보고 있을 때)
- $y_{ned} = x_{opt}$: Optical의 '오른쪽'을 그대로 NED의 '오른쪽'으로 사용
- $z_{ned} = z_{opt}$: Optical의 '렌즈 방향(앞)'을 NED의 '아래'로 사용

#### Step 2: 변환 체인

```
Tag (Camera Optical) → Tag (Camera NED) → Tag (Body) → Tag (World NED)
```

전체 변환:

$$
T_{\text{tag}}^{\text{world}} = T_{\text{vehicle}}^{\text{world}} \cdot T_{\text{camera}}^{\text{body}} \cdot T_{\text{tag}}^{\text{camera}}
$$

각 변환:

- $T_{\text{vehicle}}^{\text{world}}$: 드론의 위치와 자세 (from OdometryLocalPosition, OdometryAttitude)
- $T_{\text{camera}}^{\text{body}}$: 카메라의 드론 본체 기준 위치 (고정값: [0, 0, -0.1]m)
- $T_{\text{tag}}^{\text{camera}}$: ArUco 마커의 카메라 기준 위치 (from aruco_tracker)

#### 코드 구현

```cpp
// Optical to NED rotation
Eigen::Matrix3d R;
R << 0, -1, 0,
     1,  0, 0,
     0,  0, 1;
Eigen::Quaterniond quat_NED(R);

// Vehicle pose
auto vehicle_position = _vehicle_local_position->positionNed();
auto vehicle_orientation = _vehicle_attitude->attitude();

// Build transformation chain
Eigen::Affine3d drone_transform =
    Eigen::Translation3d(vehicle_position) * vehicle_orientation;

Eigen::Affine3d camera_transform =
    Eigen::Translation3d(0, 0, -0.1) * quat_NED;

Eigen::Affine3d tag_transform =
    Eigen::Translation3d(tag.position) * tag.orientation;

// Final world transform
Eigen::Affine3d tag_world_transform =
    drone_transform * camera_transform * tag_transform;
```

- 드론이 기울어져서 카메라에 마커가 기울어져서 찍히더라도 `vehicle_orientation`과 `tag.orientation`이 곱해지면서 서로 상쇄되므로(=항등행렬이 되므로) 드론의 기울기 상관없이 마커의 월드 좌표가 계산됨

---

## 파라미터 설정

### precision_land 파라미터

| 파라미터         | 타입  | 기본값 | 설명                        |
| ---------------- | ----- | ------ | --------------------------- |
| `descent_vel`    | float | 1.0    | 하강 속도 (m/s)             |
| `vel_p_gain`     | float | 1.5    | 속도 제어 비례 게인         |
| `vel_i_gain`     | float | 0.0    | 속도 제어 적분 게인         |
| `max_velocity`   | float | 3.0    | 최대 수평 속도 (m/s)        |
| `target_timeout` | float | 3.0    | 마커 추적 타임아웃 (s)      |
| `delta_position` | float | 0.25   | 위치 도달 판정 임계값 (m)   |
| `delta_velocity` | float | 0.25   | 속도 안정 판정 임계값 (m/s) |

### 파라미터 파일 위치

```
src/precision_land/cfg/params.yaml
src/aruco_tracker/cfg/params.yaml
```

### 파라미터 튜닝 가이드

#### 1. 하강 속도 (`descent_vel`)

- **높을 때**: 빠른 착륙, 추적 오차 증가
- **낮을 때**: 느린 착륙, 안정적 추적
- **권장**: 0.5 ~ 1.5 m/s

#### 2. 비례 게인 (`vel_p_gain`)

- **높을 때**: 빠른 응답, 오버슈트 가능
- **낮을 때**: 느린 응답, 추적 지연
- **권장**: 1.0 ~ 2.0

#### 3. 적분 게인 (`vel_i_gain`)

- **높을 때**: 정상 상태 오차 감소, 불안정 가능
- **낮을 때**: 안정적이지만 오차 잔류
- **권장**: 0.0 ~ 0.5 (보통 0으로 시작)

#### 4. 타임아웃 (`target_timeout`)

- **높을 때**: 일시적 추적 손실 허용
- **낮을 때**: 빠른 실패 감지
- **권장**: 2.0 ~ 5.0초

---

## 메시지 흐름도

시각화 및 보조 출력을 제외하고 드론의 제어에만 필요한 토픽만 추려서 흐름을 나타낸 것입니다.

```
┌──────────────┐
│   Gazebo     │
└──────┬───────┘
       │ Image + CameraInfo
       ▼
┌──────────────────┐
│  aruco_tracker   │
│                  │
│  1. Detect tag   │
│  2. Estimate     │
│     pose (PnP)   │
└──────┬───────────┘
       │ /target_pose
       │ (camera frame)
       ▼
┌──────────────────────────────────┐
│       precision_land             │
│                                  │
│  1. Convert to world frame       │
│     ┌────────────────────┐       │
│     │  getTagWorld()     │       │
│     └────────────────────┘       │
│                                  │
│  2. State machine logic          │
│     ┌────────────────────┐       │
│     │  updateSetpoint()  │       │
│     └────────────────────┘       │
│                                  │
│  3. Generate commands            │
│     ┌────────────────────┐       │
│     │  Position/Velocity │       │
│     └────────────────────┘       │
└──────┬───────────────────────────┘
       │ /trajectory_setpoint
       ▼
┌──────────────────┐
│   PX4 Autopilot  │
│                  │
│  1. Execute      │
│     setpoint     │
│  2. Control      │
│     motors       │
└──────────────────┘
```

---

## 실행 방법

### 1. 시뮬레이션 환경에서 실행

```bash
# Launch file 실행 (시각화 포함)
ros2 launch precision_land precision_landing_system.launch.py

# 시각화 비활성화
ros2 launch precision_land precision_landing_system.launch.py enable_gazebo_viz:=false
```

### 2. 노드 개별 실행

```bash
# ArUco Tracker
ros2 run aruco_tracker aruco_tracker --ros-args --params-file src/aruco_tracker/cfg/params.yaml

# Precision Land
ros2 run precision_land precision_land --ros-args --params-file src/precision_land/cfg/params.yaml
```

### 3. 모드 활성화 (PX4)

precision_land 노드가 실행되면 PX4에 "PrecisionLandCustom"이라는 커스텀 모드로 등록됩니다.
QGroundControl 또는 MAVSDK를 통해 해당 모드로 전환하면 정밀 착륙이 시작됩니다.

---

## 디버깅 및 모니터링

### 1. 로그 확인

```bash
# Precision land 로그
ros2 topic echo /rosout | grep precision_land

# 상태 전환 로그 예시:
# "Switching to Search"
# "Target acquired"
# "Switching to Approach"
# "Switching to Descend"
```

### 2. 토픽 모니터링

```bash
# 마커 위치 (카메라 좌표계)
ros2 topic echo /target_pose

# 마커 위치 (월드 좌표계)
ros2 topic echo /target_pose_world

# PX4로 전송되는 setpoint
ros2 topic echo /fmu/in/trajectory_setpoint
```

### 3. RViz 시각화

```bash
ros2 run rviz2 rviz2
```

추가할 Display:

- `/target_pose_world` (PoseStamped)
- TF frames (map, camera_optical_frame)

---

## 트러블슈팅

### 문제 1: "Target lost" 메시지가 자주 발생

**원인**:

- ArUco 마커 인식 실패
- 조명 조건 불량
- 카메라 흔들림

**해결책**:

- `target_timeout` 파라미터 증가 (3.0 → 5.0)
- ArUco 마커 크기 증가
- 하강 속도 감소 (`descent_vel`: 1.0 → 0.5)

### 문제 2: 착륙 지점이 부정확함

**원인**:

- PI 게인 튜닝 부족
- 카메라 보정 오차

**해결책**:

- `vel_p_gain` 증가 (1.5 → 2.0)
- `vel_i_gain` 추가 (0.0 → 0.1)
- 카메라 캘리브레이션 재수행

### 문제 3: 드론이 진동하거나 불안정함

**원인**:

- 게인이 너무 높음
- 노이즈가 많은 센서 데이터

**해결책**:

- `vel_p_gain` 감소 (1.5 → 1.0)
- `vel_i_gain`을 0으로 설정
- 저역 통과 필터 추가 고려

---

## 참고 자료

### 코드 위치

- **Launch File**: `src/precision_land/launch/precision_landing_system.launch.py`
- **Main Code**: `src/precision_land/PrecisionLand.cpp`, `PrecisionLand.hpp`
- **ArUco Tracker**: `src/aruco_tracker/`

### 외부 문서

- [PX4 Custom Flight Modes](https://docs.px4.io/main/en/concept/flight_modes.html)
- [ArUco Marker Detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [ROS 2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
