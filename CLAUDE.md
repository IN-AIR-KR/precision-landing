# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 빌드 및 실행

```bash
# 클론 (서브모듈 포함)
git clone --recursive https://github.com/<your-org>/precision-landing.git

# 빌드 — 워크스페이스는 레포 안의 pl_ws/
cd ~/precision-landing/pl_ws
colcon build --packages-select pl_msgs
colcon build --packages-select pl_nodes pl_bringup
source install/setup.bash

# SITL 실행 (터미널 4개 필요)
# export 대신 심볼릭 링크로 등록하는 게 더 안정적
ln -s ~/precision-landing/simulation/models/v_marker ~/PX4-Autopilot/Tools/simulation/gz/models/v_marker
ln -s ~/precision-landing/simulation/worlds/precision_landing.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/precision_landing.sdf
PX4_GZ_MODEL_POSE="0,0,35,0,0,0" PX4_GZ_WORLD=precision_landing make px4_sitl gz_x500_mono_cam_down
MicroXRCEAgent udp4 -p 8888
ros2 launch pl_bringup precision_landing_sitl.launch.py

# 실제 하드웨어
ros2 launch pl_bringup precision_landing_real.launch.py video_device:=/dev/video0

# 착륙 시작 / 중단
ros2 service call /landing_controller_node/start_landing std_srvs/srv/Trigger
ros2 service call /landing_controller_node/abort std_srvs/srv/Trigger
```

## 아키텍처

### 전체 흐름

카메라 소스(`usb_cam` 또는 `ros_gz_bridge`) → `/pl/image_raw`, `/pl/camera_info` → 검출 노드 2개 → `landing_controller_node` (FSM + PID) → PX4 (`/fmu/in/*` 토픽, uXRCE-DDS).

### 노드 역할

- **`v_marker_detector_node`** — `cv2.HoughCircles`로 V-마커 원형 테두리 검출. 실패 시 이진화+윤곽선 폴백. 픽셀 오프셋과 거리 추정값을 `MarkerDetection`으로 발행.
- **`aruco_detector_node`** — `DICT_4X4_50` ID 0 마커(0.5m) 검출. `solvePnP`로 tvec 추출 후 `MarkerDetection.pose`에 카메라 프레임 기준 3D 포즈 포함.
- **`landing_controller_node`** — 10 Hz 제어 루프. FSM 8개 상태 관리. XY 축 각각 독립 PID (coarse/fine 2단계). PX4 OFFBOARD 모드로 `TrajectorySetpoint` velocity 필드로 제어. `~/start_landing`, `~/abort` 서비스 제공.

### FSM 상태 전이 핵심 규칙

```
IDLE → SEARCH_V_MARKER → ALIGN_V_MARKER → DESCEND_V_MARKER
     → ALIGN_ARUCO → DESCEND_ARUCO → FINAL_DESCENT → LANDED
```

- 고도 < 8m AND ArUco 검출 시 `DESCEND_V_MARKER` → `ALIGN_ARUCO` 전환
- 고도 < 2m 시 ArUco 유실 여부와 무관하게 `FINAL_DESCENT`로 전환 (정상 경로)
- 검출 유실 시 상위 검출기 상태로 복귀. 타임아웃 시 `ABORT` → `IDLE`
- 모든 상태 전환 시 PID 적분기 리셋

### 좌표 변환

광학 → NED 변환: `R_OPT_NED = [[0,-1,0],[1,0,0],[0,0,1]]`  
고도 AGL = `-vehicle_local_position.z` (NED z 음수 = 위쪽)

### 커스텀 메시지

- `pl_msgs/MarkerDetection` — 검출 결과 (픽셀 좌표, 신뢰도, 추정 거리, ArUco 3D 포즈)
- `pl_msgs/LandingState` — FSM 상태, 고도, XY 오차, 폴백 원인

### SITL 모델

PX4 기본 모델 `x500_mono_cam_down` 사용 (하방 1280×960, FOV 1.74 rad, 30 Hz).  
Gazebo 카메라 토픽: `/world/precision_landing/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image`  
센서 이름이 `imager`일 경우 `precision_landing_sitl.launch.py`의 `GZ_SENSOR` 변수 수정 필요.

### 파라미터 파일

`pl_ws/src/pl_nodes/config/` 아래 yaml 3개. 변경 후 `colcon build --packages-select pl_nodes` 재실행 필요.

## 문서

- `docs/architecture.md` — 노드 그래프, FSM 다이어그램, 수식 (PID, PnP, 좌표 변환)
- `docs/run_manual.md` — 상세 실행 절차, 트러블슈팅
