# 실행 매뉴얼

## 목차

1. [사전 요구사항](#1-사전-요구사항)
2. [빌드](#2-빌드)
3. [SITL 실행](#3-sitl-실행)
4. [실제 하드웨어 실행](#4-실제-하드웨어-실행)
5. [모니터링](#5-모니터링)
6. [착륙 시작 및 중단](#6-착륙-시작-및-중단)
7. [파라미터 튜닝](#7-파라미터-튜닝)
8. [트러블슈팅](#8-트러블슈팅)

## 1. 사전 요구사항

### 필수 소프트웨어

| 소프트웨어          | 버전            |
| ------------------- | --------------- |
| ROS2                | Humble          |
| PX4-Autopilot       | 최신            |
| Gazebo Harmonic     | 8.x             |
| MicroXRCE-DDS Agent | 최신            |
| px4_msgs            | main (서브모듈) |
| usb_cam             | 최신            |

### 레포 클론

```bash
# 서브모듈(px4_msgs)도 함께 받는다
git clone --recursive https://github.com/IN-AIR-KR/precision-landing.git

# 이미 클론한 경우 서브모듈 초기화
git submodule update --init
```

### usb_cam 설치 (실제 하드웨어 전용)

```bash
sudo apt install ros-humble-usb-cam
```

## 2. 빌드

```bash
cd ~/precision-landing/pl_ws

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드 (순서 중요: pl_msgs 먼저)
colcon build --packages-select pl_msgs
colcon build --packages-select pl_nodes pl_bringup

# 환경 소스
source install/setup.bash
```

## 3. SITL 실행

총 4개의 터미널을 사용한다.

### 터미널 1: PX4 SITL + Gazebo 실행

```bash
cd ~/PX4-Autopilot

# V-마커 모델/월드 경로 등록
export GZ_SIM_RESOURCE_PATH=~/precision-landing/simulation/models:\
~/precision-landing/simulation/worlds:\
${GZ_SIM_RESOURCE_PATH}

# 드론 지상 근처 스폰, precision_landing 월드 사용
PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" \
PX4_GZ_WORLD=precision_landing \
make px4_sitl gz_x500_mono_cam_down
```

PX4 콘솔이 열리면 아래 명령으로 35m까지 이륙한다.

```
pxh> commander takeoff 35
```

> **대안 — export가 적용되지 않을 경우**: PX4 모델/월드 디렉터리에 심볼릭 링크를 생성하면 환경 변수 없이도 인식된다.
>
> ```bash
> ln -s ~/precision-landing/simulation/models/v_marker \
>   ~/PX4-Autopilot/Tools/simulation/gz/models/v_marker
> ln -s ~/precision-landing/simulation/worlds/precision_landing.sdf \
>   ~/PX4-Autopilot/Tools/simulation/gz/worlds/precision_landing.sdf
> ```

> **주의**: Gazebo가 실행되면 `gz topic -l` 로 카메라 토픽 이름을 확인한다.  
> 예상 경로: `/world/precision_landing/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image`  
> 센서 이름이 `imager`일 경우 `src/pl_bringup/launch/precision_landing_sitl.launch.py` 의 `GZ_SENSOR` 변수를 수정한다.

### 터미널 2: MicroXRCE-DDS Agent 실행

```bash
MicroXRCEAgent udp4 -p 8888
```

PX4와 ROS2 간 통신(uXRCE-DDS) 브리지 역할. PX4 콘솔에 `uxrce_dds_client start -t udp -p 8888` 메시지가 보이면 정상.

### 터미널 3: ROS2 시스템 실행

```bash
source /opt/ros/humble/setup.bash
source ~/precision-landing/pl_ws/install/setup.bash

ros2 launch pl_bringup precision_landing_sitl.launch.py
```

### 터미널 4: 이륙 및 착륙 시퀀스 시작

```bash
source /opt/ros/humble/setup.bash
source ~/precision-landing/pl_ws/install/setup.bash

# 터미널 1 PX4 콘솔에서 이륙 후 35m 도달 확인
# pxh> commander takeoff 35

# 착륙 시퀀스 시작
ros2 service call /landing_controller_node/start_landing std_srvs/srv/Trigger

# 상태 모니터링
ros2 topic echo /pl/landing_state
```

## 4. 실제 하드웨어 실행

### 사전 확인

1. 하방 USB 카메라 연결 확인: `ls /dev/video*`
2. PX4 비행 컨트롤러와 Companion Computer 연결
3. MicroXRCE-DDS Agent 실행 (Companion Computer에서)

### 터미널 1: MicroXRCE-DDS Agent

```bash
# 시리얼 연결
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600

# 또는 UDP (이더넷 연결)
MicroXRCEAgent udp4 -p 8888
```

### 터미널 2: ROS2 시스템 실행

```bash
source /opt/ros/humble/setup.bash
source ~/precision-landing/pl_ws/install/setup.bash

# 기본 실행 (카메라 /dev/video0)
ros2 launch pl_bringup precision_landing_real.launch.py

# 카메라 장치 지정
ros2 launch pl_bringup precision_landing_real.launch.py video_device:=/dev/video1
```

### 터미널 3: 착륙 시퀀스 시작

```bash
# 드론이 35m 상공에 도달한 후 실행
ros2 service call /landing_controller_node/start_landing std_srvs/srv/Trigger
```

## 5. 모니터링

### FSM 상태 확인

```bash
ros2 topic echo /pl/landing_state
```

출력 예시:

```
state: DESCEND_V_MARKER
altitude_agl: 15.3
v_marker_visible: true
aruco_visible: false
error_x: 0.23
error_y: -0.11
time_in_state: 8.4
```

### 디버그 영상 확인 (rqt)

```bash
# V-마커 검출 결과
ros2 run rqt_image_view rqt_image_view /pl/v_marker_debug_image

# ArUco 검출 결과
ros2 run rqt_image_view rqt_image_view /pl/aruco_debug_image
```

### 전체 노드 그래프

```bash
ros2 run rqt_graph rqt_graph
```

### 검출 결과 확인

```bash
ros2 topic echo /pl/v_marker_detection
ros2 topic echo /pl/aruco_detection
```

## 6. 착륙 시작 및 중단

### 착륙 시작

```bash
ros2 service call /landing_controller_node/start_landing std_srvs/srv/Trigger
```

응답:

- `success: true` → 정상 시작
- `success: false` → 이미 다른 상태 진행 중

### 비상 중단 (ABORT)

```bash
ros2 service call /landing_controller_node/abort std_srvs/srv/Trigger
```

ABORT 상태 전환 후 드론은 LOITER 모드로 전환.

## 7. 파라미터 튜닝

파라미터 파일 위치: `src/pl_nodes/config/`

### 검출 민감도 조정

`v_marker_detector_params.yaml`:

```yaml
hough_param2: 30 # 낮출수록 더 많이 검출 (오탐 증가 주의)
min_confidence: 0.25 # 최소 신뢰도 임계값
```

### PID 게인 조정

`landing_controller_params.yaml`:

```yaml
# XY 정렬이 진동할 경우 kp를 낮추거나 kd를 높임
v_marker_kp: 0.8 # 비례
v_marker_kd: 0.10 # 미분 (진동 억제)

# 정렬이 느릴 경우 kp를 높임
aruco_kp: 1.5
```

### 고도 전환점 조정

```yaml
alt_switch_to_aruco: 8.0 # ArUco 전환 고도 (높이면 ArUco 더 일찍 시도)
alt_final_descent: 2.0 # 맹목 하강 전환 고도
```

파라미터 변경 후 **리빌드 및 재실행** 필요:

```bash
cd ~/precision-landing/pl_ws
colcon build --packages-select pl_nodes && source install/setup.bash
```

## 8. 트러블슈팅

### 문제: V-마커가 인식되지 않음

1. `/pl/v_marker_debug_image` 확인 → 원이 그려지는지 체크
2. `hough_param2` 값 낮추기 (기본 30 → 20)
3. 조명 조건 확인 (밝은 환경에서 테스트)
4. `real_circle_diameter` 파라미터 실제 마커 크기와 일치 여부 확인

### 문제: ArUco 인식이 불안정함

1. `/pl/aruco_debug_image` 에서 마커 코너 검출 확인
2. 카메라 초점/흔들림 확인
3. `alt_switch_to_aruco` 를 낮춰 더 가까운 거리에서 전환 시도

### 문제: Gazebo 카메라 영상이 오지 않음

1. `gz topic -l | grep camera` 로 실제 토픽 이름 확인
2. `precision_landing_sitl.launch.py` 의 `GZ_SENSOR` 변수 수정 (`camera` ↔ `imager`)
3. `ros2 topic list | grep pl` 로 브리지 동작 확인

### 문제: PX4와 ROS2 통신 안됨

1. MicroXRCE-DDS Agent 실행 확인
2. `ros2 topic list | grep fmu` → 토픽이 없으면 연결 실패
3. PX4 콘솔에서 `uxrce_dds_client status` 확인
4. 방화벽/UDP 포트 8888 열림 확인

### 문제: OFFBOARD 모드 전환 실패

1. PX4 GPS fix 확인 (SITL은 가상 GPS이므로 자동 fix)
2. 안전 스위치(Safety Switch) 비활성화 확인
3. `OffboardControlMode` 토픽이 10Hz 이상으로 발행되는지 확인:
   ```bash
   ros2 topic hz /fmu/in/offboard_control_mode
   ```
