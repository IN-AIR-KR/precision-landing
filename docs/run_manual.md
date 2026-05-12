# 실행 매뉴얼

## 목차

1. [사전 요구사항](#1-사전-요구사항)
2. [빌드](#2-빌드)
3. [V-마커 텍스처 생성 (최초 1회)](#3-v-마커-텍스처-생성-최초-1회)
4. [SITL 실행](#4-sitl-실행)
5. [실제 하드웨어 실행](#5-실제-하드웨어-실행)
6. [모니터링](#6-모니터링)
7. [착륙 시작 및 중단](#7-착륙-시작-및-중단)
8. [파라미터 튜닝](#8-파라미터-튜닝)
9. [트러블슈팅](#9-트러블슈팅)

---

## 1. 사전 요구사항

### 필수 소프트웨어

| 소프트웨어 | 버전 | 설치 방법 |
|-----------|------|----------|
| ROS2 | Humble | [공식 설치 가이드](https://docs.ros.org/en/humble/Installation.html) |
| PX4-Autopilot | v1.14 이상 | `git clone --recursive https://github.com/PX4/PX4-Autopilot.git` |
| Gazebo Harmonic | 8.x | PX4 SITL 의존성으로 자동 설치 |
| MicroXRCE-DDS Agent | 최신 | 아래 참고 |
| px4_msgs | Humble 브랜치 | 아래 참고 |
| usb_cam | 최신 | 실제 하드웨어 전용 |

### MicroXRCE-DDS Agent 설치

```bash
pip install --user -U empy pyros-genmsg setuptools
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
# 또는 snap 사용
sudo snap install micro-xrce-dds-agent --edge
```

### ROS2 워크스페이스 구성

```bash
mkdir -p ~/precision_ws/src
cd ~/precision_ws/src

# px4_msgs 클론 (ROS2 Humble 브랜치)
git clone -b release/1.14 https://github.com/PX4/px4_msgs.git

# 이 패키지 심볼릭 링크 또는 복사
ln -s /Users/sungho/code/narae/precision-landing/src/pl_msgs .
ln -s /Users/sungho/code/narae/precision-landing/src/pl_nodes .
ln -s /Users/sungho/code/narae/precision-landing/src/pl_bringup .
```

### usb_cam 설치 (실제 하드웨어 전용)

```bash
sudo apt install ros-humble-usb-cam
# 또는 소스 빌드:
cd ~/precision_ws/src
git clone https://github.com/ros-drivers/usb_cam.git -b ros2
```

---

## 2. 빌드

```bash
cd ~/precision_ws

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드 (순서 중요: pl_msgs 먼저)
colcon build --packages-select pl_msgs
colcon build --packages-select pl_nodes pl_bringup

# 환경 소스
source install/setup.bash
```

---

## 3. V-마커 텍스처 생성 (최초 1회)

SITL 시뮬레이션에서 Gazebo 모델에 사용할 텍스처 PNG를 생성한다.

```bash
cd /Users/sungho/code/narae/precision-landing
python3 scripts/generate_marker_texture.py

# 결과 확인 (미리보기)
python3 scripts/generate_marker_texture.py --preview

# 사용자 지정 경로
python3 scripts/generate_marker_texture.py --output /custom/path/v_marker.png
```

생성 결과: `simulation/models/v_marker/v_marker.png`

---

## 4. SITL 실행

총 4개의 터미널을 사용한다.

### 터미널 1: PX4 SITL + Gazebo 실행

```bash
cd /Users/sungho/code/narae/PX4-Autopilot

# V-마커 모델/월드 경로 등록
export GZ_SIM_RESOURCE_PATH=/Users/sungho/code/narae/precision-landing/simulation/models:\
/Users/sungho/code/narae/precision-landing/simulation/worlds:\
${GZ_SIM_RESOURCE_PATH}

# 드론 35m 상공 스폰, precision_landing 월드 사용
PX4_GZ_MODEL_POSE="0,0,35,0,0,0" \
PX4_GZ_WORLD=precision_landing \
make px4_sitl gz_x500_mono_cam_down
```

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
source ~/precision_ws/install/setup.bash

ros2 launch pl_bringup precision_landing_sitl.launch.py
```

### 터미널 4: 착륙 시퀀스 시작

```bash
source /opt/ros/humble/setup.bash
source ~/precision_ws/install/setup.bash

# 착륙 시작
ros2 service call /landing_controller_node/start_landing std_srvs/srv/Trigger

# 상태 모니터링
ros2 topic echo /pl/landing_state
```

---

## 5. 실제 하드웨어 실행

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
source ~/precision_ws/install/setup.bash

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

---

## 6. 모니터링

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

---

## 7. 착륙 시작 및 중단

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

---

## 8. 파라미터 튜닝

파라미터 파일 위치: `src/pl_nodes/config/`

### 검출 민감도 조정

`v_marker_detector_params.yaml`:
```yaml
hough_param2: 30   # 낮출수록 더 많이 검출 (오탐 증가 주의)
min_confidence: 0.25  # 최소 신뢰도 임계값
```

### PID 게인 조정

`landing_controller_params.yaml`:
```yaml
# XY 정렬이 진동할 경우 kp를 낮추거나 kd를 높임
v_marker_kp: 0.8   # 비례
v_marker_kd: 0.10  # 미분 (진동 억제)

# 정렬이 느릴 경우 kp를 높임
aruco_kp: 1.5
```

### 고도 전환점 조정

```yaml
alt_switch_to_aruco: 8.0    # ArUco 전환 고도 (높이면 ArUco 더 일찍 시도)
alt_final_descent: 2.0      # 맹목 하강 전환 고도
```

파라미터 변경 후 **리빌드 및 재실행** 필요:
```bash
colcon build --packages-select pl_nodes && source install/setup.bash
```

---

## 9. 트러블슈팅

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
