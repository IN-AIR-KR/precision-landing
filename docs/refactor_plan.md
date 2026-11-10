# 정밀착륙 개선 순서

기준 브랜치는 `feature/moving-aruco` (`5df7b91`)이다. 현재 이동 타겟의 탐색·접근·하강 동작과 QGC의 `PrecisionLandCustom` 모드를 기준으로 각 변경을 검증한다. **영상 입력 분리 → Python 인식기 전환 → PX4 공식 릴리스 검증 → 공통 모니터링 → Delayed KF 적용** 순서로 진행한다.

## 0. 기준 동작 기록 및 워크스페이스 구조 유지

진행 결과와 재현 명령: [0번 기준 기록](baseline_stage0.md). 패키지는 아래 이름으로 분리한다. 인식기는 2번 전까지 기존 C++ 구현을, 제어기는 5번 전까지 기존 KF를 유지한다. 기준 비교 자료는 baselines에 보존한다.

- 현재 PX4·`px4_msgs`·`px4_ros2_cpp` 조합을 기록한다. 정적·이동 타겟 SITL에서 카메라 영상과 보정 정보, `/target_pose`, 기체 자세·위치, 착륙 로그를 확보한다. 같은 입력으로 인식과 추정 변경을 비교할 수 있게 ROS bag으로 저장한다.
- 저장소 루트를 그대로 `colcon` 워크스페이스로 쓴다. 프로젝트 패키지는 `src/`, Gazebo 월드와 모델은 `simulation/`에 둔다. 실기체와 시뮬의 설치·빌드·실행 대상을 각각 정의한다.

### 패키지 구성

| 패키지 | 역할 |
|---|---|
| `pl_camera` | `camera_sim.launch.py`의 Gazebo 브리지, 실기체 CSI 캡처, 촬영 시각 기록, `Image`·`CameraInfo` 발행 |
| `pl_detection` | Python ArUco 검출 및 카메라 기준 `/target_pose` 발행 |
| `pl_control` | Delayed KF, 착륙 상태 머신, PX4 상태 수신·모드 등록·명령 전송 |
| `pl_bringup` | `sim.launch.py`와 `hardware.launch.py`, 환경별 설정과 토픽 연결 |
| `pl_viz` | 실기체와 시뮬 공통 모니터링 화면 및 모니터링용 출력 |

PX4 연동 코드는 `pl_control` 내부의 어댑터로 둔다. Micro XRCE-DDS Agent는 별도 프로세스로 실행하고, 연결마다 실행 주체를 하나로 둔다. [PX4 uXRCE-DDS 문서](https://docs.px4.io/v1.17/en/middleware/uxrce_dds)

- **시뮬** — `pl_bringup`의 `sim.launch.py`가 `ExecuteProcess`로 `MicroXRCEAgent udp4 -p 8888`을 실행한다. `pl_camera/camera_sim.launch.py`를 include하여 `ros_gz_bridge`로 Gazebo 카메라 데이터를 전달한다.
- **실기체** — `hardware.launch.py`가 `ExecuteProcess`로 UART Agent를 띄우고, CSI 캡처·인식·제어 노드도 함께 실행한다. UART 장치와 보드레이트는 실제 배선과 PX4의 `uxrce_dds_client` 설정에 맞춘다. 제어기는 PX4 연결과 유효한 기체 상태를 확인한 뒤 착륙 제어를 시작한다.
- **기존 서비스 정리** — 실기체 launch를 검증한 뒤 Jetson에 설치된 기존 USB 카메라·인식기·Agent 서비스를 비활성화하고 `services/`의 개별 서비스와 스크립트를 정리한다. 기존 Agent는 UART 중복 점유를 피하도록 새 launch를 시험하기 전에 중지한다. 부팅 자동 실행이 필요할 때는 `hardware.launch.py` 전체를 실행하는 `systemd` 서비스 하나를 둔다.

### 설정 파일 위치

`pl_bringup/config/`에 `hardware.yaml`과 `sim.yaml` 두 파일을 둔다. 각 launch는 해당 파일을 읽고, 파일 안의 섹션을 노드 파라미터와 프로세스 실행 인자로 전달한다.

| 섹션 | 관리할 값 |
|---|---|
| `camera` | 실기체 CSI 센서 ID·모드, 해상도·FPS, optical frame, 내부 보정값·왜곡 계수 |
| `detection` | ArUco 딕셔너리·ID, 마커 크기, 디버그 영상 설정 |
| `control` | 기체 기준 카메라 장착 위치·방향, 제어 게인, 유실 타임아웃, Delayed KF 설정 |
| `agent` | 실기체 UART 장치·보드레이트 또는 SITL UDP 포트 |
| `viz` | 상태 표시 갱신 주기, 데이터 유효 시간, 모니터링 영상의 전송 방식·최대 FPS·JPEG 품질 |
| `bridge` | 시뮬 전용 Gazebo 영상·`CameraInfo` 토픽 연결 |

`hardware.yaml`의 `camera` 섹션에 실제 출력 해상도에 맞는 보정값을 함께 둔다. `sim.yaml`은 `bridge` 섹션을 통해 Gazebo가 제공하는 `CameraInfo`를 전달한다.

## 1. 실기체와 시뮬 영상 입력 분리

상태: **공통 입력·환경별 장착 설정 분리 완료, CSI 구현·실기체 검증은 사용자 요청으로 보류**. [적용 범위와 시험 방법](camera_inputs_stage1.md). 아래 완료 기준 중 Jetson/CSI 항목은 아직 충족하지 않았다.

- 실기체는 Jetson에 연결한 MIPI CSI 카메라를 사용한다. IMX219는 후보로 두고, 실제 카메라 모듈·Jetson 보드·JetPack 조합의 드라이버 지원과 캡처 동작을 확인한다. Jetson ISP를 사용하는 경우 `libargus` 또는 GStreamer `nvarguscamerasrc`를 검토하고, 촬영 시각을 가져와 ROS 시간 기준으로 변환할 수 있는 경로를 선택한다. [NVIDIA 카메라 개발 문서](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/CameraDevelopment/CameraSoftwareDevelopmentSolution.html)
- 인식기의 Gazebo 월드·모델 경로를 제거한다. 실기체 CSI 캡처 노드와 시뮬 `ros_gz_bridge`를 launch에서 같은 `Image`·`CameraInfo` 인터페이스에 연결한다. CSI 캡처 노드는 이미지 발행 시각 대신 촬영 시각을 `Image.header.stamp`에 넣고, 인식기는 이를 카메라 optical frame 기준 `/target_pose`에 그대로 전달한다.
- 실기체에서 사용할 센서 모드와 출력 해상도에 맞춰 `CameraInfo`를 준비한다. 기체 기준 카메라 장착 위치·방향과 마커 크기도 환경별 설정으로 분리한다. Gazebo 브리지와 기존 월드 마커 표시는 시뮬 전용으로 분리하고, `pl_viz`는 두 환경에서 공통으로 사용한다.
- **완료 기준** — 같은 인식기 빌드로 launch 설정만 바꿔 두 카메라 입력을 받을 수 있다. Jetson에서 CSI 프레임의 촬영 시각 출처와 ROS 시간 변환을 확인하고, 실제 출력 해상도에서 마커의 방향·거리를 검증한다. 실기체 프로필은 CSI 입력으로 실행되고 Gazebo는 시뮬 프로필에서 실행된다.

## 2. Python 인식기로 전환하고 설치 의존성 정리

- ArUco 딕셔너리·ID, 마커 크기, PnP 포즈 계산, `/target_pose`의 타입·좌표계·타임스탬프·QoS를 유지한 채 기존 `aruco_tracker`를 Python 패키지 `pl_detection`으로 옮긴다. `cv2`는 Jetson에서 동작을 확인한 headless 바이너리 패키지로 설치한다.
- Python 인식기에서 이미지 변환을 직접 처리할 경우 지원 인코딩과 행 간격(`Image.step`)을 명시한다. 이 경로를 검증한 뒤 `cv_bridge`와 `vision_opencv` 소스 서브모듈을 제거한다. OpenCV 소스 설치 스크립트와 `/usr/local` 링크도 정리한다. Python 인식은 `cv2` 런타임을 사용한다.
- 인식기의 `px4_msgs`·Eigen, 제어기의 `tf2`·`tf2_ros`처럼 코드가 사용하지 않는 직접 의존성을 제거한다. 제어기의 Eigen은 좌표 변환과 칼만 필터에 사용한다.
- **완료 기준** — 같은 ROS bag에서 기존 인식기와 마커 검출 결과, 위치·자세를 비교한다. Jetson에서 처리율과 촬영부터 포즈 발행까지의 지연 분포를 측정하고, OpenCV 소스 빌드 없이 새 환경에서 실행한다.

## 3. PX4 커밋 의존성을 검증된 릴리스 조합으로 전환

- `aruco_moving.sdf`를 이 저장소의 `simulation/gz/worlds/`에 두고, Gazebo를 별도로 띄워 PX4 SITL을 연결하는 방식을 검증한다. 프로젝트의 월드 파일을 PX4 소스 트리에 복사하는 절차를 제거한다. [PX4 Gazebo Standalone](https://docs.px4.io/v1.16/en/sim_gazebo_gz/#standalone-mode)
- 실기체에 사용할 공식 PX4 릴리스 하나를 정하고, 그 릴리스에 맞는 `px4_msgs`와 `px4_ros2_cpp` 버전을 검증한다. 커스텀 모드 등록, 착륙 감지 토픽, 접근 단계의 XY 속도와 Z 위치 혼합 setpoint, 하강 중 속도와 yaw 명령을 확인한다. 검증된 조합을 문서와 의존성 설정에 기록한 뒤 README의 기존 PX4 SHA 요구사항을 교체한다. [PX4 ROS 2 호환성 문서](https://docs.px4.io/main/en/ros2/user_guide#compatibility-issues)
- **완료 기준** — 공식 릴리스로 새 환경에서 정적·이동 타겟 SITL을 재현한다. QGC와 ROS 양쪽에서 모드 시작, 접근·하강·착륙 완료, 타겟 유실 및 제어권 회수를 확인한 뒤 실기체에서 검증한다.

### QGC와 ROS에서 정밀착륙 시작

- QGC에서는 기존 `PrecisionLandCustom` 모드를 선택한다. ROS에서는 `pl_control`의 `/pl/start_landing` 서비스가 PX4에 같은 모드로 전환을 요청한다. PX4 등록 과정에서 받은 모드 ID를 사용하고, 활성화 후에는 두 경로가 같은 착륙 FSM을 실행한다. [PX4 외부 모드](https://docs.px4.io/v1.16/en/ros2/px4_ros2_control_interface) · [PX4 모드 전환 명령](https://docs.px4.io/v1.16/en/msg_docs/VehicleCommand)
- ROS 요청은 모드 등록, 유효한 기체 상태, 무장 및 비행 중 여부를 확인한 뒤 보낸다. 서비스는 착륙 모드 전환만 요청하며 무장과 이륙은 별도 조작으로 둔다. PX4가 모드를 실제로 활성화하면 성공을 반환한다. 요청 거절과 전환 확인 타임아웃을 구분하고, 이미 활성 상태에서 요청이 반복되면 FSM을 다시 시작하지 않는다. QGC나 RC의 모드 변경으로 제어권을 회수할 수 있어야 한다.

구현 후 실기체와 시뮬에서 같은 명령을 사용한다.

```bash
ros2 service call /pl/start_landing std_srvs/srv/Trigger '{}'
```

## 4. 실기체와 시뮬 공통 모니터링

- `hardware.launch.py`와 `sim.launch.py`에서 `pl_viz`의 모니터링 출력을 기본으로 제공한다. 운영자가 화면을 열면 아래 항목을 같은 화면에서 확인할 수 있게 한다.

| 표시 항목 | 내용 |
|---|---|
| ArUco 인식 | 카메라 영상, 검출·미검출 여부, 마커 ID와 윤곽, 검출된 마커의 위치·거리 |
| 착륙 FSM | 현재 단계(`Idle`, `Search`, `Approach`, `Descend`, `Finished`)와 타겟 유실 여부 |
| 기체 상태 | GPS fix 상태, 사용 중인 위성 수, 기체 수평·수직 속도(m/s) |
| 배터리 | 팩 전압(V), PX4가 추정한 잔량(%) |

- GPS·기체 속도·배터리 값은 양쪽 환경에서 PX4 텔레메트리를 사용한다. 선택한 PX4 릴리스에서 필요한 토픽을 확인한다. 수신되지 않거나 유효 시간이 지난 값은 화면에서 `수신 없음` 또는 `갱신 지연`으로 표시한다. 영상이 끊겨도 상태는 계속 표시한다. [PX4 DDS 토픽 목록](https://docs.px4.io/v1.17/en/middleware/dds_topics)
- 실기체 모니터는 JPEG 압축 영상(`/image_proc/compressed`, `sensor_msgs/msg/CompressedImage`)을 기본으로 구독한다. 시뮬 모니터는 비압축 `/image_proc`를 기본으로 구독한다. 전송 방식과 영상 발행 제한은 각 환경의 `viz` 섹션에서 선택한다.
- 모니터링 영상과 상태 출력은 **출력별로 구독자가 있을 때만** 처리·발행한다. 압축 구독만 있으면 JPEG 출력만 발행하고, 압축 구독이 없으면 JPEG 인코딩을 건너뛴다. 두 영상 출력에 구독이 없으면 마커 표시도 그리지 않는다. 내부 구독도 필요할 때만 연결해 모니터링 노드가 상류의 디버그 발행을 계속 유발하지 않도록 한다. 착륙에 필요한 원본 영상·`/target_pose`·PX4 상태 수신은 모니터링 화면과 독립적으로 동작한다.
- **완료 기준** — 실기체와 SITL에서 같은 화면으로 모든 항목을 확인한다. 비압축 영상·압축 영상·상태 구독을 각각 열고 닫아 해당 출력만 시작·중지되는지 확인한다. 화면을 닫아도 인식과 착륙 제어의 입력은 계속 전달된다.

## 5. Delayed KF로 촬영 시각 기준 추정

- 이미지 촬영 시각과 PX4 자세·위치 샘플 시각을 같은 시간 기준으로 맞춘다. 기체 위치·자세 이력을 보관하고 촬영 시각으로 보간한 뒤, 해당 시각의 기체 자세와 카메라 장착 변환으로 관측을 NED에 옮긴다. 이력이 없는 관측은 폐기한다.
- 기존 위치·속도 KF에 제한된 시간 구간의 상태·공분산과 관측 이력을 둔다. 늦게 도착한 관측은 촬영 시각 상태에 반영하고, 그 뒤에 이미 반영했던 관측까지 시각 순서대로 재처리해 현재 상태를 복원한다. 제어 루프는 현재 시각으로 예측한 위치·속도를 사용한다. [관측 순서 역전 시 재처리 방식 참고](https://arxiv.org/abs/2505.20043)
- 허용 지연 구간은 측정한 지연 분포와 제어 루프 처리 시간을 기준으로 정한다. 너무 오래되거나 중복된 관측은 걸러내고, 타겟 유실 판단은 수신 시각 대신 **유효하게 반영된 최신 관측의 촬영 시각**을 기준으로 한다. 모드 재진입과 시뮬 시계 초기화 때는 이전 추정 상태와 이력을 초기화한다.
- **완료 기준** — 같은 기록에 지연·순서 뒤바뀜·프레임 유실을 주입해 기존 KF와 타겟 위치·속도 오차를 비교한다. SITL에서 오래된 관측이 유실 판정을 늦추지 않고, 모드 재진입 시 이전 타겟을 따라가지 않는지 확인한다.
