# Precision Landing Flight Analyzer

ARMS의 `SW/scripts/flight_analyzer`를 기반으로 한 ROS 2 착륙 로그 뷰어다.

## 녹화

ROS Humble 환경에서 rosbag CLI와 SQLite 플러그인을 설치하고 프로젝트를 빌드한다.

```bash
sudo apt install ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins
./scripts/build.sh real
./scripts/run_real.sh
```

실기체, SITL(`./scripts/run.sh`), HITL(Jetson의 `hitl_jetson.launch.py`)에서 녹화기는 기본으로 실행된다. PX4가 ARM 상태일 때 녹화를 시작하고 DISARM 때 bag을 마감한다. 녹화를 끄려면 `record:=false`를 지정한다. 다시 ARM하면 새 bag을 만든다. 이미 ARM 상태에서 녹화기를 시작하면 첫 PX4 상태 수신부터 기록한다. 녹화 중 런치를 종료해도 열린 bag을 마감한다.

녹화 파일은 저장소 루트의 `flight_logs/<날짜_시각>_<프로필>_<세션>/bag/`에 저장된다. 다른 경로에 저장하려면 런치에 `log_dir:=/path/to/flight_logs`를 전달한다. 스택을 이미 실행했다면 같은 ROS 환경과 Domain ID에서 `ros2 run pl_logging flight_recorder`로 ARM 상태를 감시하는 녹화기를 별도로 띄울 수 있다.

기록 항목은 착륙 FSM, ArUco 검출 상태, 상대 및 월드 타겟 포즈, 모니터 상태, PX4 기체 상태·자세·위치·GPS·배터리·착지 상태와 노출된 제어 명령 및 응답이다. 유일한 영상 토픽은 viz의 `/pl/monitor/image/compressed`다.

## 다시 보기

Jetson의 기록 디렉터리를 통째로 노트북의 `flight_logs/`에 복사할 수 있다. 뷰어 실행에는 ROS가 필요 없다.

```bash
python3 -m venv /tmp/pl-viewer
source /tmp/pl-viewer/bin/activate
python -m pip install -r scripts/flight_analyzer/requirements.txt
python scripts/flight_analyzer/app.py
```

브라우저에서 <http://127.0.0.1:5050/>을 열면 최신 기록을 불러온다. 영상 재생, 시간 탐색, 그래프 커서 따라가기와 토픽 필드 패널 추가를 지원한다. 그래프 라이브러리 uPlot은 브라우저에서 CDN으로 불러온다.

별도 로그 폴더나 단일 bag 디렉터리는 `--log-dir`로 열 수 있다. PX4 메시지 필드를 볼 때는 기록과 같은 `px4_msgs` 서브모듈을 초기화하거나 `--px4-msg-dir /path/to/px4_msgs/msg`를 지정한다. HUD와 모니터 상태 그래프는 표준 ROS 메시지를 사용한다.

`Video + Graphs`의 영상 아래 3D 뷰에서 드론 궤적, 드론이 추정한 마커, 시뮬레이터의 실제 마커를 같은 시간축으로 볼 수 있다.
마우스 드래그로 회전하고 휠로 확대한다. **보기 중심**의 **원점**은 ARM 위치를 (0, 0, 0)으로 표시하고,
**드론 따라가기**는 드론 위치를 따라간다. 영상이 없는 기록도 3D 재생과 탐색이 가능하다.
ARM 전환 이후부터 녹화된 로그에서는 첫 유효 드론 위치를 원점으로 쓴다.

SITL 및 HITL에서 Gazebo가 제공하는 마커와 드론의 실제 위치를 bag에 기록한다. 뷰어는 첫 공통
드론 위치 샘플로 Gazebo ENU 원점을 PX4 로컬 원점에 맞춘다. 실제 마커 위치가 기록된 bag에만
실제 마커가 표시된다.
