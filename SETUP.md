# Jetson Orin Nano 8GB (Super) 개발 환경 세팅

이 문서는 **Jetson Orin Nano 8GB Developer Kit (Super)** 에서 이 저장소를 빌드·실행하기 위해
실제로 설치한 패키지와 버전, 설치 방법을 기록한 것입니다.

- 작성일: 2026-11-12
- 대상 브랜치: `feat/precision-landing-refactor-plan`
- 저장소 위치: `~/INAIR/precision_landing`
- 젯슨이 담당하는 프로필: `hitl_jetson` (PC와 분산 SITL) / `hardware` (실기체 FC) / `sim` (젯슨 단독 SITL)

## 0. 베이스 환경 (SDK Manager로 이미 설치된 상태)

| 항목 | 값 |
| --- | --- |
| 아키텍처 | `aarch64` / dpkg `arm64` |
| Jetson Linux | R36.5.0 (`nvidia-l4t-core 36.5.0-20260115194252`) — JetPack 6.2.2 |
| OS | Ubuntu 22.04.5 LTS (jammy) |
| 커널 | 5.15.185-tegra |
| Python | 3.10.12 |
| 전원 모드 | 25W (`nvpmodel -q`) |

`nvidia-l4t-*` 패키지는 **제거·버전 변경하지 않았습니다.** `do-release-upgrade`,
`apt upgrade` 전면 실행, `apt dist-upgrade`도 수행하지 않았습니다.

### CUDA / cuDNN / TensorRT 는 설치하지 않음

플래시 직후 `nvidia-jetpack` 메타패키지가 설치되지 않은 상태였고(`/usr/local/cuda` 없음),
**이 저장소는 CUDA·cuDNN·TensorRT·PyTorch·ONNX Runtime 중 어느 것도 사용하지 않기 때문에**
의도적으로 설치하지 않았습니다. 근거:

- `src/`, `scripts/` 전체 Python import: `cv2`, `numpy`, `rclpy`, `yaml`, `pymavlink` 뿐
- C++ 의존성: `rclcpp`, `Eigen3`, `px4_ros2_cpp` 뿐 (`CMakeLists.txt` 기준)
- 마커 인식은 딥러닝이 아니라 `cv2.aruco` + `cv2.solvePnP` (CPU) 기반

나중에 GPU 추론이 필요해지면 `sudo apt install nvidia-jetpack` (apt 후보 `6.2.3+b81`)으로
JetPack SDK 구성요소를 따로 설치하면 됩니다. 그때 PyTorch는 **PyPI 기본 휠이 아니라
NVIDIA Jetson용 JetPack 6 빌드**(`https://pypi.jetson-ai-lab.io/` 또는 NVIDIA 공식 `.whl`)를
써야 합니다. x86_64 휠은 설치되지 않습니다.

## 1. 빌드용 swap 24GB (영구)

8GB RAM + zram(6 x 633.9M = 3.7GiB)만으로는 `px4_msgs`(400개 이상 메시지 생성)와
`ros_gz_bridge`(sim 프로필) 컴파일 구간에서 OOM이 납니다.

> **zram은 용량을 늘려주지 않습니다.** RAM을 압축해서 쓰는 것이라 7.4GiB 안에서
> 돌려막는 수준이고, 단일 컴파일이 2~3GB를 먹는 구간에는 도움이 되지 않습니다.
> 반드시 **디스크 swap**이 필요합니다.

```bash
sudo fallocate -l 24G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw,pri=-2 0 0' | sudo tee -a /etc/fstab
```

확인:

```bash
swapon --show
# /dev/zram0..5  partition  633.9M  PRIO 5
# /swapfile      file          24G  PRIO -2
free -h   # Swap: 27Gi
```

- `pri=-2`로 두면 빠른 zram을 먼저 쓰고 모자랄 때만 eMMC swap으로 넘어갑니다.
- `/etc/fstab` 한 줄이 **필수**입니다. 이게 없으면 재부팅 후 zram 3.7GiB만 남고,
  `ros_gz_bridge` 컴파일 중 머신이 멈추며 강제 리부팅됩니다(실제로 3회 발생).
- 실제 `sim` 빌드 중 swap 사용량은 **1.3GiB**였습니다(zram 1.1GiB + `/swapfile` 177MiB).

## 2. 기본 도구

```bash
sudo add-apt-repository -y universe
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release software-properties-common python3-pip cmake
```

| 패키지 | 버전 |
| --- | --- |
| `cmake` | 3.22.1 |
| `python3-pip` | 22.0.2 (Python 3.10) |

## 3. ROS 2 apt 저장소 등록

ROS 공식 `ros2-apt-source` deb로 키와 소스 목록을 등록했습니다(권장 방식).
NVIDIA L4T apt 저장소는 그대로 두었습니다.

```bash
ROS_APT_SOURCE_VERSION=$(curl -sSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -fsSL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
sudo apt-get install -y /tmp/ros2-apt-source.deb
sudo apt-get update
```

| 패키지 | 버전 |
| --- | --- |
| `ros2-apt-source` | 1.3.0~jammy |

등록 결과 `/etc/apt/sources.list.d/ros2.sources` 생성, `ros-humble-ros-base` 후보
`0.10.0-1jammy.20260916.125240` (**arm64**) 확인.

## 4. ROS 2 Humble + 빌드 도구

RViz·Gazebo가 필요 없는 온보드 컴퓨터이므로 `desktop`이 아니라 **`ros-base`** 를 설치했습니다.

```bash
sudo apt-get install -y ros-humble-ros-base ros-dev-tools ros-humble-eigen3-cmake-module libeigen3-dev
```

| 패키지 | 버전 | 아키텍처 |
| --- | --- | --- |
| `ros-humble-ros-base` | 0.10.0-1jammy.20260916.125240 | arm64 |
| `ros-humble-eigen3-cmake-module` | 0.1.1-4jammy.20260307.143609 | arm64 |
| `libeigen3-dev` | 3.4.0-2ubuntu2 | all |
| `python3-colcon-common-extensions` | 0.3.0-100 | all |
| `python3-rosdep` | 0.27.0-1 | all |

확인: `ROS_DISTRO=humble`, `ros2 pkg list | wc -l` → 200

`ros-base`가 `rclcpp`, `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`,
`launch`, `launch_ros`, `ament_index_python`, `python3-yaml` 등 각 `package.xml`의
의존성을 모두 포함합니다.

## 5. rosdep

```bash
sudo rosdep init
rosdep update
```

`/etc/ros/rosdep/sources.list.d/20-default.list` 생성, 사용자 캐시
`~/.ros/rosdep/sources.cache` 갱신 완료.

## 6. 인식기 Python 런타임 (저장소 내부 격리)

`requirements-detection.txt`의 고정 버전을 **저장소 안 `.deps/python`에만** 설치합니다.
`scripts/env.sh`가 `PYTHONPATH`에 이 경로를 앞세우므로, 시스템 Python과
JetPack/apt OpenCV에는 영향이 없습니다.

```bash
cd ~/INAIR/precision_landing
python3 -m pip install --only-binary=:all: --upgrade --target "$PWD/.deps/python" -r requirements-detection.txt
```

| 패키지 | 버전 | 휠 |
| --- | --- | --- |
| `numpy` | 1.26.4 | `numpy-1.26.4-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl` |
| `opencv-contrib-python-headless` | 4.10.0.84 | `opencv_contrib_python_headless-4.10.0.84-cp37-abi3-manylinux_2_17_aarch64.manylinux2014_aarch64.whl` |

둘 다 **aarch64 전용 휠**이며 `--only-binary=:all:`로 소스 빌드를 차단했습니다.

> **함정: `~/.local`의 numpy 버전.** PX4의 `Tools/setup/ubuntu.sh`는 `pip install --user`로
> numpy 2.x(2.2.6)를 설치합니다. 그러면 numpy 1.x에 맞춰 컴파일된 apt `python3-opencv`와
> `cv_bridge`가 ABI 불일치로 깨집니다
> (`ImportError: numpy.core.multiarray failed to import`, `AttributeError: _ARRAY_API not found`).
> PX4는 `numpy>=1.13`만 요구하므로 프로젝트 핀과 같은 버전으로 되돌리면 양쪽 다 동작합니다:
>
> ```bash
> python3 -m pip install --user "numpy==1.26.4"
> ```

### OpenCV 충돌 처리 방식

| 레이어 | 버전 | 변경 여부 |
| --- | --- | --- |
| apt `libopencv-*` (C++) | 4.5.4+dfsg-9ubuntu4 | **변경 없음** |
| apt `python3-opencv` (시스템 `cv2`) | 4.5.4+dfsg-9ubuntu4 | **변경 없음** (ROS 설치 시 신규 유입, 아래 참고) |
| 시스템 Python `numpy` (apt) | 1.21.5 | **변경 없음** |
| 프로젝트 `.deps/python` | cv2 4.10.0 / numpy 1.26.4 | 신규 |

- `pip install opencv-python`을 시스템에 설치하거나 덮어쓰지 **않았습니다.**
- `headless` 휠이라 GTK/Qt를 끌고 오지 않아 시스템 GUI 스택과 충돌하지 않습니다.
- 이 코드가 쓰는 `cv2.aruco.ArucoDetector` API는 OpenCV 4.7+ 에서만 제공되므로,
  apt의 4.5.4로는 동작하지 않습니다. 격리 설치가 필수입니다.

> **ROS 설치의 부수 효과**: `ros-humble-ros-base`가 `ros-humble-cv-bridge`를 통해
> apt `python3-opencv 4.5.4`를 함께 설치합니다(초기 `env_report.txt`에는 `cv2`가 없었음).
> 즉 **`source scripts/env.sh` 없이** `python3 -c "import cv2"`를 하면 4.5.4가 잡히고,
> `cv2.aruco.ArucoDetector`가 없어 인식기가 실패합니다.
> 노드를 직접 실행하는 모든 터미널에서 `env.sh`를 반드시 source하세요.
>
> ```bash
> $ env -u PYTHONPATH python3 -c "import cv2; print(cv2.__version__, cv2.__file__)"
> 4.5.4 /usr/lib/python3/dist-packages/cv2.cpython-310-aarch64-linux-gnu.so   # env.sh 없이
> $ source scripts/env.sh && python3 -c "import cv2; print(cv2.__version__)"
> 4.10.0                                                                      # env.sh 적용
> ```
>
> 어느 쪽도 상대를 덮어쓰지 않습니다. `PYTHONPATH` 우선순위로만 갈라집니다.

확인:

```bash
cd ~/INAIR/precision_landing && source scripts/env.sh
python3 -c "import cv2, numpy; print(cv2.__version__, cv2.__file__, numpy.__version__)"
# 4.10.0 /home/inair/INAIR/precision_landing/.deps/python/cv2/__init__.py 1.26.4
```

## 7. 워크스페이스 빌드

서브모듈(핀 고정):

| 서브모듈 | 커밋 | 태그 |
| --- | --- | --- |
| `src/px4_msgs` | `d3673f4` | 1.17.0 |
| `src/px4-ros2-interface-lib` | `7338c1d` | 2.2.1 |
| `src/ros_gz` | `6be1a56` | 0.244.20 (젯슨에서는 빌드하지 않음) |

```bash
cd ~/INAIR/precision_landing
git submodule update --init --recursive
env CMAKE_BUILD_PARALLEL_LEVEL=2 ./scripts/build.sh hitl_jetson
```

`build.sh hitl_jetson`은 인식기·제어기·HUD를 빌드합니다. 실기체용 `build.sh real`에는 CSI 카메라 패키지 `pl_camera`도 포함됩니다.

8GB RAM 환경이라 `CMAKE_BUILD_PARALLEL_LEVEL=2`로 낮췄습니다
(`build.sh` 기본값은 4, colcon `--parallel-workers`는 스크립트에서 2로 고정).

빌드 결과 — **6/6 성공, 총 19분 45초**:

| 패키지 | 시간 |
| --- | --- |
| `pl_detection` | 2.95s |
| `px4_msgs` | 12min 53s |
| `px4_ros2_cpp` | 5min 12s |
| `pl_control` | 1min 38s |
| `pl_viz` | 3.27s |
| `pl_bringup` | 1.96s |

확인:

```bash
source scripts/env.sh
ros2 pkg executables pl_detection pl_control pl_viz
# pl_detection aruco_tracker / pl_control precision_land / pl_viz monitor
```

### AMENT_PREFIX_PATH 경고

1. `WARNING: The path '.../.deps/apt/root/opt/ros/humble' in AMENT_PREFIX_PATH doesn't exist`
   — `env.sh`는 모든 프로필에서 이 경로를 추가하지만, `.deps/apt`는 `setup.sh sim`/`hitl_pc`가
   `gps_msgs`를 받을 때만 생성됩니다. 젯슨 프로필에는 `gps_msgs`가 필요 없습니다.

## 8. 실기체(`real`) 프로필 추가 구성

### 8.1 카메라·뷰어 패키지

```bash
sudo apt-get install -y ros-humble-usb-cam ros-humble-rqt-image-view
```

| 패키지 | 버전 | 아키텍처 |
| --- | --- | --- |
| `ros-humble-usb-cam` | 0.8.1-1jammy.20260909.232237 | arm64 |
| `ros-humble-rqt-image-view` | 1.2.0-2jammy.20260908.055541 | arm64 |

`docs/hardware_setting.md`가 지정한 USB 카메라 노드입니다. 현재 `/dev/video*`가 없으므로
카메라를 연결한 뒤 `ros2 run usb_cam usb_cam_node_exe`로 확인하세요.

### 8.2 UART 권한

`hardware.launch.py`가 `/dev/ttyTHS1`(`crw-rw---- root dialout`)을 사용합니다.

```bash
sudo usermod -aG dialout inair
```

`id inair`에 `20(dialout)` 추가 확인. **다음 로그인부터 적용**되므로
로그아웃/재로그인 또는 재부팅이 필요합니다.

### 8.3 Micro XRCE-DDS Agent

`hardware.launch.py`가 `MicroXRCEAgent serial -D /dev/ttyTHS1 -b 3000000`을 실행합니다.
(`hitl_jetson` 프로필에서는 PC의 Agent를 네트워크로 공유하므로 젯슨에 필요 없습니다.)

- **출처**: eProsima 공식 저장소 `https://github.com/eProsima/Micro-XRCE-DDS-Agent.git`
- **버전**: `v2.4.2` (커밋 `57d0862`) — PX4 공식 문서가 uXRCE-DDS 브리지에 권장하는 태그이며,
  이 저장소가 검증했다고 명시한 PX4 커밋 `ee99864`(`git describe` = `v1.18.0-beta1-236-gee99864808`)와 맞는 조합입니다.
  PX4 v1.18은 Micro-XRCE-DDS-Client **2.4.0**을 쓰고 `UXRCE_DDS_CLIENT_USE_DDS_V3` 기본값이 `n`이라 Agent 2.4.2와 호환됩니다
- **이유**: aarch64용 공식 바이너리 배포가 없어 소스 빌드했습니다

superbuild가 자체적으로 받아 빌드한 의존성 (`build/temp_install/` 안에만 설치됨):

| 구성요소 | 버전 |
|---|---|
| Fast-CDR | 1.1.1 |
| Fast-DDS (fastrtps) | **2.12.2** |
| Micro-CDR | 2.0.1 |
| Micro-XRCE-DDS-Client | 2.4.2 |
| foonathan_memory | v0.7-3 |

#### 상류 저장소 수정 1건 — Fast-DDS 태그 고정

v2.4.2의 `src/CMakeLists.txt`는 Fast-DDS를 **브랜치** `2.12.x`로 받도록 적혀 있는데,
eProsima가 그 브랜치를 삭제해 현재는 체크아웃이 실패합니다.

```
fatal: invalid reference: 2.12.x
CMake Error ... Failed to checkout tag: '2.12.x'
```

같은 계열의 불변 릴리스 태그 `v2.12.2`로 바꿔 해결했습니다
(`_fastdds_version`이 `2.12`로 적혀 있으므로 의도한 버전과 일치합니다).
Agent 버전을 올리는 대안(v2.4.3 → Fast-DDS `2.14.x`, v3.0.2 → `3.3.x`)도 가능하지만,
PX4 문서가 지정한 Agent 태그를 유지하는 쪽을 택했습니다.

```bash
sed -i 's/set(_fastdds_tag 2\.12\.x)/set(_fastdds_tag v2.12.2)/' ~/INAIR/micro-xrce-agent/src/CMakeLists.txt
```

#### 빌드와 PATH 등록

```bash
mkdir -p ~/INAIR/micro-xrce-agent
git clone -b v2.4.2 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/INAIR/micro-xrce-agent/src
sed -i 's/set(_fastdds_tag 2\.12\.x)/set(_fastdds_tag v2.12.2)/' ~/INAIR/micro-xrce-agent/src/CMakeLists.txt
cmake -S ~/INAIR/micro-xrce-agent/src -B ~/INAIR/micro-xrce-agent/build -DCMAKE_BUILD_TYPE=Release
cmake --build ~/INAIR/micro-xrce-agent/build -j2
# /usr/local 전체 설치 대신 바이너리 심볼릭 링크만 등록 (이유는 아래)
sudo ln -sfn ~/INAIR/micro-xrce-agent/build/MicroXRCEAgent /usr/local/bin/MicroXRCEAgent
```

**`sudo cmake --install`을 하지 않은 이유.** `DESTDIR` 드라이런으로 확인한 결과
`cmake --install`은 `/usr/local`에 **653개 파일**을 쓰고, 그중에는
`libfastrtps.so.2.12.2`가 포함됩니다. 이 기기의 ROS 2 Humble은
`ros-humble-fastrtps 2.6.12`를 쓰므로, `/usr/local/lib`가 `ldconfig` 캐시에 올라가면
Humble 노드가 2.12.2를 집을 위험이 생깁니다
(설치 전 `ldconfig -p | grep -c fastrtps` = 0, 즉 지금은 그런 경로가 없습니다).

빌드된 바이너리는 `readelf -d`로 확인한 RUNPATH가 빌드 트리
(`build/temp_install/*/lib`)를 절대경로로 가리키므로, 시스템에 라이브러리를 설치하지 않아도
그대로 실행됩니다. 요구사항은 `MicroXRCEAgent`가 PATH에 있는 것뿐이므로
시스템에 추가된 파일은 653개가 아니라 **링크 1개**입니다.

> **주의**: 이 방식은 빌드 디렉터리에 의존합니다.
> `~/INAIR/micro-xrce-agent/build`를 삭제하거나 옮기면 Agent가 실행되지 않습니다.

확인 결과:

```bash
$ ls -l /usr/local/bin/MicroXRCEAgent
lrwxrwxrwx 1 root root 55 ... /usr/local/bin/MicroXRCEAgent -> /home/inair/INAIR/micro-xrce-agent/build/MicroXRCEAgent
$ which MicroXRCEAgent
/usr/local/bin/MicroXRCEAgent
$ MicroXRCEAgent --help
Usage: 'MicroXRCEAgent <udp4|udp6|tcp4|tpc6|canfd|serial|multiserial|pseudoterminal> <<args>>'
$ ls /usr/local/lib/
python2.7  python3.10          # Fast-DDS 등이 들어가지 않았음을 확인
```

## 9. SITL(`sim` 프로필) 환경 — PX4 + Gazebo

젯슨 단독으로 SITL을 돌리기 위한 구성입니다. `hitl_jetson`/`hardware`만 쓸 거라면 이 절은 건너뛰어도 됩니다.

### 9.1 PX4-Autopilot 체크아웃

- **출처**: `https://github.com/PX4/PX4-Autopilot.git` (공식)
- **커밋**: `ee99864808e03c164975fc38c450fa22179fe0cb` (저장소가 핀으로 지정한 커밋)
- **`git describe --tags`**: `v1.18.0-beta1-236-gee99864808`
- **위치**: `~/PX4-Autopilot` (약 2.9G, 서브모듈 포함)

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout --detach ee99864808e03c164975fc38c450fa22179fe0cb
git submodule update --init --recursive
```

> **얕은 클론(`--depth`)을 쓰면 안 됩니다.** PX4 빌드가 `git describe --tags`로 버전 헤더를
> 생성하는데, 태그가 없으면 실패합니다.

의존성 설치는 PX4가 제공하는 스크립트를 **핀 커밋 기준으로** 받아서 실행했습니다
(`--no-nuttx`: 실기체 펌웨어 크로스 컴파일러는 SITL에 불필요):

```bash
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
```

> **함정 1 — numpy.** 이 스크립트가 `pip install --user`로 numpy 2.2.6을 올립니다. 6절의 경고를 참고해
> `numpy==1.26.4`로 되돌리세요. 되돌리지 않으면 apt `cv2`와 `cv_bridge`가 깨집니다.
>
> **함정 2 — OpenCV 런타임 누락.** PX4 configure가 이렇게 실패합니다:
>
> ```
> CMake Error at /usr/lib/cmake/opencv4/OpenCVModules.cmake:172 (message):
>   "opencv_core" references the file "/usr/lib/libopencv_core.so.4.8.0"
>   but this file does not exist.
> ```
>
> 원인은 **apt pin priority**입니다. NVIDIA 젯슨 저장소가 600, Ubuntu ports가 500이라
> `libopencv-dev`가 Ubuntu 4.5.4가 아니라 NVIDIA `4.8.0-1-g6371ee1`로 설치됩니다.
> 그런데 NVIDIA의 `-dev` 패키지는 런타임 `libopencv`를 `Depends`가 아니라 `Recommends`로만
> 걸어두기 때문에, `--no-install-recommends`를 쓰면 런타임이 빠집니다. 런타임만 추가하면 됩니다:
>
> ```bash
> sudo apt-get install -y libopencv   # NVIDIA 4.8.0 런타임, 67MB, CUDA 의존성 없음
> ```
>
> soname이 달라서(`.408` vs `.4.5d`) Ubuntu 4.5.4와 공존하며, apt `cv2`는 4.5.4 그대로입니다.
> `libopencv-dev`를 Ubuntu 4.5.4로 **다운그레이드하지 마세요** — NVIDIA 저장소 패키지의
> 버전을 바꾸는 행위입니다. PX4의 `optical_flow` 모듈이
> `find_package(OpenCV REQUIRED)`를 무조건 호출하므로 우회도 불가능합니다.

### 9.2 Gazebo Harmonic

- **출처**: OSRF 공식 저장소 `http://packages.osrfoundation.org/gazebo/ubuntu-stable`
- **버전**: `gz-harmonic 1.0.0-1~jammy` (arm64) → `gz sim` 8.15.0
- **이유**: `scripts/env.sh`가 `GZ_VERSION=harmonic`으로 고정

```bash
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update
sudo apt-get install -y gz-harmonic python3-gz-transport13 python3-gz-msgs10
```

확인:

```bash
gz sim --versions          # 8.15.0
gz sim -s -r --iterations 20 empty.sdf   # exit 0
```

렌더링은 Tegra EGL(`libnvidia-eglcore.so.540.5.0` + nvidia EGL ICD)로 GPU 가속됩니다.
실행 중 로그에 아래 경고가 뜨지만 **무해합니다** — Mesa가 dri2 경로를 먼저 시도하다
실패하고 NVIDIA EGL ICD로 넘어가는 것으로, 카메라 센서는 정상 렌더링됩니다.

```
libEGL warning: MESA-LOADER: failed to open nvidia-drm
libEGL warning: egl: failed to create dri2 screen
```

### 9.3 PX4 SITL 빌드

```bash
cd ~/PX4-Autopilot
GZ_VERSION=harmonic make px4_sitl_default -j3
```

결과 — **1221/1221, 8분 58초, 에러 0**:

| 산출물 | 값 |
| --- | --- |
| `build/px4_sitl_default/bin/px4` | 57MB |
| gz 플러그인 | 10개 (`libOpticalFlowSystem.so`, `libMovingPlatformController.so`, `libGstCameraSystem.so` 포함) |
| `build/px4_sitl_default/rootfs/gz_env.sh` | 생성됨 (`run.sh`가 source 함) |

### 9.4 `.deps/apt` 추가 패키지

`sim` 프로필은 `ros_gz_bridge`를 빌드하는데, 깨끗한 머신에서는 의존성 2개가 없어 실패합니다:

```
Could not find a package configuration file provided by "actuator_msgs"
```

> **`scripts/setup.sh`의 빈 구멍입니다.** 이 스크립트는 `sim`/`hitl_pc`에서 `gps_msgs`만
> `.deps/apt`로 받는데, `ros_gz_bridge/package.xml`은 15개를 요구하고 그중
> `actuator_msgs`와 `vision_msgs`가 `ros-humble-ros-base`에 포함되지 않습니다.

저장소의 기존 방식(sudo 없이 deb 추출)을 그대로 따랐습니다:

```bash
cd ~/INAIR/precision_landing/.deps/apt
apt-get download ros-humble-actuator-msgs ros-humble-vision-msgs
for d in ros-humble-actuator-msgs_*.deb ros-humble-vision-msgs_*.deb; do dpkg-deb -x "$d" root; done
```

| 패키지 | 버전 |
| --- | --- |
| `actuator_msgs` | 0.0.1-1jammy |
| `vision_msgs` | 4.1.1-1jammy |
| `gps_msgs` | 3.1.1-1jammy (`setup.sh`가 받음) |

### 9.5 `sim` 프로필 빌드

```bash
cd ~/INAIR/precision_landing
PX4_DIR=$HOME/PX4-Autopilot CMAKE_BUILD_PARALLEL_LEVEL=1 ./scripts/build.sh sim
```

결과 — **9/9 성공, 25분 20초**:

| 패키지 | 시간 |
| --- | --- |
| `ros_gz_bridge` | **25min 9s** |
| `pl_detection` | 3.99s |
| `px4_msgs` | 5.97s (캐시) |
| `ros_gz_interfaces` | 1.93s |
| `px4_ros2_cpp` | 0.93s |
| `pl_control` / `pl_viz` / `pl_bringup` | 각 1초 미만 (캐시) |
| `pl_camera` | 3.20s |

- 시간의 99%가 `ros_gz_bridge`입니다. 생성 소스가 거대해서 단일 컴파일이 수 GB를 씁니다.
  **1절의 24GB swap이 없으면 여기서 머신이 멈춥니다.**
- `CMAKE_BUILD_PARALLEL_LEVEL=1`로 낮춰야 안전합니다(기본값 4).

### 9.6 SITL 실행

```bash
cd ~/INAIR/precision_landing
HEADLESS=1 PX4_DIR=$HOME/PX4-Autopilot ./scripts/run.sh
```

- `HEADLESS=1`이면 `gz sim -g`(GUI)와 `rqt_image_view`를 띄우지 않습니다. 젯슨에서는 권장.
- 로그: `.run/{gazebo,px4,ros}.log` (DDS Agent 로그는 `ros.log`에 섞임)
- 기본 월드는 `SIM_WORLD=aruco_moving`(marker_size 1.0). 정지 마커는 `SIM_WORLD=aruco`(0.5).
- `pgrep -x px4`가 잡히거나 UDP `:8888`이 사용 중이면 실행을 거부합니다. 재실행 전에 완전 종료를 확인하세요.

### 9.7 `scripts/test_flight.py` 버그 수정

```
NameError: name 'paused_detector' is not defined   (test_flight.py:155)
```

`paused_detector`는 `--exercise-handover`를 줬을 때만 123번 줄에서 대입되는데
`finally`의 155번 줄은 플래그와 무관하게 항상 읽습니다. 그래서 **비행이 성공해도
핸드오버 테스트를 안 돌리면 무조건 비정상 종료**하고, `PASS:` 출력과 `velocity(0)` 복구,
`rclpy.shutdown()`이 모두 건너뛰어집니다. 72번 줄에서 초기화하도록 고쳤습니다:

```diff
-start_future = repeat_future = None
+start_future = repeat_future = paused_detector = None
```

## 10. 설치하지 않은 것과 그 이유

| 항목 | 이유 |
| --- | --- |
| PyTorch / torchvision / onnxruntime-gpu | 저장소 어디에도 import 없음 (0절 참고) |
| CUDA / cuDNN / TensorRT (`nvidia-jetpack`) | 위와 동일. 설치 시 `nvidia-l4t-*`가 함께 갱신되므로 필요할 때 별도 판단 |
| pip `opencv-python` (시스템 설치) | JetPack/apt OpenCV와 충돌. `.deps/python` 격리로 대체 (6절) |
| NuttX 크로스 컴파일러 | PX4 `ubuntu.sh --no-nuttx`로 제외. SITL에는 불필요하고 실기체 펌웨어는 젯슨에서 빌드하지 않음 |

> **이전 문서에서 "설치하지 않음"으로 적혀 있던 Gazebo Harmonic, PX4-Autopilot 체크아웃,
> `gps_msgs`, `pymavlink`는 9절에서 모두 설치했습니다.** 젯슨 단독 SITL을 돌리기 위해서입니다.
> `pymavlink`는 PX4 `ubuntu.sh`가 `~/.local`에 함께 설치합니다.

## 11. 실행 환경 변수

HITL(PC와 분산 SITL)에서 사용하는 각 터미널마다:

```bash
cd ~/INAIR/precision_landing
source scripts/env.sh
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

실행:

```bash
# PC를 먼저 띄운 뒤
ros2 launch pl_bringup hitl_jetson.launch.py

# 실기체
ros2 launch pl_bringup hardware.launch.py
```

젯슨 단독 SITL(`sim`)은 `run.sh`가 `ROS_DOMAIN_ID=42`와 `GZ_PARTITION=precision_landing`을
스스로 설정하므로 위 변수들을 따로 export할 필요가 없습니다:

```bash
HEADLESS=1 PX4_DIR=$HOME/PX4-Autopilot ./scripts/run.sh
```

- PC와 젯슨이 같은 Wi-Fi/LAN에 있어야 하고, AP의 Client/AP isolation을 끄고
  DDS 검색 multicast와 UDP를 허용해야 합니다.
- 양쪽 시스템 시간을 동기화하세요.
- `hardware.launch.py`는 `hardware.yaml`의 `camera.calibration_verified`와
  `camera.extrinsics_verified`가 모두 `true`가 아니면 의도적으로 실행을 거부합니다.
  카메라 보정과 장착 변환을 검증한 뒤 바꾸세요(현재 둘 다 `false`).

## 12. 스모크 테스트 결과 (2026-11-12)

### 12.1 `hitl_jetson` 프로필 단독 기동

PC 없이 젯슨만으로 `hitl_jetson` 프로필을 기동한 결과:

```
[INFO] [aruco_tracker-1]: process started
[INFO] [precision_land-2]: process started
[INFO] [monitor-3]: process started
[precision_land-2] [INFO] descent_vel: 0.600000
[precision_land-2] [DEBUG] Waiting for FMU...
[precision_land-2] [DEBUG] timeout while waiting for FMU publisher discovery
[precision_land-2] px4_ros2::Exception: Registration failed
```

- 세 노드 모두 정상 기동, 파라미터 로드 확인
- `aruco_tracker`(cv2 4.10 로딩 포함)와 `monitor`는 clean 종료
- `precision_land`의 `Waiting for FMU` → `Registration failed`는 **PC측 PX4 SITL과
  DDS Agent가 없을 때의 정상 동작**입니다 (README 6.3 ④ 참고)

### 12.2 `sim` 프로필 SITL — 전 구간 통과

`HEADLESS=1 ./scripts/run.sh`로 기동한 뒤 `scripts/test_flight.py`를 돌린 결과입니다.

기동 상태:

| 항목 | 결과 |
| --- | --- |
| PX4 | `INFO [px4] Startup script returned successfully` |
| 월드 / 기체 | `aruco_moving` / `x500_mono_cam_down_0` |
| uXRCE-DDS | `init UDP agent IP:127.0.0.1, port:8888`, `UXRCE_DDS_DOM_ID: 0 -> 42` |
| ROS 2 노드 | 6개: `aruco_tracker`, `camera_bridge`, `debug_image_bridge`, `landing_monitor`, `precision_land`, `tag_pose_visualizer` |
| 외부 모드 등록 | `Registering 'PrecisionLandCustom'` → `Got RegisterExtComponentReply` |
| 카메라 스트림 | `/camera/image_raw` **13.9 Hz** (GPU 렌더링 정상) |

비행 테스트 2종 모두 통과:

| 테스트 | 명령 | 소요 | 결과 |
| --- | --- | --- | --- |
| 정지 표적 | `python3 scripts/test_flight.py --speed 0` | 34.4s | `phase: passed`, 자동 disarm |
| 이동 표적 | `python3 scripts/test_flight.py --speed 0.3` | 32.6s | `PASS: landing and automatic disarm (target speed 0.3 m/s)`, 종료 코드 0 |

이동 표적 시퀀스(요약):

```
16.2s  precision  nav=23  z=-2.21   PrecisionLandCustom 진입
17.8s  tracking   tags=6  y=0.84    마커 포착
26.3s  tracking   tags=96 z=-1.34   하강 추종
30.4s  tracking  tags=114 y=4.03    착지 (landed=true)
32.6s  passed    tags=114 y=4.55    자동 disarm (armed=false)
```

플랫폼이 0.3 m/s로 계속 이동하는 동안 y축으로 4.5m를 추종하며 착륙했습니다.

### 아직 검증되지 않은 것

- 실제 Wi-Fi 환경에서 PC↔젯슨 JPEG 영상 처리율·지연
- 실기체 FC와의 UART 연결 (`/dev/ttyTHS1`, 3000000 baud)
- CSI/USB 카메라 입력과 카메라 보정값 (`hardware.yaml`은 전부 placeholder)
- GUI 모드 SITL (`HEADLESS` 미설정). 헤드리스만 검증했습니다
- `test_flight.py --exercise-handover` (타깃 소실 → Hold 인계 시나리오)
