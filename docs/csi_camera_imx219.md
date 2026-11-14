# CSI 카메라(IMX219) — Jetson Orin Nano + JetPack 6

Raspberry Pi Camera Module v2(**IMX219**)를 CAM0에 연결해 ROS 2 토픽으로 영상을
발행하기까지의 기록입니다.

- 작업일: 2026-11-13
- 결과: **성공** — `/camera/image_raw` 1920x1080, Argus ISP 자동 노출·화이트밸런스
- IMX219는 **JetPack 정품 지원 센서**라 드라이버를 따로 빌드할 필요가 없습니다.
  OV5647(Pi Camera v1)을 쓰려면 드라이버 포팅이 필요합니다 →
  [csi_camera_ov5647.md](csi_camera_ov5647.md)

## 0. 두 센서 비교

같은 CAM0 커넥터에 번갈아 꽂을 수 있습니다. 오버레이만 바꾸면 됩니다.

| | OV5647 (Pi Cam v1) | **IMX219 (Pi Cam v2)** |
| --- | --- | --- |
| JetPack 기본 지원 | 없음 (드라이버 포팅 필요) | **있음** (`nv_imx219`) |
| 픽셀 포맷 | `BG10` (BGGR 베이어) | `RG10` (RGGB 베이어) |
| 최대 해상도 | 2592x1944 @15 | 3280x2464 @21 |
| 게인 범위 | 16 ~ 1023 | 16 ~ **171** (훨씬 낮음) |
| 노출 범위 | ~131 ~ 500000 us | 13 ~ 683710 us |
| Argus(하드웨어 ISP) | **불가** (Orin에서 미지원) | **가능** |
| 부팅 시 첫 프로브 | 실패 → 재로드 필요 | 실패 → 재로드 필요 (5.1절) |

IMX219를 권합니다. Argus가 동작해 자동 노출·화이트밸런스가 되고, raw 베이어를
직접 디베이어할 필요가 없습니다.

## 1. 오버레이 적용

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

**Configure Jetson 22pin CSI Connector** → `Camera IMX219-A` → **Save pin changes** → 재부팅.

- CAM0 = CSI 포트 **A** 입니다. 파일명의 `-A`가 CAM0, `-C`가 CAM1입니다.
- 40핀 헤더 UART와 **동시에 쓸 수 있습니다.** jetson-io가 두 오버레이를 `OVERLAYS`
  한 줄에 모아 씁니다 → [pixhawk_uart_link.md](pixhawk_uart_link.md) 3절

비대화형으로도 가능합니다.

```bash
sudo /opt/nvidia/jetson-io/config-by-hardware.py -n 2="Camera IMX219-A"
```

확인:

```bash
ls /sys/firmware/devicetree/base/bus@0/cam_i2cmux/i2c@0/   # rbpcv2_imx219_a@10
ls /sys/bus/i2c/devices/ | grep 9-0010
lsmod | grep nv_imx219
ls -l /dev/video0
```

커널 로그가 이렇게 나오면 성공입니다.

```
imx219 9-0010: tegracam sensor driver:imx219_v2.0.6
tegra-camrtc-capture-vi tegra-capture-vi: subdev imx219 9-0010 bound
```

## 2. 지원 모드

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

| `sensor_mode` | 해상도 | fps |
| --- | --- | --- |
| 0 | 3280x2464 | 21 |
| 1 | 3280x1848 | 28 |
| 2 | 1920x1080 | 30 |
| 3 | 1640x1232 | 30 |
| 4 | 1280x720 | 60 |

`sensor_mode`와 `--set-fmt-video`의 해상도가 **일치해야 합니다.** 불일치하면
Argus가 재협상하면서 화각과 프레임레이트가 바뀝니다.

## 3. Argus(하드웨어 ISP) 설치

Argus를 쓰면 **자동 노출·화이트밸런스**가 적용된 정상 컬러 영상이 나옵니다.
JetPack 기본 설치에는 GStreamer 플러그인이 빠져 있습니다.

```bash
gst-inspect-1.0 nvarguscamerasrc        # "No such element" 면 미설치
```

```bash
sudo apt-get install -y --no-install-recommends \
    nvidia-l4t-gstreamer=36.5.0-20260115194252
```

> **버전을 반드시 고정하세요.** apt 기본 후보는 `36.5.2`인데, 그걸 설치하면 다른
> `nvidia-l4t-*` 패키지가 함께 올라갈 수 있습니다. 현재 설치된 것들과 같은
> `36.5.0-20260115194252`로 지정해야 합니다. 설치 전 확인:
>
> ```bash
> apt-get -s install nvidia-l4t-gstreamer=36.5.0-20260115194252
> # "0 upgraded, 1 newly installed, 0 to remove" 여야 한다
> ```

확인:

```bash
gst-inspect-1.0 nvarguscamerasrc | head -3
dpkg -l | grep '^ii  nvidia-l4t' | awk '{print $3}' | sort -u
# 36.5.0-20260115194252 만 나와야 한다 (커널 패키지 제외)
```

캡처 시험:

```bash
gst-launch-1.0 -q nvarguscamerasrc num-buffers=60 \
  ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' \
  ! nvvidconv ! 'video/x-raw,format=I420' ! jpegenc ! multifilesink location=test.jpg
```

> **Argus 세션은 한 번에 하나뿐입니다.** 미리보기나 ROS 노드가 떠 있으면
> `Failed to create CaptureSession` / `Error AlreadyAllocated`가 납니다.
> 다른 쪽을 먼저 닫으세요.

## 4. 함정 — `override_enable`

**raw(V4L2) 경로에서 노출·게인을 직접 다룰 때만 해당합니다.** Argus를 쓰면 무관합니다.

### 증상

`exposure`를 8배 넘게 바꿔도 밝기가 전혀 변하지 않습니다. 더 나쁜 건
`v4l2-ctl --get-ctrl=exposure`로 읽으면 **설정한 값이 그대로 보인다**는 점입니다.

```
override_enable=0 (기본값)
  exposure= 4000 us -> raw mean 12876.9
  exposure=16000 us -> raw mean 12878.7
  exposure=33000 us -> raw mean 12873.4     변화 없음
```

### 원인

Tegra VI 드라이버가 **스트리밍을 시작하는 순간 노출·게인을 센서 모드 기본값으로
되돌립니다.** IMX219의 기본 노출은 2495us라 실내에서는 거의 깜깜합니다.

### 해결

`override_enable=1`을 다른 컨트롤보다 **먼저** 설정합니다.

```bash
v4l2-ctl -d /dev/video0 \
  --set-ctrl=override_enable=1,sensor_mode=2,exposure=32000,gain=171 \
  --set-fmt-video=width=1920,height=1080,pixelformat=RG10 \
  --stream-mmap --stream-count=15 --stream-to=frames.raw
```

```
override_enable=1
  exposure= 4000 us -> 신호 18.6%  (8bit 환산  60.7)
  exposure=16000 us -> 신호 57.3%  (8bit 환산 153.5)
  exposure=33000 us -> 신호 67.7%  (8bit 환산 178.6)
```

### 그 밖의 raw 경로 주의사항

- `exposure`는 **마이크로초**, `gain`은 **1/16 단위**(16 = 1배).
- 노출은 프레임 길이를 넘을 수 없습니다. 30fps면 약 33ms가 상한입니다.
  더 길게 가려면 `frame_rate`를 낮춰야 합니다.
- IMX219는 게인 상한이 **171**(약 10.7배)로 OV5647(1023, 64배)보다 훨씬 낮습니다.
  어두운 실내에서는 노출을 길게 가져가야 합니다.
- raw 데이터에는 **블랙 레벨 4096**(10비트 64를 16비트로 좌측 정렬)이 깔려 있습니다.
  그냥 `>> 8`만 하면 대비가 죽습니다. 빼고 스케일하세요.

## 5. 알려진 문제

### 5.1 부팅 직후 첫 프로브 실패 (미해결)

```
imx219 9-0010: imx219_board_setup: error during i2c read probe (-121)
imx219 9-0010: board setup failed
imx219: probe of 9-0010 failed with error -121
```

`-121`은 `EREMOTEIO`(I2C NACK)입니다. **OV5647에서도 똑같이 발생합니다.**
센서 종류와 무관한 **보드의 부팅 타이밍 문제**로 보입니다.

회피:

```bash
sudo modprobe -r nv_imx219 && sudo modprobe nv_imx219
```

재로드하면 정상 인식되고 `/dev/video0`이 생깁니다. 항구적 해결책(systemd oneshot
유닛 또는 드라이버 재시도 루프)은 아직 적용하지 않았습니다.

### 5.2 Argus 단일 세션

3절 참고. 미리보기와 ROS 노드를 동시에 띄울 수 없습니다.

## 6. 도구

### 6.1 미리보기 — `scripts/camera_preview/imx219_preview.py`

```bash
python3 scripts/camera_preview/imx219_preview.py                 # auto: argus -> raw
python3 scripts/camera_preview/imx219_preview.py --backend raw   # ISP 없이 raw 경로
python3 scripts/camera_preview/imx219_preview.py --mode 4        # 1280x720 @60fps
```

- `argus` 백엔드는 ISP를, `raw` 백엔드는 V4L2 + 소프트웨어 디베이어를 씁니다.
  `raw`에서는 `override_enable=1`이 자동으로 적용됩니다.
- 키: `q` 종료, `s` 스냅샷, `g` 흑백 전환, `[`/`]` 노출, `-`/`=` 게인(raw 전용)
- HUD에 해상도·fps·노출·게인·평균 밝기·ArUco 검출 수가 표시됩니다.
- `source scripts/env.sh` 한 셸에서 실행해도 동작합니다. `.deps/python`의 OpenCV는
  headless 빌드라 창을 못 띄우는데, 스크립트가 그 경로를 `PYTHONPATH`에서 빼고
  자기 자신을 다시 실행합니다.

OV5647용 뷰어는 `scripts/camera_preview/ov5647_preview.py`입니다(BG10/BGGR, 모드표가 다름).

### 6.2 ROS 2 노드 — `pl_camera/csi_camera`

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=42
ros2 launch pl_camera camera_csi.launch.py
```

`sensor_msgs/Image`(bgr8 또는 mono8)와 `sensor_msgs/CameraInfo`를 발행해,
`pl_detection/aruco_tracker`가 Gazebo 브리지와 똑같이 받아 쓸 수 있게 합니다.

| | 값 |
| --- | --- |
| 기본 토픽 | `/camera/image_raw`, `/camera/camera_info` |
| 기본 모드 | 1920x1080 @30fps, bgr8 |
| 실측 발행률 | **24.8 fps** |
| 설정 파일 | `src/pl_camera/config/csi_camera.yaml` |

주요 파라미터: `sensor_id`, `width`, `height`, `fps`, `flip_method`(장착 방향),
`encoding`(`bgr8`/`mono8`), `camera_info_url`, `wbmode`, `exposure_time_range`, `gain_range`.

> **GStreamer를 PyGObject(`gi`)로 몹니다.** 프로젝트가 `.deps/python`에 고정한 OpenCV
> 휠은 `GStreamer: NO`로 빌드돼 있어 `cv2.VideoCapture(gst_pipeline)`가 열리지
> 않습니다. `gi`는 시스템 dist-packages에 있어 `scripts/env.sh` 아래서도 import됩니다.
> 추가 설치가 필요 없습니다.

### 6.3 카메라 보정이 아직 없습니다

`camera_info_url`이 비어 있으면 `k`를 0으로 채운 `CameraInfo`를 발행합니다.
`aruco_tracker`는 `k[0][0]`이 0이 아닐 때까지 구독을 유지하므로,
**틀린 내부 파라미터를 쓰는 대신 기다립니다.**

그 상태에서도 **마커 검출과 오버레이는 동작합니다.** 검출은 내부 파라미터 없이
가능하고, 3D 자세(`/target_pose`)만 보정이 필요합니다.

```bash
# 카메라 + 검출 + 뷰어 (터미널 3개)
ros2 launch pl_camera camera_csi.launch.py
ros2 run pl_detection aruco_tracker --ros-args -p aruco_id:=0 -p dictionary:=2 -p marker_size:=0.1
ros2 run rqt_image_view rqt_image_view /image_proc
```

`/pl/detection/status`에 `detected`와 `pose_valid`가 나옵니다.
보정 후 `camera_info_url`에 결과 파일을 넣으면 `pose_valid`가 `true`로 바뀝니다.

`src/pl_bringup/config/hardware.yaml`의 `camera.calibration_verified`와
`camera.extrinsics_verified`가 둘 다 `true`가 아니면 `hardware.launch.py`는
의도적으로 실행을 거부합니다.

## 7. 아직 검증되지 않은 것

- 카메라 내부 파라미터 보정 (`camera_calibration`)
- 장착 변환(extrinsics)과 `flip_method` 실측
- 3280x2464 등 고해상도 모드에서의 발행률
- 실기체 진동 환경에서의 롤링 셔터 영향

관련 문서: [csi_camera_ov5647.md](csi_camera_ov5647.md) ·
[pixhawk_uart_link.md](pixhawk_uart_link.md) ·
[camera_inputs_stage1.md](camera_inputs_stage1.md)
