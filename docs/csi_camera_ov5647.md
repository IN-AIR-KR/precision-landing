# CSI 카메라(OV5647) 연결 — Jetson Orin Nano + JetPack 6

Raspberry Pi Camera Module v1(OmniVision **OV5647**)을 Jetson Orin Nano 개발키트의
CAM0 커넥터에 연결해 `/dev/video0`로 영상을 받기까지의 기록입니다.

- 작업일: 2026-11-12
- 결과: **성공** — 1920x1080 @30fps raw Bayer 캡처 확인
- 전제: JetPack 기본 설치만으로는 **동작하지 않습니다.** 커널 모듈 빌드와
  디바이스 트리 오버레이가 필요합니다.

## 0. 환경

| 항목 | 값 |
| --- | --- |
| 보드 | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super |
| `compatible` | `nvidia,p3768-0000+p3767-0005-super` |
| 모듈 / 캐리어 | P3767-0005 / P3768-0000 |
| L4T | R36.5.0 (JetPack 6.2.2) |
| 커널 | 5.15.185-tegra |
| 카메라 | Raspberry Pi Camera Module v1 (OV5647, 5MP) |
| 커넥터 | CAM0 (22핀). 파이 카메라는 15핀이므로 **15→22핀 변환 리본 필요** |

## 1. 왜 기본 JetPack으로는 안 되는가

### 1.1 증상

카메라를 꽂고 부팅해도 아무것도 생기지 않습니다.

```bash
$ ls /dev/video*
ls: cannot access '/dev/video*': No such file or directory
```

`lsusb`에도 없고(USB 카메라가 아니므로 당연), 센서 드라이버 모듈도 로드되지 않습니다.

### 1.2 원인 1 — 디바이스 트리 오버레이 미적용

젯슨은 **CSI 카메라를 자동 감지하지 않습니다.** 디바이스 트리에 센서를 정적으로
선언해야 합니다. 초기 상태의 `/boot/extlinux/extlinux.conf`에는 `OVERLAYS` 줄이
없었고, 카메라용 I2C 버스도 꺼져 있었습니다.

```bash
$ grep -E 'i2c@3190000|i2c@31c0000' <(dtc -I dtb -O dts /boot/dtb/kernel_*.dtb)
i2c@3190000   disabled
i2c@31c0000   disabled

$ find /proc/device-tree -name tegra-camera-platform
(없음)
```

### 1.3 원인 2 — OV5647 드라이버가 L4T에 없음

```bash
$ modinfo ov5647
modinfo: ERROR: Module ov5647 not found.

$ find /lib/modules/$(uname -r) -iname 'ov*'
/lib/modules/5.15.185-tegra/kernel/drivers/media/i2c/ov5645.ko
```

`ov5645`는 **이름만 비슷한 다른 센서**입니다. 커널 설정에 옵션 자체는 있으나
빌드가 꺼져 있습니다.

```bash
$ zcat /proc/config.gz | grep OV5647
# CONFIG_VIDEO_OV5647 is not set
```

설령 빌드해도 메인라인 `ov5647.c`는 표준 V4L2 subdev 드라이버라 Tegra의 VI에
바인딩되지 않습니다. 젯슨은 NVIDIA 독자 **tegracam** API에 맞춘 드라이버만 받습니다.

### 1.4 `jetson-io`가 지원하는 센서

```
Header 2: Jetson 22pin CSI Connector
  1. Camera IMX219 Dual        5. Camera IMX477 Dual
  2. Camera IMX219-A           6. Camera IMX477-A
  3. Camera IMX219-A and IMX477-C   7. Camera IMX477-A and IMX219-C
  4. Camera IMX219-C           8. Camera IMX477-C
```

**IMX219와 IMX477뿐입니다.** OV5647은 목록에 없으므로 `jetson-io`로는 해결할 수
없습니다.

> 참고: 파이에서 같은 카메라가 바로 잡히는 이유는, 라즈베리파이 재단이 `ov5647`
> 드라이버를 커널에 포함시키고 펌웨어가 I2C로 센서를 자동 감지해 오버레이를
> 올려주기 때문입니다. 디베이어도 VideoCore ISP가 처리합니다. 젯슨은 이 세 가지가
> 모두 없습니다.

## 2. 해결 경로

서드파티 tegracam 포팅 프로젝트를 사용했습니다.

- 저장소: <https://github.com/akandr/ov5647-jetson>
- 사용한 커밋: `79927cf`
- 라이선스: GPL-2.0 (NVIDIA `imx219.c` 기반 + 메인라인 `ov5647.c`의 레지스터 시퀀스)

> **공식 지원표에는 Orin이 JetPack 7(L4T r39)로만 적혀 있습니다.**
> 우리 환경은 JetPack 6(R36.5)인데, 아래와 같이 **소스 수정 없이** 동작했습니다.
> 저장소의 `install.sh`는 Orin에서 R39를 강제하므로 **사용하지 않고** 수동으로 진행했습니다.

### 2.1 사전 호환성 검증

설치 전에 JetPack 6 베이스 DTB가 오버레이의 요구를 만족하는지 확인했습니다.

| 오버레이가 요구하는 것 | JetPack 6 베이스 DTB | |
| --- | --- | --- |
| `/tegra-capture-vi` | 존재 (`nvidia,tegra-camrtc-capture-vi`) | OK |
| `/bus@0/host1x@13e00000/nvcsi@15a00000` | 존재 | OK |
| `&cam_i2c` | `__symbols__`에 `/bus@0/i2c@3180000` | OK |
| `&gpio_aon` | `__symbols__`에 `/bus@0/gpio@c2f0000` | OK |
| `&gpio` | `__symbols__`에 `/bus@0/gpio@2200000` | OK |
| 보드 `compatible` | 오버레이 목록에 `...p3767-0005-super` 포함 | OK |

JetPack 6 **순정** IMX219-A 오버레이를 디컴파일해 핀 설정도 대조했습니다. 전부 동일합니다.

| 항목 | 순정 IMX219-A | OV5647 오버레이 |
| --- | --- | --- |
| `mux-gpios` | `0x13` | `0x13` |
| `reset-gpios` (CAM0_PWDN) | `0x3e` | `0x3e` |
| `tegra_sinterface` | `serial_b` | `serial_b` |
| `lane_polarity` | `6` | `6` |
| `port-index` | `1` | `1` |
| `discontinuous_clk` | `yes` | `yes` |

> 여기서 **CAM0 = CSI 포트 A**(`serial_b`, `port-index 1`)라는 것도 확정됩니다.
> 오버레이 파일명의 `-A`가 CAM0, `-C`가 CAM1입니다.

## 3. 설치 절차

### 3.1 소스 받기

```bash
git clone https://github.com/akandr/ov5647-jetson.git
cd ov5647-jetson
git checkout 79927cf
```

### 3.2 커널 모듈 빌드

저장소 `Makefile`은 JetPack 7의 `/usr/src/nvidia/nvidia-public`을 찾습니다.
JetPack 6에서는 같은 구조가 **`/usr/src/nvidia/nvidia-oot`**에 있으므로
`NVPUB`만 바꿔주면 됩니다.

```bash
make -C driver NVPUB=/usr/src/nvidia/nvidia-oot
```

필요한 패키지는 JetPack에 이미 들어 있습니다
(`nvidia-l4t-kernel-headers`, `nvidia-l4t-kernel-oot-headers`).

```
/usr/src/nvidia/nvidia-oot/include/media/tegracam_core.h
/usr/src/nvidia/nvidia-oot/Module.symvers      (camera_common_* 심볼 616개)
/lib/modules/5.15.185-tegra/build -> .../kernel-source
```

결과:

```
CC [M]  ov5647.o
LD [M]  ov5647.ko          (117,032 bytes, 경고·에러 0)

$ modinfo ./ov5647.ko
vermagic: 5.15.185-tegra SMP preempt mod_unload modversions aarch64
depends:  tegra-camera,videodev
```

> **JetPack 6에서 소스 수정이 필요 없는 이유.** 저장소의 `driver/compat/nvidia/conftest.h`가
> `NV_*` 매크로를 커널 버전으로 분기합니다. JetPack 6 헤더가 실제로 쓰는 것은
> `NV_V4L2_ASYNC_CONNECTION_STRUCT_PRESENT` 하나뿐이고, 6.6 미만에서는 정의되지
> 않아야 하는데 그대로 동작합니다. 저자가 "구조체 레이아웃이 깨질 수 있다"고 경고한
> `NV_TEGRA_PMC_IO_PAD_POWER_ENABLE_PRESENT`는 JetPack 6 헤더에서 아예 쓰지 않습니다.

### 3.3 디바이스 트리 오버레이 컴파일

```bash
dtc -@ -I dts -O dtb -o ov5647-jetson.dtbo dt/orin/tegra234-p3767-ov5647-overlay.dts
```

`dtc`가 그래프 관련 경고를 내지만 정상입니다(오버레이 프래그먼트 특성).

> `fdtoverlay`로 베이스 DTB에 미리 병합하는 방법도 되지만, JetPack 6의
> `extlinux.conf`는 `OVERLAYS` 키를 지원하므로 **병합 없이 dtbo를 직접 지정**하는
> 쪽이 깔끔합니다. `jetson-io`가 쓰는 방식과 같습니다.

### 3.4 설치

```bash
# 커널 모듈
sudo install -d /lib/modules/$(uname -r)/updates
sudo install -m 0644 driver/ov5647.ko /lib/modules/$(uname -r)/updates/ov5647.ko
sudo depmod -a $(uname -r)

# 오버레이
sudo install -m 0644 ov5647-jetson.dtbo /boot/ov5647-jetson.dtbo

# 부팅 설정 백업
sudo cp -a /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.orig-ov5647
```

### 3.5 부팅 항목 추가

`/boot/extlinux/extlinux.conf`에 **새 항목을 추가하고** `DEFAULT`를 바꿉니다.
기존 항목은 지우지 않고 폴백으로 남깁니다.

```
TIMEOUT 30
DEFAULT ov5647

LABEL ov5647
      MENU LABEL OV5647 (Pi Camera v1) on CAM0
      LINUX /boot/Image
      FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb
      INITRD /boot/initrd
      APPEND ${cbootargs} root=PARTUUID=... (기존과 동일)
      OVERLAYS /boot/ov5647-jetson.dtbo
```

설치 후 부팅 메뉴 구성:

| 항목 | 오버레이 | 역할 |
| --- | --- | --- |
| `ov5647` | `/boot/ov5647-jetson.dtbo` | **기본값** |
| `JetsonIO` | IMX219-A (jetson-io가 생성) | 폴백 |
| `primary` | 없음 | 최종 폴백 |

> `TIMEOUT 30`이라 부팅 시 30초 메뉴가 뜹니다. 부팅이 안 되면 모니터를 보고
> `primary kernel`을 고르면 원래대로 올라옵니다.
>
> IMX219-A 오버레이와 OV5647 오버레이는 **같은 CSI 포트를 쓰므로 동시 적용이
> 불가능합니다.** 새 항목은 `OVERLAYS`를 우리 것으로만 채웁니다.

재부팅하면 적용됩니다.

## 4. 검증

### 4.1 센서 인식

```
[ 372.874905] ov5647 9-0036: OV5647 detected (chip id 0x5647)
[ 372.890253] tegra-camrtc-capture-vi tegra-capture-vi: subdev ov5647 9-0036 bound
[ 372.891115] ov5647 9-0036: detected ov5647 sensor
```

```bash
$ ls -l /dev/video0
crw-rw----+ 1 root video 81, 0 /dev/video0

$ cat /sys/class/i2c-adapter/i2c-9/name
i2c-2-mux (chan_id 0)          # cam_i2cmux가 만든 채널

$ ls -l /sys/bus/i2c/devices/9-0036/of_node
-> .../bus@0/cam_i2cmux/i2c@0/rbpcv1_ov5647_a@36
```

### 4.2 지원 모드

```bash
$ v4l2-ctl -d /dev/video0 --list-formats-ext
[0]: 'BG10' (10-bit Bayer BGBG/GRGR)
    2592x1944 @ 15 fps     # sensor_mode=0
    1920x1080 @ 30 fps     # sensor_mode=1
    1296x972  @ 30 fps     # sensor_mode=2 (2x2 binned)
    640x480   @ 90 fps     # sensor_mode=3
    1280x720  @ 60 fps     # sensor_mode=4 (binned, 16:9 crop)
```

### 4.3 실제 캡처

```bash
v4l2-ctl -d /dev/video0 \
  --set-ctrl=sensor_mode=1,exposure=16000,gain=64 \
  --set-fmt-video=width=1920,height=1080,pixelformat=BG10 \
  --stream-mmap --stream-count=30 --stream-to=frames.raw
```

30프레임 = 124,416,000 bytes (프레임당 1920 x 1080 x 2). 디베이어 후 통계:

```
16bit raw  min/max/mean:  384 / 65535 / 13435.4
gray       min/max/mean:    4 / 255 / 53.0
표준편차(장면 대비): 37.6
```

검은 화면이나 노이즈가 아닌 실제 장면이 찍혔습니다(천장 타일, 조명, 창틀 식별 가능).

### 4.4 주의할 제어값

- `sensor_mode`와 `--set-fmt-video`의 해상도가 **일치해야 합니다.**
  불일치 시 Orin에서는 타임아웃이 납니다.
- `exposure`는 **마이크로초**. 모듈 로드 직후 최솟값이라 설정 없이 캡처하면 거의 검습니다.
- `gain`은 **1/16 단위**(16 = 1배).
- `frame_rate`는 100만분의 1 fps 단위. 스트리밍 전 설정은 시작 시 리셋됩니다.
- 1296x972 raw는 `bytesperline=2624`를 명시해야 줄이 밀리지 않습니다.

## 5. 알려진 문제

### 5.1 부팅 직후 첫 프로브 실패 (미해결)

```
[  11.062434] ov5647 9-0036: ov5647_board_setup: error during i2c read probe (-121)
[  11.062497] ov5647 9-0036: board setup failed
[  11.062548] ov5647: probe of 9-0036 failed with error -121
```

`-121`은 `EREMOTEIO`(I2C NACK)입니다. 부팅 11초 시점에는 센서가 응답하지 않고,
나중에 모듈을 다시 올리면 정상 인식됩니다. 센서 전원이나 PWDN 핀이 안정화되기 전에
드라이버가 I2C를 두드려서 생기는 타이밍 문제로 보입니다.

**현재 회피 방법** — 재부팅 후 한 번 실행:

```bash
sudo modprobe -r ov5647 && sudo modprobe ov5647
```

항구적 해결안은 두 가지입니다. 아직 적용하지 않았습니다.

1. systemd oneshot 유닛으로 부팅 후 자동 재로드 (증상 회피)
2. `ov5647_board_setup`에 재시도 루프를 넣거나 PWDN 해제 후 대기 시간을 늘림 (원인 수정)

### 5.2 Argus(하드웨어 ISP) 미동작

저장소 지원표상 Orin에서는 Argus가 동작하지 않습니다. `nvarguscamerasrc`를 쓸 수
없고 **raw Bayer만** 나옵니다. 참고로 `nvidia-l4t-gstreamer` 패키지도 이 시스템에
설치돼 있지 않습니다.

**정밀착륙 용도에는 거의 문제가 되지 않습니다.** ArUco 검출은 흑백만 필요하고,
디베이어는 OpenCV로 싸게 처리됩니다.

```python
gray = cv2.cvtColor(bayer8, cv2.COLOR_BayerBG2GRAY)
```

색이 녹색으로 치우치는 것도 ISP 튜닝이 없어서이며, 마커 검출에는 영향이 없습니다.

### 5.3 L4T 업데이트 시 재작업 필요

커널이 바뀌면 모듈을 다시 빌드해야 하고(`vermagic` 불일치), 업데이트가
`extlinux.conf`를 덮어쓰면 부팅 항목도 다시 넣어야 합니다.

## 6. 되돌리기

```bash
sudo rm -f /lib/modules/$(uname -r)/updates/ov5647.ko
sudo depmod -a $(uname -r)
sudo rm -f /boot/ov5647-jetson.dtbo
sudo cp -a /boot/extlinux/extlinux.conf.orig-ov5647 /boot/extlinux/extlinux.conf
sudo reboot
```

스톡 DTB(`/boot/dtb/kernel_tegra234-...-nv-super.dtb`)는 처음부터 수정하지
않았으므로 복원할 것이 없습니다.

## 7. 남은 작업 — ROS 2 연동

아직 `/camera/image_raw`를 발행하지 않습니다.

- `src/pl_bringup/launch/hardware.launch.py`는 **CSI 캡처 노드를 실행하지 않습니다**
  (파일 첫 줄: "External camera hardware profile. CSI capture and commissioning are deferred").
  외부에서 토픽을 발행해 준다고 가정합니다.
- `BG10`(10비트 Bayer, 16비트 컨테이너)은 `ros-humble-v4l2-camera`나 `usb_cam` 같은
  기성 노드가 다루지 못합니다.
- 따라서 **V4L2를 직접 읽어 디베이어 후 발행하는 노드**가 필요합니다.
  `pl_camera`에 이미 `image_relay`가 있으므로 그 옆에 추가하는 것이 자연스럽습니다.
- `src/pl_bringup/config/hardware.yaml`의 `camera.calibration_verified`와
  `camera.extrinsics_verified`가 둘 다 `true`가 아니면 `hardware.launch.py`는
  의도적으로 실행을 거부합니다. 카메라 보정과 장착 변환을 먼저 끝내야 합니다.

관련 문서: [hardware_setting.md](hardware_setting.md), [camera_inputs_stage1.md](camera_inputs_stage1.md)

## 8. 참고

- [akandr/ov5647-jetson](https://github.com/akandr/ov5647-jetson) — 사용한 드라이버 포팅
- [About sensor device registration in Jetpack-6](https://forums.developer.nvidia.com/t/about-sensor-device-registration-in-jetpack-6/327933) — JetPack 6 오버레이 관련 NVIDIA 포럼
- 저장소 루트의 `SETUP.md` — 젯슨 전체 개발 환경 세팅
