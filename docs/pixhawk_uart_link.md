# Pixhawk 6C mini ↔ Jetson UART 연결 (uXRCE-DDS)

Jetson Orin Nano와 Pixhawk 6C mini를 UART로 연결해 PX4 토픽을 ROS 2로 받기까지의 기록입니다.

- 작업일: 2026-11-13
- 결과: **성공** — `/fmu/out/vehicle_attitude` 100Hz 수신 확인
- 전제: **JetPack 6.2.2의 UART DMA 버그 때문에 디바이스 트리 수정이 필요합니다.**
  선만 꽂아서는 동작하지 않습니다.

## 0. 환경

| 항목 | 값 |
| --- | --- |
| 젯슨 | Jetson Orin Nano Developer Kit Super (P3767-0005 + P3768-0000) |
| L4T | R36.5.0 (JetPack 6.2.2), 커널 5.15.185-tegra |
| FC | Holybro Pixhawk 6C mini |
| PX4 | `SYS_AUTOSTART=4014` (쿼드) |
| 링크 | TELEM2 ↔ 40핀 헤더 UART1(`/dev/ttyTHS1`), 921600 8N1 |

## 1. 배선

**선 3가닥만 씁니다.** 양쪽 모두 3.3V TTL이라 레벨 시프터가 필요 없습니다.

```
Pixhawk TELEM2 (JST-GH 1.25mm 6핀)      Jetson 40핀 헤더 (J12)
─────────────────────────────────      ──────────────────────────
1  VCC  +5V        X 연결 금지
2  UART5_TX (out)  ───────────────────  10  UART1_RX
3  UART5_RX (in)   ───────────────────   8  UART1_TX
4  UART5_CTS       X 미연결
5  UART5_RTS       X 미연결
6  GND             ───────────────────   6  GND
```

- **1번 핀(빨간선, +5V)을 젯슨 40핀에 연결하면 안 됩니다.** 신호 라인은 3.3V입니다.
- TX↔RX를 **교차**합니다. 같은 번호끼리 직결하면 통신되지 않습니다.
- GND는 필수입니다. 전원이 분리돼 있어 공통 접지가 없으면 동작하지 않습니다.
- CTS/RTS를 쓰지 않으므로 PX4에서 흐름 제어가 꺼져 있어야 합니다(TELEM2 기본값 OFF).

젯슨 쪽 6·8·10번은 짝수 열에 **연속으로 붙어 있는 세 핀**이라 한 칸 밀리기 쉽습니다.
1번 핀은 보드 가장자리의 사각형 패드입니다.

```
홀수 열:  1   3   5   7   9  11
짝수 열:  2   4   6   8  10  12
                GND  TX  RX
```

> Pixhawk 6C mini의 TELEM 핀아웃 출처:
> [Holybro Pixhawk 6C Mini Ports](https://docs.holybro.com/autopilot/pixhawk-6c-mini/pixhawk-6c-mini-ports)

## 2. PX4 파라미터

| 파라미터 | 값 | 의미 |
| --- | --- | --- |
| `UXRCE_DDS_CFG` | **102** | TELEM2에서 uXRCE-DDS 클라이언트 기동 |
| `UXRCE_DDS_DOM_ID` | **42** | 젯슨 `ROS_DOMAIN_ID`와 일치시켜야 함 |
| `SER_TEL2_BAUD` | 921600 | `real.yaml`의 `px4.uart.baud`와 맞출 것 |

세 개 모두 **재부팅 후 적용**됩니다.

> **포트 번호 주의.** TELEM1 = 101, **TELEM2 = 102**입니다. PX4 소스
> `Tools/serial/generate_config.py`의 `serial_ports` 표가 원본입니다.
> QGroundControl에서는 숫자 대신 드롭다운에서 `TELEM 2`를 고르면 됩니다.

> **USB로는 안 됩니다.** PX4의 시리얼 포트 표에 USB 항목이 없어서
> `UXRCE_DDS_CFG`로 USB(`/dev/ttyACM0`)를 지정할 수 없습니다. USB는 QGC·파라미터
> 설정·펌웨어 용도로만 쓰고, DDS 브리지는 반드시 실제 UART를 써야 합니다.

### QGroundControl 없이 설정하기

이 젯슨에는 QGC를 설치할 수 없습니다(aarch64 AppImage가 Ubuntu 24.04 이상 요구,
이 기기는 22.04). USB가 연결돼 있으면 MAVLink로 직접 쓸 수 있습니다.

```python
# PX4는 INT32 파라미터를 float 필드에 비트패턴으로 실어 보낸다
import struct
from pymavlink import mavutil
T = mavutil.mavlink
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
m.wait_heartbeat()
for name, value in [('UXRCE_DDS_CFG', 102), ('UXRCE_DDS_DOM_ID', 42)]:
    raw = struct.unpack('<f', struct.pack('<i', value))[0]
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), raw, T.MAV_PARAM_TYPE_INT32)
m.mav.command_long_send(m.target_system, m.target_component,
                        T.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
```

PX4의 nsh 셸도 MAVLink로 열 수 있어 진단에 유용합니다
(`SERIAL_CONTROL` 메시지, `SERIAL_CONTROL_DEV_SHELL`). `uxrce_dds_client status`,
`mavlink status`, `param show <이름>` 등을 실행할 수 있습니다.

## 3. 젯슨 40핀 UART 활성화 — 반드시 **저장**할 것

40핀 헤더의 8/10번은 기본 상태에서 UART로 먹싱돼 있지 **않습니다.**

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

**Configure Jetson 40pin Header** → **Configure header pins manually** →
`uarta (8,10)` 확인 → **Save pin changes** → 재부팅.

> **함정.** 메뉴에서 `uarta`에 `[*]`가 보이는 것은 "이렇게 설정하겠다"는 **제안**이지
> 적용된 상태가 아닙니다. **Save를 눌러야** 오버레이가 생성되고 `extlinux.conf`에
> 등록됩니다. 저장하지 않고 나오면 아무것도 바뀌지 않습니다.

CSI 카메라와 **동시에 쓸 수 있습니다.** jetson-io는 40핀 헤더와 CSI 커넥터를 별개
헤더로 관리하고, 선택된 오버레이를 `OVERLAYS` 한 줄에 쉼표로 모아 씁니다.

```
OVERLAYS /boot/jetson-io-hdr40-user-custom.dtbo,/boot/tegra234-p3767-camera-p3768-imx219-A.dtbo
```

적용 확인:

```bash
ls /dev/ttyTHS1                       # 3100000.serial = UART1
cat /sys/class/tty/ttyTHS1/device/uevent | head -2
```

## 4. UART DMA 버그 — 이게 진짜 원인이었습니다

핀먹싱을 켜도 데이터가 깨집니다. **JetPack 6.2.2 / L4T R36.5의 알려진 버그**입니다.

### 증상

`/dev/ttyTHS1`로 `0xFF`를 개수를 바꿔가며 루프백(8번↔10번 점퍼)한 결과:

```
  1 바이트 -> 선행0   0, 정상   1     OK
  4 바이트 -> 선행0   0, 정상   4     OK
  8 바이트 -> 선행0   8, 정상   0     깨짐
 12 바이트 -> 선행0   8, 정상   4
 20 바이트 -> 선행0  16, 정상   4
128 바이트 -> 선행0 128, 정상   0     깨짐
```

**정확히 8의 배수 단위로 0x00이 되고, 나머지만 살아남습니다.** 보드레이트와 무관하며
(9600~3000000 전부 동일), 프레이밍 에러는 0입니다.

`serial-tegra` 드라이버는 8바이트 미만은 PIO로, 그 이상은 DMA로 처리합니다.
숫자가 8의 배수로 떨어지는 것이 DMA 경로임을 가리킵니다.

### 참고

- [DMA on /dev/ttyTHS1 corrupts receiving data — NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/dma-on-dev-ttyths1-corrupts-receiving-data/369191)
- [Fixing UART DMA Issues on NVIDIA Jetpack 6.2.2](https://docs.aerium.co.il/Tutorials/NVIDIA%20Jetson/fixing-uart-dma-jetpack)

### 해결 — DTB에서 `dmas` 제거

`dmas`/`dma-names`를 지우면 드라이버가 PIO로 되돌아갑니다.

> **오버레이로는 지울 수 없습니다.** `delete_prop = "dmas", "dma-names";`를 쓴
> 오버레이를 `OVERLAYS`로 적용하면, L4T UEFI 적용기가 이것을 **명령으로 해석하지 않고
> `delete_prop`이라는 속성으로 그대로 복사**합니다. `dmas`는 그대로 남습니다.
> 실제로 시도했고 실패했습니다.

베이스 DTB를 직접 고쳐 별도 파일로 저장하고, `FDT`가 그 파일을 가리키게 합니다.

```bash
BASE=/boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb
OUT=/boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-uartpio.dtb

dtc -I dtb -O dts -o base.dts "$BASE"
# serial@3100000 노드에서 dmas, dma-names 두 줄을 삭제
dtc -I dts -O dtb -o uartpio.dtb base.dts       # -@ 는 쓰지 않는다 (아래 참고)
sudo install -m 0644 uartpio.dtb "$OUT"
```

`extlinux.conf`의 `DEFAULT` 항목에서 `FDT`만 바꿉니다. **`OVERLAYS`는 그대로 둡니다** —
핀먹싱과 카메라 오버레이가 이 DTB 위에 그대로 올라갑니다.

```
LABEL JetsonIO
	FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-uartpio.dtb
	OVERLAYS /boot/jetson-io-hdr40-user-custom.dtbo,/boot/tegra234-p3767-camera-p3768-imx219-A.dtbo
```

> **`dtc -@`를 쓰지 마세요.** 디컴파일된 DTS에는 `__symbols__`가 **일반 노드**로 들어
> 있습니다. `-@`는 라벨에서 심볼을 새로 생성하려 드는데 디컴파일 결과에는 라벨이 없어
> 충돌합니다. 옵션 없이 컴파일하면 `__symbols__` 노드가 그대로 보존됩니다.
> **이게 보존되지 않으면 카메라·핀먹싱 오버레이가 적용되지 않습니다.**

재컴파일 후 반드시 확인:

```bash
dtc -I dtb -O dts uartpio.dtb | grep -c __symbols__      # 1 이어야 함
dtc -I dtb -O dts uartpio.dtb | grep -c cam_i2c          # 0 이 아니어야 함
```

적용 확인:

```bash
ls /proc/device-tree/bus@0/serial@3100000/
# clocks compatible interrupts name phandle reg reset-names resets status
#   -> dmas, dma-names 가 없어야 한다
```

## 5. 검증

### 5.1 DMA 수정 전후

같은 조건(픽스호크가 TELEM2로 MAVLink 송신, 젯슨이 8초간 수신):

| | 수정 전 | 수정 후 |
| --- | --- | --- |
| 수신 바이트 | 14,122 | 14,178 |
| 0이 아닌 바이트 | 881 (6.2%) | 12,128 (**85.5%**) |
| 파싱된 MAVLink | 거의 없음 | **20종** |

```
앞 48바이트: fe 1c b9 01 01 1e 99 24 40 00 ...   <- 정상 MAVLink v1 프레임

ATTITUDE 122 · ATTITUDE_QUATERNION 81 · GPS_RAW_INT 41
VFR_HUD 32 · ATTITUDE_TARGET 16 · HEARTBEAT 8 · ...
```

### 5.2 uXRCE-DDS 링크

```bash
cd ~/INAIR/precision_landing
source scripts/env.sh
export ROS_DOMAIN_ID=42
MicroXRCEAgent serial -D /dev/ttyTHS1 -b 921600
```

다른 터미널에서:

```bash
source scripts/env.sh && export ROS_DOMAIN_ID=42
ros2 topic list | grep /fmu/
ros2 topic hz /fmu/out/vehicle_attitude
```

결과:

```
/fmu/out/vehicle_attitude       average rate: 100.046 Hz
/fmu/out/timesync_status        average rate: 0.992 Hz
/fmu/out/vehicle_local_position
/fmu/out/vehicle_status_v1
/fmu/out/sensor_combined
... (/fmu/in/* 송신 토픽도 전부)

q: [-0.37101441, 0.01138760, 0.01187376, 0.92848140]
```

`real.launch.py`는 이 Agent를 자동으로 띄우므로 평소에는 직접 실행할 필요가 없습니다.

## 6. 문제 해결 순서

문제가 겹쳐 있을 때 변수를 하나씩 줄이는 순서입니다. 실제로 이 순서로 풀었습니다.

### 6.1 선이 연결돼 있는가 — GPIO 루프백

8번과 10번을 점퍼 한 가닥으로 잇고, UART가 아니라 **GPIO로** 확인합니다.
UART 설정과 무관하게 선과 패드만 시험합니다.

```bash
for v in 0 1; do
  gpioset --mode=signal gpiochip0 110=$v & sleep 0.3
  echo "8번=$v -> 10번=$(gpioget gpiochip0 111)"
  kill %1
done
# 110 = PR.02 (8번), 111 = PR.03 (10번)
```

`0 -> 0`, `1 -> 1`이면 선과 패드는 정상입니다.

### 6.2 UART로 동작하는가 — 시리얼 루프백

점퍼를 그대로 둔 채 바이트를 보내 되돌아오는지 봅니다.
**한 바이트씩**과 **여러 바이트 한 번에**를 모두 시험해야 DMA 버그가 드러납니다.

### 6.3 라인에 신호가 들어오는가 — UART 카운터

핀 모드를 건드리지 않고 하드웨어 카운터를 읽습니다. `gpioget`은 패드를 GPIO로
전환시켜 결과를 왜곡할 수 있으므로, UART가 살아 있는 상태에서는 이쪽이 정확합니다.

```python
import fcntl, struct, serial
TIOCGICOUNT = 0x545D   # struct serial_icounter_struct
s = serial.Serial('/dev/ttyTHS1', 921600)
v = struct.unpack('20i', fcntl.ioctl(s.fileno(), TIOCGICOUNT, bytes(80)))
# 0 cts, 1 dsr, 2 rng, 3 dcd, 4 rx, 5 tx, 6 frame, 7 overrun, 8 parity, 9 break
rx, frame, overrun, parity, brk = v[4], v[6], v[7], v[8], v[9]
```

- `rx`가 증가하면 신호가 들어오고 있습니다.
- `frame`/`parity`가 증가하면 **보드레이트가 틀렸습니다.**
- `rx`는 늘지만 내용이 0x00이면 **DMA 버그**입니다.

### 6.4 FC가 실제로 보내는가

```
nsh> mavlink status          # ttyS3 인스턴스의 tx 레이트 확인
nsh> uxrce_dds_client status # "Running, disconnected" 면 클라이언트는 떠 있음
```

송신 테스트가 필요하면 DDS 클라이언트를 멈추고 MAVLink를 띄웁니다
(파라미터에 저장되지 않아 재부팅하면 원복됩니다).

```
nsh> uxrce_dds_client stop
nsh> mavlink start -d /dev/ttyS3 -b 921600
```

### 6.5 증상별 원인

| 증상 | 원인 |
| --- | --- |
| `rx` 카운터가 0 | 선 미연결, 포트 착각, 핀 번호 밀림 |
| 수신은 되는데 `frame` 에러 다발 | 보드레이트 불일치 |
| 수신되는데 대부분 0x00 | **DMA 버그** (4절) |
| 루프백에서 1~4바이트만 정상 | **DMA 버그** |
| 유휴 시 라인이 LOW | GND가 신호 핀에 꽂힘 (한 칸 밀림) |
| 토픽이 안 보임 | `UXRCE_DDS_DOM_ID` ≠ `ROS_DOMAIN_ID` |

## 7. 되돌리기

```bash
sudo cp -a /boot/extlinux/extlinux.conf.before-uartpio-dtb /boot/extlinux/extlinux.conf
sudo reboot
```

스톡 DTB(`...-nv-super.dtb`)는 수정하지 않았으므로 복원할 것이 없습니다.
부팅이 되지 않으면 30초 메뉴에서 `primary kernel`을 고르면 됩니다.

## 8. 아직 검증되지 않은 것

- **3,000,000 baud** — 프로젝트 기본값이었으나 점퍼선 환경에서 검증하지 않았습니다.
  현재 `real.yaml`은 921600입니다. 올리려면 FC의 `SER_TEL2_BAUD`와 함께 바꾸세요.
- **PIO 모드의 처리량** — DMA를 껐으므로 CPU 부하가 늘어납니다. 100Hz 자세 수신은
  문제없이 동작하지만, 고주파 토픽을 추가로 켤 때 여유를 확인해야 합니다.
- **실제 비행 중 진동·전자기 간섭** — 점퍼선이 아닌 고정 케이블로 바꿔야 합니다.
- **L4T 업데이트 후** — 커널이나 DTB가 갱신되면 4절을 다시 적용해야 합니다.

관련 문서: [csi_camera_ov5647.md](csi_camera_ov5647.md) · [hardware_setting.md](hardware_setting.md)
