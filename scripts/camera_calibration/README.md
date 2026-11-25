# CSI 카메라 내부 파라미터 보정

Jetson GUI 터미널에서 GStreamer 및 GUI를 지원하는 시스템 OpenCV로 실행합니다.
스크립트가 Argus로 CSI 카메라를 직접 엽니다. 센서를 사용 중인 프로세스가 있으면 먼저 종료합니다.

아래 예시는 사각형이 10×7개이고 한 칸의 실측 길이가 25 mm인 보드입니다.

```bash
env -u PYTHONPATH /usr/bin/python3 scripts/camera_calibration/calibrate.py \
  --cols 9 --rows 6 --square-mm 25
```

기본값은 센서 0, 1920×1080 @ 30 FPS, flip 0입니다.
IMX219는 `--mode`로 아래 센서 모드를 선택할 수 있습니다.
`--sensor-id`와 `--flip-method`도 비행용 카메라와 동일하게 설정합니다.

| `--mode` | 센서 해상도 | FPS |
| --- | --- | ---: |
| `0` | 3280×2464 | 21 |
| `1` | 3280×1848 | 28 |
| `2` | 1920×1080 | 30 |
| `3` | 1640×1232 | 30 |
| `4` | 1280×720 | 60 |

해상도와 FPS를 직접 지정하려면 `--width`, `--height`, `--fps`를 사용합니다.

## 체커보드 샘플 촬영

실제 비행에 사용할 센서 모드에서 선명한 사진을 20장 정도 모읍니다.
예를 들어 센서가 1280×720 @ 60 FPS로 촬영하고 ROS 출력이 640×360이면,
캘리브레이션은 센서 모드인 1280×720 @ 60 FPS에서 진행합니다.

| 장수 | 구도 |
| ---: | --- |
| 4장 | 화면 중앙에서 보드를 정면에 가깝게 두고 가까이에서 2장, 조금 멀리에서 2장 찍습니다. |
| 8장 | 카메라를 고정하고 보드 전체를 화면 위·아래·왼쪽·오른쪽과 네 모서리로 옮겨 한 장씩 찍습니다. |
| 8장 | 보드의 왼쪽 변, 오른쪽 변, 위쪽 변, 아래쪽 변을 각각 카메라 쪽으로 기울여 두 장씩 찍습니다. 두 장 사이에 거리나 화면 위치도 바꿉니다. |

가장자리 샘플은 미리보기에서 아래 화살표 위치에 **보드의 중심**을 놓는다고 생각하면 됩니다.
예를 들어 왼쪽 위 샘플은 보드 전체가 미리보기 왼쪽 위에 들어오도록 옮기고,
보드가 잘리지 않게 약간의 여백을 남긴 뒤 `Space`를 누릅니다.
이때 보드는 정면에 가깝게 둬도 됩니다.

```text
↖   ↑   ↗
←   ·   →
↙   ↓   ↘
```

보드를 평평한 판에 붙이고, 빛이 반사되지 않는 밝은 곳에서 촬영합니다.
한 장씩 보드를 멈춘 뒤 찍고, 가까운 컷에서는 보드가 화면을 넉넉히 채우도록 합니다.
먼 컷에서도 체커보드 코너가 선명하게 보이는 거리까지만 물러납니다.

## 촬영 및 저장 키

- **Space**로 현재 프레임에서 체커보드를 검출하고 성공한 경우 샘플을 추가합니다. 검출 결과는 화면과 터미널에 표시됩니다.
- 검출에 성공한 사진은 즉시 `scripts/camera_calibration/samples/` 아래 PNG로 저장됩니다. 실행 시 터미널에 실제 경로가 표시됩니다.
- **U**로 마지막 샘플을 계산 목록에서 취소합니다. 저장된 PNG는 유지됩니다.
- **C**로 최소 10장에서 보정값을 계산하고 YAML에 저장합니다.
  스크립트 옆 `calibration.yaml`에 저장하며 기존 결과는 새 보정에 성공하면 교체합니다.
  저장 직후 미리보기에 보정을 적용하고 터미널에 재투영 RMS 오차(px)를 표시합니다.
- 창의 **Undistort**를 0 또는 1로 바꿔 원본과 보정 영상을 비교합니다. 샘플은 원본 프레임으로 저장합니다.
- **Q** 또는 **Esc**로 종료합니다.

생성한 YAML의 `camera.calibration`을 `src/pl_bringup/config/real.yaml`의 같은 위치에 붙여 넣습니다.
이 도구가 구하는 값은 렌즈 내부 파라미터와 왜곡 계수입니다. 기체 기준 카메라
장착 변환은 별도로 측정합니다.

## IMX219 60 FPS

Jetson GUI에서 비행 스택을 종료하고 미리보기로 움직임과 실제 FPS를 확인합니다.

```bash
env -u PYTHONPATH /usr/bin/python3 scripts/camera_preview/imx219_preview.py --backend argus --mode 4 --no-aruco
```

같은 센서 모드로 보정합니다. 체커보드에 맞춰 내부 교차점 수와 칸 길이를 지정합니다.

```bash
env -u PYTHONPATH /usr/bin/python3 scripts/camera_calibration/calibrate.py \
  --cols 9 --rows 6 --square-mm 25 --mode 4
```

IMX219를 사용할 때는 센서 모드 4와 1280×720 @ 60 fps를 지정합니다.
생성한 `camera.calibration`을 붙여 넣은 뒤 실기체 런치를 실행합니다.
`output_width: 640`과 `output_height: 360`은 그대로 사용해 검출기 입력 크기를 유지합니다.

보정 도구를 종료한 뒤 실기체 설정으로 카메라, ArUco 검출, HUD를 확인합니다.

```bash
source scripts/env.sh
ros2 launch pl_bringup camera_test.launch.py
```

HUD 창 없이 ROS 노드만 실행하려면 `view:=false`를 지정합니다.
기본 ROS 도메인은 0입니다. 다른 터미널에서 `ros2 topic list`로 테스트 토픽을 확인할 수 있습니다.

## IMX708 Argus 보정

현재 실기체 기본 설정은 IMX708 모드 1, 1536×864 @ 60 fps 캡처와
640×360 ROS 출력입니다. 보정 도구와 실기체 카메라 모두 Argus의 자동 노출,
자동 게인, 화이트밸런스를 사용합니다.

```bash
env -u PYTHONPATH /usr/bin/python3 scripts/camera_calibration/calibrate.py \
  --imx708 --imx708-mode 1 --cols 9 --rows 6 --square-mm 25
```

결과는 `imx708_1536x864_calibration.yaml`에 저장됩니다.
이 파일의 `camera.calibration`을 `real.yaml`에 적용합니다.
