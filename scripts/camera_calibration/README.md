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

- **Space**로 현재 프레임에서 체커보드를 검출하고 성공한 경우 샘플을 추가합니다. 검출 결과는 화면과 터미널에 표시됩니다. 화면 중앙, 가장자리,
  여러 거리와 기울기에서 15~25장 정도 모읍니다.
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

`real.yaml`은 센서 모드 4와 1280×720 @ 60 fps를 기본으로 사용합니다.
생성한 `camera.calibration`을 붙여 넣은 뒤 실기체 런치를 실행합니다.
`output_width: 640`과 `output_height: 360`은 그대로 사용해 검출기 입력 크기를 유지합니다.

보정 도구를 종료한 뒤 실기체 설정으로 카메라, ArUco 검출, HUD를 확인합니다.

```bash
source scripts/env.sh
ros2 launch pl_bringup camera_test.launch.py
```

HUD 창 없이 ROS 노드만 실행하려면 `view:=false`를 지정합니다.
기본 ROS 도메인은 0입니다. 다른 터미널에서 `ros2 topic list`로 테스트 토픽을 확인할 수 있습니다.
