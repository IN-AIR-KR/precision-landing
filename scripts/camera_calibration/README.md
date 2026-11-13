# CSI 카메라 내부 파라미터 보정

Jetson GUI 터미널에서 GStreamer 및 GUI를 지원하는 시스템 OpenCV로 실행합니다.
스크립트가 Argus로 CSI 카메라를 직접 엽니다. 센서를 사용 중인 프로세스가 있으면 먼저 종료합니다.

아래 예시는 사각형이 10×7개이고 한 칸의 실측 길이가 25 mm인 보드입니다.

```bash
env -u PYTHONPATH /usr/bin/python3 scripts/camera_calibration/calibrate.py \
  --cols 9 --rows 6 --square-mm 25 --output csi_1920x1080.yaml
```

기본값은 센서 0, 1920×1080 @ 30 FPS, flip 0입니다.
`--sensor-id`, `--width`, `--height`, `--fps`, `--flip-method`를 비행용 카메라와 동일하게 설정합니다.

- **Space**로 코너가 표시된 현재 프레임을 추가합니다. 화면 중앙, 가장자리,
  여러 거리와 기울기에서 15~25장 정도 모읍니다.
- **U**로 마지막 샘플을 취소합니다.
- **C**로 최소 10장에서 보정값을 계산하고 YAML에 저장합니다.
  터미널에 재투영 RMS 오차(px)가 표시됩니다.
- **Q** 또는 **Esc**로 종료합니다.

카메라 실행 시 저장한 YAML의 절대 경로를 `camera_info_url:=...`로 전달합니다.
이 도구가 구하는 값은 렌즈 내부 파라미터와 왜곡 계수입니다. 기체 기준 카메라
장착 변환은 별도로 측정합니다.
