# 2번: Python 인식기와 바이너리 의존성

> 아래 검증 결과는 당시 기록입니다. 원본 기준 데이터와 전용 기록·재생 도구는 정리했으며, 수치만 이 문서에 남깁니다.

`pl_detection`은 ament_python 패키지로 전환했습니다. 실행 파일명 `aruco_tracker`,
노드명, 토픽, 파라미터를 유지하므로 `run.sh`, `test_flight.py` 사용법은 같습니다.

## 계산과 메시지 호환성

- ArUco 기본 검출 파라미터, 딕셔너리·ID·마커 크기 유지.
- float32 마커 꼭짓점, undistortPoints, 반복형 solvePnP, OpenCV quaternion 부호 규칙 유지.
- 입력 이미지의 촬영 timestamp와 optical frame을 `/target_pose`에 전달.
- 입력·출력은 best-effort, volatile, depth 1. 첫 유효 CameraInfo를 보관하는 기존 동작 유지.
- `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8` 지원. `Image.step`의 행 패딩을 건너뛰고
  독립적인 BGR 버퍼로 변환. 8-bit 채널이므로 endianness에 영향 없음.
- 지원하지 않는 인코딩, 잘못된 크기·stride·버퍼 길이는 오류로 거부.
- 디버그 영상은 기존처럼 매 프레임 발행. 구독자에 따른 출력 제어는 4번 범위.

## 설치·빌드

```bash
cd ~/precision-landing
./scripts/setup.sh sim
source scripts/env.sh
```

`requirements-detection.txt`에 NumPy 1.26.4와 opencv-contrib-python-headless 4.10.0.84를
고정합니다. pip `--only-binary=:all:`로 `.deps/python`에 설치하며, 지원 wheel이 없으면
소스 빌드로 우회하지 않고 실패합니다. 시스템 Python/ROS 패키지는 덮어쓰지 않습니다.
`env.sh`가 이 프로젝트의 Python 경로를 설정합니다.

선택한 버전은 기존 C++ OpenCV 4.10.0과 비교하기 위한 것입니다.
[PyPI 배포 페이지](https://pypi.org/project/opencv-contrib-python-headless/4.10.0.84/)에
x86_64·aarch64 wheel이 있지만, Jetson 실제 구동 검증을 의미하지 않습니다.

인식기의 cv_bridge·px4_msgs·Eigen 의존성과 제어기의 미사용 tf2·tf2_ros를 제거합니다.
vision_opencv 서브모듈과 OpenCV 소스 설치 스크립트도 제거합니다.
기존 시스템의 `/usr/local` 라이브러리는 변경하지 않으며 새 환경에서는 필요하지 않습니다.

## 검증

```bash
source scripts/env.sh
python3 -m pytest -q src/pl_detection/test
```

Jetson 처리율·촬영부터 포즈 발행까지의 지연 분포와 CSI 실제 입력 검증은 보류입니다.
PC SITL 및 bag 결과만으로 실기체 단계 완료를 주장하지 않습니다.

## 결과 (2026-11-10)

- 기존 build/install과 OpenCV 소스 설치를 `.run/stage2-reference/`로 이동한 뒤 9개 패키지 새 빌드 통과.
- `colcon test --packages-select pl_detection`: 19개 테스트 통과.
- 정지 bag 851프레임: C++/Python 검출 timestamp 215/215 일치, 최대 포즈 성분 차이 2.12e-13.
- 이동 bag 1,004프레임: C++/Python 검출 timestamp 193/193 일치, 최대 포즈 성분 차이 2.30e-10.
- 기준 bag에 기록된 포즈와의 비교는 정지 210/210, 이동 192/192 일치. 각 bag에서 입력 이미지가 없는 1개 포즈는 비교 제외.
- 정지 SITL `20261110-205725-static`: 착륙·자동 disarm 통과, 수평 오차 0.011912 m.
- 이동 SITL `20261110-205814-moving`: 0.3 m/s, 착륙·자동 disarm 통과, 수평 오차 0.074119 m.

수평 오차는 이번 개별 비행의 결과이며, 알고리즘 성능 향상을 뜻하지 않습니다.
다른 PX4 실행으로 시작 전에 중단된 `205535-static`, `205537-moving` 기록은 비행 검증 결과에서 제외했습니다.

## 파일 및 설정 정리

인식기 구현은 `src/pl_detection/pl_detection/aruco_tracker.py` 하나에 통합했습니다.
`__init__.py`는 Python 패키지 표시용이며 비어 있습니다.
현재 sim/real 실행은 `pl_bringup/config/{sim,real}.yaml`을 읽습니다.
운용 설정은 `pl_bringup/config/sim.yaml`과 `real.yaml`에서 관리합니다. 기존 시뮬 launch도 bringup의 시뮬 설정을 사용합니다.
읽는 코드가 없는 `camera.source`, `agent.transport`, real의 `viz`는 삭제했습니다.

통합 후 재빌드·테스트 19개 통과. 이동 bag 1,004프레임 재생에서도 검출 193/193과 포즈 비교가 통과했습니다. 결과는 `.run/stage2-single-file-moving/`에 보존합니다.
