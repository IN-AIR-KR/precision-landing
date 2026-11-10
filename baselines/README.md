# 단계 0 기준 데이터

버전 잠금 정보는 `versions.json`, 수행 범위와 재현 방법은
[`docs/baseline_stage0.md`](../docs/baseline_stage0.md)를 참조하세요.

각 `YYYYMMDD-HHMMSS-{static,moving}/` 디렉터리는 다음을 보존합니다.

- `bag/`: 원본 카메라 Image/CameraInfo, target_pose, target_pose_world, PX4 자세/위치/상태/착륙,
  trajectory_setpoint, vehicle_command, rosout, clock. 기록은 best-effort/volatile QoS로 수행.
- `manifest.json`, `source/`, `project.patch`: 커밋, dirty 상태, 당시 프로젝트 소스 사본과 SHA256.
- `aruco_tracker.yaml`, `precision_land.yaml`: 실행 당시 노드 파라미터.
- `logs/`, `flight.ulg`, `flight.jsonl`, `final-gazebo-poses.txt`: 실행 로그와 비행/착륙 증거.
- `verification.json`: 토픽별 메시지 수, 영상과 보정 크기 일치, FSM 완료, 자동 disarm,
  Gazebo 실제 좌표의 착륙 오차 검증.
- `replay-verification.json`: 영상만 격리 재생한 경우 원본 촬영 시각으로 매칭한 인식 결과 비교.

대용량 데이터는 Git에서 제외합니다. 소스와 버전 목록만 저장소에 포함됩니다.
`COLCON_IGNORE`는 소스 스냅샷을 중복 ROS 패키지로 인식하지 않게 합니다.
기준 데이터는 자동 삭제하지 않습니다. 전체 복사/보관 단위는 각 실행 디렉터리입니다.

원시 Image는 1280×960 RGB 3채널이므로 한 비행에 약 3GB가 필요합니다.
다시 기록하려면 기존 SITL을 종료하고 `scripts/record_baseline.py`를 실행하세요.

## 1번 이후 입력 분리

새 기록은 `/camera/image_raw`, `/camera/camera_info`를 사용합니다. 재생 도구는
`topics.json`을 읽어 기존 Gazebo 토픽 bag과 새 bag을 모두 지원합니다. 새 재생 보고서는
`.run/replays/`에 저장하여 기준 기록을 덮어쓰지 않습니다. 1번 검증 요약은
`stage1-results.json`과 [1번 안내](../docs/camera_inputs_stage1.md)에 있습니다.
