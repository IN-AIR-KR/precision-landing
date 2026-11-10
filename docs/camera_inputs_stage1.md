# 1번: 공통 카메라 입력 분리 (CSI 보류)

> 아래 검증 결과는 당시 기록입니다. 원본 기준 데이터와 전용 기록·재생 도구는 정리했으며, 수치만 이 문서에 남깁니다.

현재 인식기는 Python으로 전환되었습니다. 아래 내용은 해당 단계 당시의 기록이며, 최신 상태는 [2번 전환 기록](python_detection_stage2.md)을 참고하세요.

## 적용한 범위

- C++ 인식기는 `/camera/image_raw`와 `/camera/camera_info`만 구독합니다.
  `world_name` 파라미터와 Gazebo 월드·모델 경로를 제거했습니다.
- 시뮬 launch가 Gazebo 센서 토픽을 공통 토픽으로 연결합니다. 외부 카메라는
  `hardware.yaml`의 `camera.image_topic`, `camera.camera_info_topic`을 remap합니다.
  인식기 바이너리는 동일합니다.
- 기존 `precision_landing_system.launch.py` 경로도 공통 입력에 맞춰 유지했습니다.
- `/target_pose`의 위치·자세 계산과 best-effort QoS는 유지합니다. timestamp는 입력 이미지의
  촬영 시각을 그대로 전달하며, 수신 시각으로 바꾸지 않습니다.
- 카메라 장착 위치·방향을 환경별 control 파라미터로 분리했습니다. 기본값은 기존 코드를 재현합니다.
## 설정과 좌표계

설정 파일은 `src/pl_bringup/config/sim.yaml`과 `hardware.yaml`입니다.

| 설정 | 의미 |
|---|---|
| `camera.image_topic`, `camera.camera_info_topic` | 입력 Image/CameraInfo 토픽 |
| `camera.optical_frame` | `/target_pose` 프레임 이름. x 오른쪽, y 아래, z 렌즈 전방인 optical 좌표계 |
| `detection.marker_size` | 실제 마커 한 변 길이(m) |
| `control.camera_position_body` | 기체 body FRD 기준 카메라 원점 `[x,y,z]`(m) |
| `control.camera_orientation_body` | optical 벡터를 body FRD로 회전하는 쿼터니언 `[w,x,y,z]` |

시뮬 기본 위치는 `[0,0,-0.1]`, 방향은 `[sqrt(0.5),0,0,sqrt(0.5)]`입니다.
이는 0번에서 검증한 기존 변환을 보존한 값이며 새 장착 측정값이 아닙니다.
변환 순서는 `T_world_body * T_body_optical * T_optical_tag`입니다.
쿼터니언은 정규화하며, 길이가 잘못되거나 NaN/Inf/영벡터이면 시작 시 오류로 거부합니다.

기존 시뮬의 `/target_pose` 프레임 이름 `camera_frame`은 호환성을 위해 유지했습니다.
이 이름의 좌표축은 위 optical 정의입니다. 프레임 이름 변경은 좌표를 회전시키지 않습니다.
인식기를 직접 실행할 때 `optical_frame`을 빈 문자열로 설정하면 입력 이미지의 frame_id를 사용합니다.
`/image_proc`는 원본 이미지 헤더를 유지합니다.

## 보류한 범위

사용자 요청으로 CSI 드라이버/캡처 노드, Jetson 센서 모드, 실제 CameraInfo 보정값,
촬영 시각 출처·ROS 시간 변환, 실기체 방향·거리 검증은 보류했습니다.
현재 hardware 프로필은 외부 Image/CameraInfo 공급자를 전제로 하며 CSI를 실행하지 않습니다.
`calibration_verified`와 `extrinsics_verified`는 모두 false입니다. 실제 검증 전에는
이 값을 바꾸지 않으며, 실기체 서비스도 이번 작업에서 변경하지 않았습니다.
따라서 계획 1번은 **소프트웨어 입력 분리 완료 / CSI 완료 조건 보류** 상태입니다.

인식기는 최초 유효 CameraInfo를 유지합니다. 해상도나 보정이 달라지면 노드를 재시작해야 합니다.
촬영 시각을 이용한 기체 상태 보간과 Delayed KF는 5번 항목입니다.

## 직접 테스트

기존 실행 방법은 같습니다.

```bash
cd ~/precision-landing
./scripts/run.sh
```

다른 터미널에서 자동 이륙·타겟 이동·착륙:

```bash
cd ~/precision-landing
source scripts/env.sh
python3 scripts/test_flight.py
```

정적 월드로 보려면 `SIM_WORLD=aruco ./scripts/run.sh`를 사용하고,
시험 명령은 `python3 scripts/test_flight.py --speed 0`입니다.

카메라 장착 변환 단위 시험:

```bash
colcon build --packages-select pl_control --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select pl_control --ctest-args -R '^camera_extrinsics_test$' --output-on-failure
colcon test-result --test-result-base build/precision_land/test_results --verbose
```

## 검증 결과 (2026-11-10)

- 변경 패키지 빌드 완료. 장착 변환 단위 시험 3개 통과.
- 정적 SITL: `20261110-201110-static`, 착륙·자동 disarm 성공, 중심 오차 약 0.96cm.
- 이동 SITL(0.3m/s): `20261110-201221-moving`, 착륙·자동 disarm 성공, 중심 오차 약 4.29cm.
- 실행 중 ROS 카메라 토픽이 `/camera/image_raw`, `/camera/camera_info`로 제공되는 것을 확인.
- 0번 이동 bag: 비교 가능한 192/192 포즈 일치, 최대 성분 차이 0.
- 0번 정적 bag을 `/test/external_camera/*`로 remap: 210/210 포즈 일치, 최대 성분 차이 0.
  두 시험의 인식기 실행 파일 SHA256이 동일함을 확인.
- 새 공통 토픽 정적 bag도 재생 성공: 비교 가능한 202/202 포즈 일치, 최대 성분 차이 0.
  원본 영상이 함께 기록되지 않은 포즈는 기존 bag 각각 1개, 새 정적 bag 4개이며 별도 집계.
- 매칭은 원본 촬영 timestamp 기준으로 수행했고 출력 프레임 `camera_frame`도 확인.
- hardware 기본 설정은 보정/장착 검증 미완료로 실행 거부. Gazebo 경로·프로세스는 포함하지 않음.

위 비행 오차는 각 실행의 결과이며
제어 알고리즘 개선 효과를 의미하지 않습니다. 기존 KF·FSM·게인은 바꾸지 않았습니다.

## 패키지 구조 보완

- `pl_camera/launch/camera_sim.launch.py`: Gazebo 카메라 브리지. pl_bringup에서 include. CSI 드라이버는 보류.
- `pl_detection`: 기존 C++ 인식기를 이동. Python 전환은 2번.
- `pl_control`: 기존 PX4 모드, 착륙 FSM, KF를 이동. Delayed KF는 5번.
- `pl_viz`: 기존 Gazebo 표시를 이동. 공통 모니터링은 4번.
- `pl_bringup`: 프로필과 프로세스 연결.

노드명과 실행 파일명은 기준 bag 및 파라미터 호환성을 위해 유지합니다.

패키지 이동 후 재검증:

- 정지 비행 `20261110-203625-static`: 착륙·자동 disarm 통과, 수평 오차 0.019715 m.
- 이동 비행 `20261110-203718-moving`: 0.3 m/s, 착륙·자동 disarm 통과, 수평 오차 0.028204 m.
- 기존 이동 bag 재생: 비교 가능한 192/192 포즈 일치, 최대 성분 차이 0. 원본 이미지가 없는 1개 포즈는 비교 제외.
- 카메라 장착 변환 단위 테스트 3개 통과.
