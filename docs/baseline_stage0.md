# 리팩터링 0번: 기준 동작과 실행 경계

현재 인식기는 Python으로 전환되었습니다. 아래 내용은 해당 단계 당시의 기록이며, 최신 상태는 [2번 전환 기록](python_detection_stage2.md)을 참고하세요.

이 문서는 0번 시점의 기준 기록입니다. 이후 입력 분리 변경은 [1번 안내](camera_inputs_stage1.md)에 기록합니다.

## 적용 범위

현재 C++ 인식기, 좌표 변환, KF, 착륙 FSM과 `PrecisionLandCustom` 모드를 그대로 유지했습니다.
저장소 루트는 colcon 워크스페이스이며 패키지는 `src/`, 모델과 월드는 `simulation/`에 있습니다.
최초 0번 구현에서 패키지 분리를 누락했으며, 사용자 지적에 따라 아래 구조로 보완했습니다.
아래 수치는 당시 검증 결과이며, 원본 데이터와 전용 기록·재생 도구는 정리했습니다.

| 패키지 | 현재 구현 |
|---|---|
| pl_camera | camera_sim.launch.py에서 Gazebo 브리지 소유. CSI는 보류 |
| pl_detection | 기존 C++ ArUco 구현 이동. Python 전환은 2번 |
| pl_control | 기존 PX4 모드·KF·착륙 FSM 이동. Delayed KF는 5번 |
| pl_bringup | sim/real launch, YAML 설정, Agent 실행 |
| pl_viz | 기존 Gazebo 마커 표시 이동. 공통 모니터는 4번 |

하드웨어 설정에는 UART 장치/baud, 외부 Image/CameraInfo 토픽과 보정 확인 플래그를 둡니다.
CSI 캡처·보정값·카메라 장착 변환의 실제 구현/검증은 1번입니다. 기본 real 프로필은
`calibration_verified: false`로 명확히 실패하며 실기체 검증 완료를 의미하지 않습니다.
기존 Jetson 서비스는 수정/중지하지 않았습니다. 실기체 launch 검증 후 정리합니다.

## 설치·빌드·실행 대상

| 대상 | 설치/빌드 | 실행 | 검증 범위 |
|---|---|---|---|
| 시뮬 | `./scripts/setup.sh sim`, `./scripts/build.sh sim` | `./scripts/run.sh` | 정적·이동 SITL 비행 |
| 실기체 | `./scripts/setup.sh real`, `./scripts/build.sh real` | `./scripts/run_real.sh` | 현재 PC에서 미보정 설정 거부 확인. 실기체 비행 미검증 |

setup은 기존 시스템 ROS Humble, Gazebo(시뮬), MicroXRCEAgent 및 C++ 빌드 도구 설치를 전제로 합니다.
real 빌드 대상은 `pl_bringup`, `pl_control`, `pl_detection` 및 그 의존성입니다.
Gazebo 브리지/시각화와 PX4 SITL은 제외합니다. 하드웨어 프로필의 외부 카메라 입력은
현재 인식기의 공통 카메라 토픽에 launch remap으로 연결합니다. 코드 내부 Gazebo 경로는 1번에서 제거했습니다.

`pl_bringup/sim.launch.py`가 UDP Agent 하나를 소유하고 `real.launch.py`가 UART Agent 하나를
소유합니다. `run.sh`는 PX4를 먼저 부팅하고 sim launch를 실행하며 Agent를 따로 띄우지 않습니다.
설정 파일은 `src/pl_bringup/config/{sim,real}.yaml`이며 수정 뒤 `colcon build --packages-select pl_bringup`으로 설치합니다.

## 현재 검증 방법

일반 비행 검증은 [README 실행 방법](../README.md#6-실행방법)을 참고하세요.
과거 원본 bag, 소스 스냅샷, JSON 요약 및 전용 기록·검증·재생 스크립트는 삭제했습니다.
촬영 시각 기반 상태 보간과 Delayed KF 검증 데이터는 5번 구현 시 새로 수집합니다.

## 이번 검증 결과 (2026-11-10)

| 구성 | 시나리오 | 기록 폴더 | 영상 수 | target_pose 수 | 착륙 중심 오차 |
|---|---|---|---:|---:|---:|
| 기존 launch | 정적 | `20261110-193346-static` | 851 | 211 | 0.68cm |
| 기존 launch | 이동 0.3m/s | `20261110-193450-moving` | 1004 | 193 | 7.99cm |
| pl_bringup | 정적 | `20261110-193601-static` | 919 | 224 | 1.13cm |
| pl_bringup | 이동 0.3m/s | `20261110-193706-moving` | 1005 | 154 | 7.48cm |

모든 실행에서 필수 12개 토픽, 유효한 영상/CameraInfo, FSM의 Finished 전환과 자동 disarm을
검증했습니다. 위 차이는 각각 새 시뮬레이션에서 수행한 결과이며 개선 효과로 해석하지 않습니다.
변경 전후 인식·제어 C++ 소스 해시와 두 노드의 실제 런타임 파라미터는 일치합니다.

소스 구조 검증: pl_bringup 빌드, sim launch 인자 확인, real 미보정 설정 거부,
셸/Python 문법, SDF 구문, Git 공백 오류 검사를 통과했습니다.
실기체 빌드 선택 목록에 Gazebo 패키지와 PX4 SITL이 들어가지 않는 것도 확인했습니다.
실제 Jetson CSI·UART·비행은 이 PC에서 검증하지 않았습니다.

### 기록 영상 재생 검증

기존 구성의 정적 영상 851장, 이동 영상 1004장을 모두 순차 재생했습니다.
정적 참조 포즈 211개 중 입력 영상이 있는 210개, 이동 참조 포즈 193개 중 입력 영상이 있는
192개를 모두 재현했고, 매칭된 위치와 쿼터니언 성분의 최대 차이는 0이었습니다.
각 bag에서 포즈에 해당하는 원본 영상이 없는 1개는 수신 시점/기록 유실로 비교할 수 없으며
당시 결과에서 별도로 집계했습니다. 전체 검출 수는 원래 실시간 인식기에서
처리하지 못했던 입력도 순차 재생하기 때문에 증가할 수 있습니다.
