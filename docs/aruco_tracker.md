# ArUco Tracker 시스템

## 목차

1. [시스템 개요](#시스템-개요)
2. [ArUco 마커란?](#aruco-마커란)
3. [시스템 아키텍처](#시스템-아키텍처)
4. [노드 구조](#노드-구조)
5. [마커 감지 알고리즘](#마커-감지-알고리즘)
6. [3D 위치 추정 (PnP)](#3d-위치-추정-pnp)
7. [카메라 캘리브레이션](#카메라-캘리브레이션)
8. [파라미터 설정](#파라미터-설정)
9. [실행 방법](#실행-방법)

---

## 시스템 개요

ArUco Tracker는 카메라 영상에서 ArUco 마커를 실시간으로 감지하고, 마커의 3D 위치와 방향(자세)를 추정하는 컴퓨터 비전 노드입니다. 드론의 정밀 착륙 시스템에서 목표 지점을 인식하는 핵심 역할을 수행합니다.

### 주요 기능

- **실시간 마커 감지**: OpenCV ArUco 라이브러리를 사용한 고속 마커 인식
- **3D 포즈 추정**: PnP (Perspective-n-Point) 알고리즘으로 마커의 위치와 자세 계산
- **카메라 왜곡 보정**: 렌즈 왜곡을 보정하여 정확한 위치 추정
- **시각화**: 감지된 마커와 좌표축을 영상에 오버레이하여 출력

---

## ArUco 마커란?

### ArUco 마커의 정의

ArUco는 **Augmented Reality University of Cordoba**의 약자로, 증강 현실 및 로봇 비전에서 널리 사용되는 2D 사각형 마커입니다.

```
┌─────────────────────┐
│ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ │  ← 검은색 테두리
│ ■ □ □ ■ ■ □ ■ □ □ ■ │
│ ■ □ ■ ■ □ ■ ■ ■ □ ■ │
│ ■ ■ □ □ □ ■ □ □ ■ ■ │  ← 내부 패턴 (ID 정보)
│ ■ □ ■ □ ■ □ ■ ■ □ ■ │
│ ■ ■ □ ■ □ ■ □ □ ■ ■ │
│ ■ □ ■ ■ □ □ ■ ■ □ ■ │
│ ■ □ □ ■ ■ ■ ■ □ □ ■ │
│ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ │
└─────────────────────┘
     ArUco Marker
     (예: ID = 0)
```

### ArUco 마커의 특징

1. **고유 ID**: 각 마커는 고유한 숫자 ID를 가짐
2. **빠른 감지**: 간단한 이진 패턴으로 고속 처리 가능
3. **에러 정정**: 패턴 손상이나 부분 가림에도 어느 정도 robust
4. **방향 인식**: 마커의 회전 상태 자동 인식
5. **3D 포즈 추정**: 단일 마커로도 6-DOF (위치 + 자세) 추정 가능

### Dictionary (사전)

ArUco 마커는 **Dictionary**라는 미리 정의된 패턴 집합에서 선택됩니다.

| Dictionary         | 설명                     | 마커 개수 | 비트 크기 |
| ------------------ | ------------------------ | --------- | --------- |
| `DICT_4X4_50`      | 4x4 비트, 50개           | 50        | 4x4       |
| `DICT_4X4_100`     | 4x4 비트, 100개          | 100       | 4x4       |
| **`DICT_4X4_250`** | 4x4 비트, 250개 (기본값) | 250       | 4x4       |
| `DICT_5X5_50`      | 5x5 비트, 50개           | 50        | 5x5       |
| `DICT_6X6_250`     | 6x6 비트, 250개          | 250       | 6x6       |

이 시스템에서는 **DICT_4X4_250** (dictionary = 2)을 사용합니다.

---

## 시스템 아키텍처

```
┌──────────────────────────────────────────────────────┐
│                   Gazebo Simulator                   │
│                                                      │
│  ┌──────────────┐              ┌──────────────┐      │
│  │   Camera     │              │ ArUco Marker │      │
│  │  (Downward)  │──────────▶   │  (Ground)    │      │
│  │              │   observes   │   ID: 0      │      │
│  └──────┬───────┘              └──────────────┘      │
└─────────┼────────────────────────────────────────────┘
          │
          │ /image (sensor_msgs/Image)
          │ /camera_info (sensor_msgs/CameraInfo)
          ▼
┌───────────────────────────────────────────────────────┐
│              ROS 2: aruco_tracker Node                │
│                                                       │
│  ┌────────────────────────────────────────────────┐   │
│  │           Image Callback Pipeline              │   │
│  │                                                │   │
│  │  1. ROS → OpenCV 변환                           │   │
│  │     ┌──────────────────────┐                   │   │
│  │     │  cv_bridge           │                   │   │
│  │     └──────┬───────────────┘                   │   │
│  │            │                                   │   │
│  │  2. ArUco 마커 감지                              │   │
│  │     ┌──────▼───────────────┐                   │   │
│  │     │ ArucoDetector        │                   │   │
│  │     │ detectMarkers()      │                   │   │
│  │     └──────┬───────────────┘                   │   │
│  │            │ corners[], ids[]                  │   │
│  │            │                                   │   │
│  │  3. 코너 포인트 왜곡 제거                           │   │
│  │     ┌──────▼───────────────┐                   │   │
│  │     │ undistortPoints()    │                   │   │
│  │     └──────┬───────────────┘                   │   │
│  │            │                                   │   │
│  │  4. 3D 포즈 추정 (PnP)                           │   │
│  │     ┌──────▼───────────────┐                   │   │
│  │     │ solvePnP()           │                   │   │
│  │     │ - Input: 2D corners  │                   │   │
│  │     │ - Input: 3D model    │                   │   │
│  │     │ - Output: rvec, tvec │                   │   │
│  │     └──────┬───────────────┘                   │   │
│  │            │                                   │   │
│  │  5. 회전 벡터 → 쿼터니언                           │   │
│  │     ┌──────▼───────────────┐                   │   │
│  │     │ Rodrigues()          │                   │   │
│  │     │ createFromRotMat()   │                   │   │
│  │     └──────┬───────────────┘                   │   │
│  │            │                                   │   │
│  │  6. ROS 메시지 발행                               │   │
│  │     ┌──────▼───────────────┐                   │   │
│  │     │ PoseStamped          │                   │   │
│  │     └──────────────────────┘                   │   │
│  └────────────────────────────────────────────────┘   │
└──────────┬───────────────────┬────────────────────────┘
           │                   │
           │ /target_pose      │ /image_proc
           │ (3D pose)         │ (annotated image)
           ▼                   ▼
     ┌──────────────┐    ┌──────────────┐
     │ precision_   │    │   Gazebo     │
     │    land      │    │ Visualization│
     └──────────────┘    └──────────────┘
```

---

## 노드 구조

### 클래스 다이어그램

```
┌─────────────────────────────────────────────┐
│          ArucoTrackerNode                   │
│         (rclcpp::Node)                      │
├─────────────────────────────────────────────┤
│ - _detector: ArucoDetector                  │
│ - _camera_matrix: cv::Mat (3x3)             │
│ - _dist_coeffs: cv::Mat (Nx1)               │
│ - _param_aruco_id: int                      │
│ - _param_dictionary: int                    │
│ - _param_marker_size: double                │
├─────────────────────────────────────────────┤
│ + ArucoTrackerNode()                        │
│ - loadParameters()                          │
│ - image_callback()                          │
│ - camera_info_callback()                    │
│ - annotate_image()                          │
└─────────────────────────────────────────────┘
```

### 주요 멤버 변수

#### 1. `_detector` (ArucoDetector)

- OpenCV의 ArUco 마커 감지 객체
- Dictionary와 DetectorParameters로 초기화
- `detectMarkers()` 메서드로 마커 감지

#### 2. `_camera_matrix` (3x3 행렬)

카메라 내부 파라미터 행렬 (Camera Intrinsic Matrix):

$$
K = \begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

여기서:

- $f_x, f_y$: 초점 거리 (focal length, 픽셀 단위)
- $c_x, c_y$: 주점 (principal point, 이미지 중심)

#### 3. `_dist_coeffs` (왜곡 계수)

렌즈 왜곡 모델 파라미터:

$$
\mathbf{d} = [k_1, k_2, p_1, p_2, k_3]
$$

- $k_1, k_2, k_3$: Radial distortion (방사형 왜곡)
- $p_1, p_2$: Tangential distortion (접선 왜곡)

### ROS 2 인터페이스

#### Subscriptions (입력)

1. **`/world/aruco/.../image`**
   - 타입: `sensor_msgs/msg/Image`
   - QoS: Best Effort
   - 역할: 카메라 영상 수신
   - 콜백: `image_callback()`

2. **`/world/aruco/.../camera_info`**
   - 타입: `sensor_msgs/msg/CameraInfo`
   - QoS: Best Effort
   - 역할: 카메라 보정 정보 수신 (1회만)
   - 콜백: `camera_info_callback()`

#### Publications (출력)

1. **`/target_pose`**
   - 타입: `geometry_msgs/msg/PoseStamped`
   - QoS: Best Effort
   - 내용: 마커의 3D 위치와 자세 (카메라 좌표계)
   - frame_id: `camera_frame`

2. **`/image_proc`**
   - 타입: `sensor_msgs/msg/Image`
   - QoS: Best Effort
   - 내용: 마커와 좌표축이 그려진 처리된 영상
   - 용도: 시각화 및 디버깅

---

## 마커 감지 알고리즘

### 1. 영상 전처리

```cpp
cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
```

ROS 메시지 (sensor_msgs/Image)를 OpenCV 형식 (cv::Mat)으로 변환합니다.

### 2. ArUco 마커 감지

```cpp
std::vector<int> ids;
std::vector<std::vector<cv::Point2f>> corners;
_detector->detectMarkers(cv_ptr->image, corners, ids);
```

#### 감지 과정

```
입력 영상 (BGR)
      ▼
┌──────────────────┐
│  그레이스케일       │
│  변환             │
└────────┬─────────┘
         ▼
┌──────────────────┐
│  적응형 임계값      │  ← 이진화
│  (Adaptive Thresh)│
└────────┬─────────┘
         ▼
┌──────────────────┐
│  윤곽선 검출        │  ← 사각형 찾기
│  (Contour)       │
└────────┬─────────┘
         ▼
┌──────────────────┐
│  후보 필터링        │  ← 크기, 비율 체크
└────────┬─────────┘
         ▼
┌──────────────────┐
│  패턴 인식         │  ← ID 디코딩
│  (Bit extraction)│
└────────┬─────────┘
         ▼
   ids[], corners[]
```

#### 출력

- **`ids`**: 감지된 마커의 ID 리스트
  - 예: `[0, 5, 12]`
- **`corners`**: 각 마커의 4개 코너 좌표 (픽셀)
  - 순서: Top-Left → Top-Right → Bottom-Right → Bottom-Left (시계방향)
  - 예: `[[{10, 20}, {50, 22}, {48, 60}, {12, 58}], ...]`

### 3. 마커 시각화

```cpp
cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
```

영상에 감지된 마커의 테두리와 ID를 그립니다.

---

## 3D 위치 추정 (PnP)

### PnP (Perspective-n-Point) 문제

**목표**: 2D 이미지 좌표와 3D 월드 좌표의 대응 관계로부터 카메라의 위치와 자세를 추정

### 1. 왜곡 제거 (Undistortion)

렌즈 왜곡이 있는 2D 좌표를 왜곡이 제거된 좌표로 변환합니다.

```cpp
std::vector<cv::Point2f> undistortedCorner;
cv::undistortPoints(corner, undistortedCorner, _camera_matrix, _dist_coeffs,
                    cv::noArray(), _camera_matrix);
```

왜곡 모델:

$$
\begin{align}
x_{\text{distorted}} &= x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2p_1 xy + p_2(r^2 + 2x^2) \\
y_{\text{distorted}} &= y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1(r^2 + 2y^2) + 2p_2 xy
\end{align}
$$

여기서 $r^2 = x^2 + y^2$

### 2. 3D 모델 정의

마커의 실제 3D 좌표를 정의합니다 (마커 중심을 원점으로).

```cpp
float half_size = _param_marker_size / 2.0f;  // 예: 0.25m
std::vector<cv::Point3f> objectPoints = {
    cv::Point3f(-half_size,  half_size, 0),  // Top-left
    cv::Point3f( half_size,  half_size, 0),  // Top-right
    cv::Point3f( half_size, -half_size, 0),  // Bottom-right
    cv::Point3f(-half_size, -half_size, 0)   // Bottom-left
};
```

마커 좌표계:

```
       Y
       ▲
       │
  TL ──┼── TR
       │
───────┼───────▶ X
       │
  BL ──┼── BR
       │
       Z (out of page)
```

마커 크기 = 0.5m인 경우:

- TL: (-0.25, +0.25, 0)
- TR: (+0.25, +0.25, 0)
- BR: (+0.25, -0.25, 0)
- BL: (-0.25, -0.25, 0)

### 3. PnP 풀이

```cpp
cv::Vec3d rvec, tvec;
cv::solvePnP(objectPoints, undistortedCorners[i], _camera_matrix,
             cv::noArray(), rvec, tvec);
```

#### 수식

카메라 투영 모델:

$$
\lambda \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K [R | \mathbf{t}] \begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix}
$$

여기서:

- $(u, v)$: 2D 이미지 좌표 (픽셀)
- $(X, Y, Z)$: 3D 월드 좌표 (미터)
- $K$: 카메라 내부 파라미터 행렬
- $R$: 회전 행렬 (3x3)
- $\mathbf{t}$: 이동 벡터 (translation)
- $\lambda$: 깊이 스케일 팩터

#### 입력 및 출력

**입력**:

- `objectPoints`: 3D 포인트 (마커 좌표계)
- `undistortedCorners`: 2D 이미지 포인트 (왜곡 제거됨)
- `_camera_matrix`: 카메라 내부 행렬 $K$

**출력**:

- `rvec`: 회전 벡터 (Rodrigues 표현, 3x1)
  - 벡터의 방향: 회전축
  - 벡터의 크기: 회전 각도 (라디안)
- `tvec`: 이동 벡터 (3x1, 미터 단위)
  - 카메라 좌표계에서 마커까지의 거리

### 4. 회전 벡터 → 회전 행렬

Rodrigues 공식을 사용하여 회전 벡터를 회전 행렬로 변환:

```cpp
cv::Mat rot_mat;
cv::Rodrigues(rvec, rot_mat);
```

#### Rodrigues 공식

$$
R = I + \sin(\theta) \cdot K + (1 - \cos(\theta)) \cdot K^2
$$

여기서:

- $\theta = \|\mathbf{r}\|$ (회전 각도)
- $\mathbf{k} = \frac{\mathbf{r}}{\theta}$ (단위 회전축)
- $K$ = skew-symmetric matrix of $\mathbf{k}$:

$$
K = \begin{bmatrix}
0 & -k_z & k_y \\
k_z & 0 & -k_x \\
-k_y & k_x & 0
\end{bmatrix}
$$

### 5. 회전 행렬 → 쿼터니언

회전 행렬을 쿼터니언으로 변환하여 ROS 메시지에 담습니다.

```cpp
cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();
```

쿼터니언 $q = (w, x, y, z)$는 회전을 표현하는 4차원 단위 벡터입니다.

### 6. 좌표축 그리기

```cpp
cv::drawFrameAxes(cv_ptr->image, _camera_matrix, cv::noArray(),
                  rvec, tvec, _param_marker_size);
```

마커의 3D 좌표축을 영상에 투영하여 그립니다:

- **빨강**: X축
- **초록**: Y축
- **파랑**: Z축

---

## 카메라 캘리브레이션

### 캘리브레이션 정보 수신

```cpp
void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    _camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    _dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();

    // 정보 수신 후 구독 해제
    _camera_info_sub.reset();
}
```

### CameraInfo 메시지 구조

```yaml
header:
  stamp: ...
  frame_id: "camera_frame"
height: 480 # 영상 높이 (픽셀)
width: 640 # 영상 너비 (픽셀)
k: [
    fx,
    0,
    cx, # 3x3 카메라 행렬 (row-major)
    0,
    fy,
    cy,
    0,
    0,
    1,
  ]
d: [k1, k2, p1, p2, k3] # 왜곡 계수
```

### 예시 값

```
Camera Matrix K:
┌                    ┐
│ 554.25  0.0  320.5 │
│  0.0  554.25 240.5 │
│  0.0    0.0    1.0 │
└                    ┘

Distortion Coefficients:
[k1=-0.01, k2=0.001, p1=0.0, p2=0.0, k3=0.0]
```

해석:

- $f_x = f_y = 554.25$: 초점 거리 (픽셀)
- $c_x = 320.5, c_y = 240.5$: 주점 (이미지 중심, 640x480 영상의 중심)
- $k_1 = -0.01$: 약간의 배럴 왜곡 (음수)

### One-Time Subscription

카메라 보정 정보는 시간에 따라 변하지 않으므로, 첫 메시지를 받은 후 구독을 해제합니다.

```cpp
if (_camera_matrix.at<double>(0, 0) != 0) {
    RCLCPP_INFO(get_logger(), "Updated camera intrinsics from camera_info topic.");
    _camera_info_sub.reset();  // 구독 해제
}
```

---

## 파라미터 설정

### 파라미터 정의

| 파라미터      | 타입   | 기본값 | 설명                                     |
| ------------- | ------ | ------ | ---------------------------------------- |
| `aruco_id`    | int    | 0      | 추적할 ArUco 마커의 ID                   |
| `dictionary`  | int    | 2      | ArUco Dictionary 종류 (2 = DICT_4X4_250) |
| `marker_size` | double | 0.5    | 마커의 실제 크기 (미터)                  |

### Dictionary 코드 매핑

OpenCV ArUco Dictionary 열거형:

```cpp
enum PredefinedDictionaryType {
    DICT_4X4_50 = 0,
    DICT_4X4_100,
    DICT_4X4_250,        // 2 ← 기본값
    DICT_4X4_1000,
    DICT_5X5_50,
    DICT_5X5_100,
    DICT_5X5_250,
    // ... 등등
};
```

### 파라미터 파일

**위치**: `src/aruco_tracker/cfg/params.yaml`

```yaml
aruco_tracker:
  ros__parameters:
    aruco_id: 0 # 마커 ID
    dictionary: 2 # DICT_4X4_250
    marker_size: 0.5 # 0.5m = 50cm
```

### 파라미터 선택 가이드

#### 1. `aruco_id`

- 환경에 배치된 마커의 ID와 일치해야 함
- 여러 마커가 있어도 이 ID만 추적
- ArUco 마커 생성 시 지정한 ID 사용

#### 2. `dictionary`

- **4x4 Dictionary** (0-3): 빠른 감지, 적은 마커 수, 근거리용
- **5x5 Dictionary** (4-7): 중간 성능
- **6x6 Dictionary** (8-11): 느린 감지, 많은 마커 수, 원거리용
- **권장**: DICT_4X4_250 (코드 2) - 속도와 정확도 균형

#### 3. `marker_size`

- **매우 중요**: PnP 추정의 스케일을 결정
- 실제 프린트/제작한 마커의 크기를 **정확히** 측정하여 입력
- 단위: 미터 (m)
- 예시:
  - A4 용지 마커: 약 0.2m (20cm)
  - 착륙 패드 마커: 0.5m ~ 1.0m

---

## 처리 파이프라인 상세

### 전체 흐름도

```
┌───────────────────────────────────────────────────────────────┐
│                     image_callback()                          │
│                                                               │
│  ┌──────────────────────────────────────────────────┐         │
│  │ 1. ROS → OpenCV 변환                              │         │
│  │    cv_bridge::toCvCopy()                         │         │
│  └──────────────┬───────────────────────────────────┘         │
│                 ▼                                             │
│  ┌──────────────────────────────────────────────────┐         │
│  │ 2. ArUco 마커 감지                                 │         │
│  │    _detector->detectMarkers()                    │         │
│  │    Output: ids[], corners[]                      │         │
│  └──────────────┬───────────────────────────────────┘         │
│                 ▼                                             │
│  ┌──────────────────────────────────────────────────┐         │
│  │ 3. 감지된 마커 시각화                                 │         │
│  │    cv::aruco::drawDetectedMarkers()              │         │
│  └──────────────┬───────────────────────────────────┘         │
│                 ▼                                             │
│            Camera calibrated?                                 │
│                 │                                             │
│        ┌────────┴────────┐                                    │
│       No                Yes                                   │
│        │                 │                                    │
│        ▼                 ▼                                    │
│   [Skip]    ┌─────────────────────────────┐                   │
│             │ 4. 마커 코너 왜곡 제거           │                  │
│             │    cv::undistortPoints()    │                   │
│             └──────────┬──────────────────┘                   │
│                        ▼                                      │
│             ┌─────────────────────────────┐                   │
│             │ 5. Loop: 각 마커 처리          │                   │
│             │    if (id == target_id)     │                   │
│             └──────────┬──────────────────┘                   │
│                        ▼                                      │
│             ┌─────────────────────────────┐                   │
│             │ 6. 3D 객체 포인트 생성          │                   │
│             │    objectPoints (4 corners) │                   │
│             └──────────┬──────────────────┘                   │
│                        ▼                                      │
│             ┌─────────────────────────────┐                   │
│             │ 7. PnP 문제 풀이              │                   │
│             │    cv::solvePnP()           │                   │
│             │    Output: rvec, tvec       │                   │
│             └──────────┬──────────────────┘                   │
│                        ▼                                      │
│             ┌─────────────────────────────┐                   │
│             │ 8. 좌표축 시각화               │                   │
│             │    cv::drawFrameAxes()      │                   │
│             └──────────┬──────────────────┘                   │
│                        ▼                                      │
│             ┌─────────────────────────────┐                   │
│             │ 9. 회전 변환                  │                   │
│             │    Rodrigues() → Quaternion │                   │
│             └──────────┬──────────────────┘                   │
│                        ▼                                      │
│             ┌─────────────────────────────┐                   │
│             │ 10. ROS 메시지 발행            │                   │
│             │     /target_pose            │                   │
│             │     (PoseStamped)           │                   │
│             └──────────┬──────────────────┘                   │
│                        ▼                                      │
│             ┌─────────────────────────────┐                   │
│             │ 11. 영상 텍스트 주석            │                   │
│             │     annotate_image()        │                   │
│             │     "X: 1.2 Y: 0.3 Z: 2.5"  │                   │
│             └──────────┬──────────────────┘                   │
│                        ▼                                      │
│                     break (첫 마커만 처리)                       │
│                                                               │
│  ┌──────────────────────────────────────────────────┐         │
│  │ 12. 처리된 영상 발행                                 │         │
│  │     /image_proc (sensor_msgs/Image)              │         │
│  └──────────────────────────────────────────────────┘         │
│                                                               │
└───────────────────────────────────────────────────────────────┘
```

### 중요 로직: 첫 번째 마커만 처리

```cpp
for (size_t i = 0; i < ids.size(); i++) {
    if (ids[i] != _param_aruco_id) {
        continue;  // ID가 일치하지 않으면 스킵
    }

    // ... PnP 추정 및 발행 ...

    break;  // 첫 번째 일치하는 마커만 처리
}
```

여러 마커가 감지되어도 **target ID와 일치하는 첫 번째 마커**만 처리합니다.

---

## 좌표계와 출력

### 출력 좌표계

`/target_pose`의 좌표는 **카메라 광학 좌표계** (Optical Frame)입니다.

```
      Y (Down)
      │
      │
      └───── X (Right)
     ╱
    Z (Forward, away from camera)
```

### PoseStamped 메시지

```cpp
geometry_msgs::msg::PoseStamped pose_msg;
pose_msg.header.stamp = msg->header.stamp;
pose_msg.header.frame_id = "camera_frame";

// Position (tvec)
pose_msg.pose.position.x = tvec[0];  // 우측 방향
pose_msg.pose.position.y = tvec[1];  // 아래 방향
pose_msg.pose.position.z = tvec[2];  // 전방 (카메라로부터 거리)

// Orientation (quaternion)
pose_msg.pose.orientation.x = quat.x;
pose_msg.pose.orientation.y = quat.y;
pose_msg.pose.orientation.z = quat.z;
pose_msg.pose.orientation.w = quat.w;
```

### 해석 예시

```yaml
position:
  x: 0.05 # 마커가 카메라 중심에서 오른쪽으로 5cm
  y: -0.1 # 마커가 카메라 중심에서 위쪽으로 10cm (Y가 음수 = 위)
  z: 2.5 # 마커가 카메라로부터 2.5m 전방

orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0 # 회전 없음 (마커가 카메라와 평행)
```

---

## 시각화 출력

### 1. 마커 감지 시각화

```
┌────────────────────────┐
│                        │
│    ┌───────────┐       │
│    │ ■ ■ ■ ■ ■ │       │  ← 녹색 테두리
│    │ ■ □ □ ■ ■ │       │  ← 마커 ID 표시
│    │ ■ □ ■ ■ □ │  ID:0 │
│    │ ■ ■ □ □ □ │       │
│    │ ■ ■ ■ ■ ■ │       │
│    └───────────┘       │
│                        │
└────────────────────────┘
```

### 2. 좌표축 오버레이

```
┌────────────────────────┐
│                        │
│    ┌───────────┐       │
│    │           │       │
│    │     │     │       │  ← Z축 (파랑, 전방)
│    │     └──→  │       │  ← X축 (빨강, 우)
│    │      ↓    │       │  ← Y축 (초록, 하)
│    │           │       │
│    └───────────┘       │
│                        │
│  X: 0.05 Y: -0.1 Z: 2.5│  ← 위치 정보 텍스트
└────────────────────────┘
```

### 3. 텍스트 주석

```cpp
void annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);
    stream << "X: " << target[0] << " Y: " << target[1] << " Z: " << target[2];

    // 우측 하단에 텍스트 표시
    cv::putText(image->image, stream.str(), textOrg,
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
}
```

---

## 실행 방법

### 1. Launch File로 실행 (권장)

```bash
ros2 launch precision_land precision_landing_system.launch.py
```

ArUco Tracker가 자동으로 실행됩니다.

### 2. 독립 실행

```bash
# 파라미터 파일과 함께 실행
ros2 run aruco_tracker aruco_tracker --ros-args \
    --params-file src/aruco_tracker/cfg/params.yaml

# 파라미터 오버라이드
ros2 run aruco_tracker aruco_tracker --ros-args \
    -p aruco_id:=5 \
    -p marker_size:=0.3
```

### 3. 토픽 확인

```bash
# 마커 위치 확인
ros2 topic echo /target_pose

# 처리된 영상 확인 (RViz 또는 rqt_image_view)
ros2 run rqt_image_view rqt_image_view /image_proc
```

---

## 디버깅 및 트러블슈팅

### 문제 1: 마커가 감지되지 않음

**증상**:

- `/target_pose` 토픽에 메시지가 발행되지 않음
- `/image_proc`에 마커 테두리가 그려지지 않음

**원인 및 해결책**:

| 원인              | 확인 방법                | 해결책                              |
| ----------------- | ------------------------ | ----------------------------------- |
| 조명 부족         | 영상이 너무 어두움       | 조명 추가 또는 카메라 노출 증가     |
| 마커 크기 작음    | 마커가 영상의 5% 미만    | 마커 크기 증가 또는 카메라를 가까이 |
| 잘못된 Dictionary | ID가 일치해도 감지 안 됨 | `dictionary` 파라미터 확인          |
| 마커 손상/오염    | 마커 인쇄 품질 저하      | 새 마커로 교체                      |
| 블러 (흐림)       | 빠른 움직임/초점 불량    | 움직임 감소, 초점 재조정            |

### 문제 2: 위치 추정이 부정확함

**증상**:

- 마커는 감지되지만 위치가 실제와 다름
- 좌표축이 흔들림

**원인 및 해결책**:

```
잘못된 marker_size
└─▶ marker_size를 실제 크기로 정확히 설정

카메라 보정 오류
└─▶ 카메라 캘리브레이션 재수행
    $ ros2 run camera_calibration cameracalibrator ...

왜곡 계수 미설정
└─▶ camera_info 토픽이 올바르게 발행되는지 확인
    $ ros2 topic echo /camera_info
```

### 문제 3: "Missing camera calibration" 에러

**증상**:

```
[ERROR] [aruco_tracker]: Missing camera calibration
```

**원인**:

- `/camera_info` 토픽이 발행되지 않음
- `_camera_matrix`가 비어있음

**해결책**:

```bash
# camera_info 토픽 확인
ros2 topic list | grep camera_info

# camera_info 내용 확인
ros2 topic echo /world/aruco/.../camera_info

# Bridge 노드가 실행 중인지 확인
ros2 node list | grep bridge
```

### 문제 4: 성능 저하 (낮은 FPS)

**원인**:

- 고해상도 영상
- 너무 많은 마커 감지

**해결책**:

```yaml
# 영상 해상도 감소 (Gazebo 설정)
<width>640</width>  # 1280 → 640
<height>480</height> # 960 → 480

# 또는 detectMarkers 파라미터 튜닝
auto params = cv::aruco::DetectorParameters();
params.minMarkerPerimeterRate = 0.1;  # 최소 마커 크기 제한
params.maxMarkerPerimeterRate = 4.0;  # 최대 마커 크기 제한
```

---

## 성능 고려사항

### 처리 시간 분석

일반적인 640x480 영상 처리 시간 (대략):

| 단계         | 시간 (ms) | 비율     |
| ------------ | --------- | -------- |
| ArUco 감지   | 5-15      | 50%      |
| Undistortion | 1-2       | 10%      |
| solvePnP     | 2-5       | 20%      |
| 시각화       | 2-4       | 15%      |
| 기타         | 1         | 5%       |
| **총합**     | **11-27** | **100%** |

→ 약 **30-90 FPS** 처리 가능

### 최적화 팁

1. **영상 해상도 감소**: 640x480 이하로 설정
2. **ROI 설정**: 관심 영역만 처리
3. **감지 빈도 조절**: 매 프레임이 아닌 N 프레임마다 감지
4. **GPU 가속**: OpenCV CUDA 버전 사용 (선택적)

---

## 관련 문서

### 내부 문서

- [정밀 착륙 시스템](precision_landing_system.md) - ArUco Tracker의 출력을 사용하는 상위 시스템

### 외부 참고자료

- [OpenCV ArUco Tutorial](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [PnP Problem](https://en.wikipedia.org/wiki/Perspective-n-Point)
- [Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [ROS 2 cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)

---

## 수식 요약

### 카메라 투영 모델

$$
\lambda \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \end{bmatrix} \begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix}
$$

### 렌즈 왜곡 모델

$$
\begin{align}
\mathbf{x}_d &= \mathbf{x}(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + \begin{bmatrix} 2p_1 xy + p_2(r^2 + 2x^2) \\ p_1(r^2 + 2y^2) + 2p_2 xy \end{bmatrix} \\
r^2 &= x^2 + y^2
\end{align}
$$

### Rodrigues 공식

$$
R = I + \frac{\sin \theta}{\theta} [\mathbf{r}]_\times + \frac{1 - \cos \theta}{\theta^2} [\mathbf{r}]_\times^2
$$

여기서 $\theta = \|\mathbf{r}\|$, $[\mathbf{r}]_\times$는 skew-symmetric matrix
