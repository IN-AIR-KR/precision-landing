# 하드웨어 세팅

## 카메라 연결

1. ros 2 usb cam 패키지 다운로드
   - https://github.com/ros-drivers/usb_cam.git

   ```bash
   sudo apt-get install ros-humble-usb-cam
   ```

2. 테스트
   - ros 2 usb cam 실행

   ```bash
   ros2 run usb_cam usb_cam_node_exe
   ```

   - rqt에서 `/image_raw` 선택해서 확인 가능

   ```bash
   ros2 run rqt_image_view rqt_image_view
   ```

## 정밀착륙 연동

1. `src/aruco_tracker/ArucoTracker.cpp` 코드에서 영상 입력 topic 수정

   ```c++
       _image_sub = create_subscription<sensor_msgs::msg::Image>(
                       "/image_raw", qos, std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));
   ```

2. 빌드 후 적용

   ```bash
   colcon build --packages-select aruco_tracker
   source install/setup.bash
   ```

3. 실행
   - 기존 SITL과 동일하게 실행
   - rqt에서 `/image_proc` 선택
