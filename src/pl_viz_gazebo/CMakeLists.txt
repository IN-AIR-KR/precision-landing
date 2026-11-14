cmake_minimum_required(VERSION 3.5)
project(pl_viz_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(tag_pose_visualizer tag_pose_visualizer.cpp)
target_compile_features(tag_pose_visualizer PRIVATE cxx_std_17)
target_compile_options(tag_pose_visualizer PRIVATE -Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
ament_target_dependencies(tag_pose_visualizer rclcpp geometry_msgs)
target_link_libraries(tag_pose_visualizer gz-transport13::core gz-msgs10::core)
install(TARGETS tag_pose_visualizer DESTINATION lib/${PROJECT_NAME})

ament_package()
