cmake_minimum_required(VERSION 3.5)
project(pl_viz)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Only the optional Gazebo marker executable needs Gazebo development libraries.
option(PL_VIZ_GAZEBO "Build simulation-only Gazebo markers" OFF)
if(PL_VIZ_GAZEBO)
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
  add_executable(tag_pose_visualizer tag_pose_visualizer.cpp)
  ament_target_dependencies(tag_pose_visualizer rclcpp geometry_msgs)
  target_link_libraries(tag_pose_visualizer gz-transport13::core gz-msgs10::core)
  install(TARGETS tag_pose_visualizer DESTINATION lib/${PROJECT_NAME})
endif()
install(PROGRAMS monitor.py DESTINATION lib/${PROJECT_NAME} RENAME monitor)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
