cmake_minimum_required(VERSION 3.8)
project(mpcc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclpy 
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  Eigen3
  vision_msgs
  ultralytics_ros
  tf2_ros)

  find_package(${PACKAGE} REQUIRED)

endforeach()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/include/opencv4/opencv2)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(ego_agent src/ego_agent.cc)
add_executable(following src/car_following.cc)
add_executable(detector src/car_detector.cc)

ament_target_dependencies(ego_agent rclcpp std_msgs sensor_msgs nav_msgs ackermann_msgs)
ament_target_dependencies(following rclcpp std_msgs sensor_msgs ultralytics_ros ackermann_msgs)
ament_target_dependencies(detector rclcpp std_msgs sensor_msgs geometry_msgs vision_msgs ultralytics_ros)

install(TARGETS
  ego_agent
  following
  detector
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
