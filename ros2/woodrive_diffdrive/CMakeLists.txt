cmake_minimum_required(VERSION 3.16)
project(woodrive_diffdrive)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Depends directly on core/ (same as woodrive_ros2) -- not on the
# woodrive_ros2 *package*, since this robot doesn't need woodrive_ros2's
# generic single-controller topics/services/actions, just the SDK itself.
add_executable(dual_motor_node
  src/dual_motor_node.cpp
  ../../core/WooDriveSdk.cpp
)
target_include_directories(dual_motor_node PRIVATE ../../core)
target_compile_options(dual_motor_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(dual_motor_node rclcpp)
install(TARGETS dual_motor_node DESTINATION lib/${PROJECT_NAME})

ament_package()
