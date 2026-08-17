cmake_minimum_required(VERSION 3.16)
project(woodrive_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WooDriveStatus.msg"
  "srv/BasicCheck.srv"
  "srv/MotionCommand.srv"
  "action/MovePosition.action"
  "action/AutoMotorSetup.action"
  DEPENDENCIES std_msgs
)

add_executable(woodrive_node
  src/woodrive_node.cpp
  ../../core/WooDriveSdk.cpp
)
target_include_directories(woodrive_node PRIVATE
  include
  ../../core
)
target_compile_options(woodrive_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(woodrive_node rclcpp rclcpp_action std_msgs std_srvs)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(woodrive_node "${cpp_typesupport_target}")

install(TARGETS woodrive_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Simple, one-shot example nodes (no topics/services/actions) mirroring
# raspberrypi/Example01..05 -- see examples/README.md. woodrive_node above
# is the persistent, full-featured driver; these are onboarding material.
set(WOODRIVE_EXAMPLES
  basic_check_node
  read_status_node
  auto_motor_setup_node
  speed_node
  position_node
)
foreach(example ${WOODRIVE_EXAMPLES})
  add_executable(${example}
    examples/${example}.cpp
    ../../core/WooDriveSdk.cpp
  )
  target_include_directories(${example} PRIVATE ../../core)
  target_compile_options(${example} PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(${example} rclcpp)
  install(TARGETS ${example} DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_utils test/test_command_utils.cpp)
  target_include_directories(test_command_utils PRIVATE include)
endif()

ament_package()
