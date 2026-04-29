cmake_minimum_required(VERSION 3.16)
project(cobot_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_STANDARD LESS 17)
  message(FATAL_ERROR "CMAKE_CXX_STANDARD must be 17 or higher")
endif()

# Clang-tidy as static analysis
if(NOT CMAKE_CXX_CLANG_TIDY)
  find_program(CLANG_TIDY clang-tidy)
  if(CLANG_TIDY)
    set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY} -extra-arg=-Wno-unknown-warning-option)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

# Cobot controller executable
add_executable(cobot_control src/cobot_control.cpp)
ament_target_dependencies(
  cobot_control
  rclcpp
  controller_interfaces
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_task_constructor_core
)

# Install targets
install(
  TARGETS cobot_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
