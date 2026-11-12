cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)

# heading_controller = the ROS node + the PID maths
add_executable(heading_controller
  src/heading_controller.cpp
  src/pid.cpp
)
target_include_directories(heading_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(heading_controller PUBLIC cxx_std_17)
ament_target_dependencies(heading_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  interfaces
)

install(TARGETS heading_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
