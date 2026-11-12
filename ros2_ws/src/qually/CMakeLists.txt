cmake_minimum_required(VERSION 3.8)
project(qually)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_executable(qually_fsm src/qually_fsm.cpp)
target_include_directories(qually_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(qually_fsm PUBLIC cxx_std_17)
ament_target_dependencies(qually_fsm
  rclcpp
  std_msgs
  mavros_msgs
  interfaces
)

install(TARGETS qually_fsm
  DESTINATION lib/${PROJECT_NAME})

ament_package()
