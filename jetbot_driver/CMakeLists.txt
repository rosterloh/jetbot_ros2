cmake_minimum_required(VERSION 3.5)
project(jetbot_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jetbot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(include/)
file(GLOB_RECURSE AllHeaders ${PROJECT_SOURCE_DIR}/*.hpp)

add_executable(jetbot_driver
  src/composition.cpp
  src/jetbot_serial.cpp
  src/jetbot.cpp
  src/data.cpp
  src/pi_controller.cpp
  ${AllHeaders})

target_link_libraries(jetbot_driver pthread)

target_include_directories(jetbot_driver
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(jetbot_driver
  rclcpp
  std_msgs
  geometry_msgs
  jetbot_msgs
  nav_msgs
  tf2_geometry_msgs
  diagnostic_updater
  )

install(TARGETS
  jetbot_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()