cmake_minimum_required(VERSION 3.8)
project(laser_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# LiDAR SDK paths
set(LIDAR_SDK_ROOT "/media/wht/I/视频教程/自动驾驶/激光雷达/SDK_V3.0")

include_directories(
  ${LIDAR_SDK_ROOT}/include
  ${LIDAR_SDK_ROOT}/src/base
  ${LIDAR_SDK_ROOT}/dep
)
link_directories(${LIDAR_SDK_ROOT}/lib)

# Build the driver node
add_executable(laser_driver_node
  src/laser_driver_node.cpp
  ${LIDAR_SDK_ROOT}/src/base/HC_serial.cpp
  ${LIDAR_SDK_ROOT}/src/base/hchead.cpp
  ${LIDAR_SDK_ROOT}/src/base/hclidar.cpp
  ${LIDAR_SDK_ROOT}/src/base/HcSDK.cpp
  ${LIDAR_SDK_ROOT}/src/base/lidar.cpp
  ${LIDAR_SDK_ROOT}/src/base/ReadParsePackage.cpp
)
target_include_directories(laser_driver_node PRIVATE
  ${LIDAR_SDK_ROOT}/src/base
)
target_link_libraries(laser_driver_node lidar)
ament_target_dependencies(laser_driver_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS laser_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
