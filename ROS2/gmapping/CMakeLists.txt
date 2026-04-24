cmake_minimum_required(VERSION 3.8)
project(gmapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# Openslam_gmapping include dir from install space
set(OPENSAM_GMAPPING_INCLUDE_DIRS
  "/root/ros2_ws/install/openslam_gmapping/include"
)
set(OPENSAM_GMAPPING_LIB_DIR
  "/root/ros2_ws/install/openslam_gmapping/lib"
)

# Build full paths to openslam_gmapping libraries (order matters!)
# Dependencies: gridfastslam -> scanmatcher -> log -> sensor_range -> sensor_base
#               scanmatcher -> sensor_odometry, utils
set(OPENSAM_GMAPPING_LIBS
  ${OPENSAM_GMAPPING_LIB_DIR}/libgridfastslam.a
  ${OPENSAM_GMAPPING_LIB_DIR}/libscanmatcher.a
  ${OPENSAM_GMAPPING_LIB_DIR}/liblog.a
  ${OPENSAM_GMAPPING_LIB_DIR}/libconfigfile.a
  ${OPENSAM_GMAPPING_LIB_DIR}/libsensor_range.a
  ${OPENSAM_GMAPPING_LIB_DIR}/libsensor_odometry.a
  ${OPENSAM_GMAPPING_LIB_DIR}/libsensor_base.a
  ${OPENSAM_GMAPPING_LIB_DIR}/libutils.a
)

# Add src directory to include path
include_directories(src ${OPENSAM_GMAPPING_INCLUDE_DIRS})

# Main slam_gmapping executable
add_executable(slam_gmapping
  src/slam_gmapping.cpp
  src/main.cpp
)
ament_target_dependencies(slam_gmapping
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)
target_link_libraries(slam_gmapping
  /root/ros2_ws/install/openslam_gmapping/lib/libgridfastslam.a
  /root/ros2_ws/install/openslam_gmapping/lib/libscanmatcher.a
  /root/ros2_ws/install/openslam_gmapping/lib/liblog.a
  /root/ros2_ws/install/openslam_gmapping/lib/libconfigfile.a
  /root/ros2_ws/install/openslam_gmapping/lib/libsensor_range.a
  /root/ros2_ws/install/openslam_gmapping/lib/libsensor_odometry.a
  /root/ros2_ws/install/openslam_gmapping/lib/libsensor_base.a
  /root/ros2_ws/install/openslam_gmapping/lib/libutils.a
)

# ROS2 Component library
add_library(slam_gmapping_component SHARED
  src/slam_gmapping.cpp
  src/component.cpp
)
target_compile_definitions(slam_gmapping_component PRIVATE "SLAM_GMAPPING_BUILDING_LIBRARY")
ament_target_dependencies(slam_gmapping_component
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)
target_link_libraries(slam_gmapping_component
  /root/ros2_ws/install/openslam_gmapping/lib/libgridfastslam.a
  /root/ros2_ws/install/openslam_gmapping/lib/libscanmatcher.a
  /root/ros2_ws/install/openslam_gmapping/lib/liblog.a
  /root/ros2_ws/install/openslam_gmapping/lib/libconfigfile.a
  /root/ros2_ws/install/openslam_gmapping/lib/libsensor_range.a
  /root/ros2_ws/install/openslam_gmapping/lib/libsensor_odometry.a
  /root/ros2_ws/install/openslam_gmapping/lib/libsensor_base.a
  /root/ros2_ws/install/openslam_gmapping/lib/libutils.a
)
rclcpp_components_register_node(slam_gmapping_component
  PLUGIN "SlamGMappingComponent"
  EXECUTABLE slam_gmapping_node
)

# Replay executable
add_executable(slam_gmapping_replay
  src/slam_gmapping.cpp
  src/replay.cpp
)
ament_target_dependencies(slam_gmapping_replay
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  rosbag2_cpp
  rosbag2_storage
)
target_link_libraries(slam_gmapping_replay
  /root/ros2_ws/install/openslam_gmapping/lib/libgridfastslam.a
  /root/ros2_ws/install/openslam_gmapping/lib/libscanmatcher.a
  /root/ros2_ws/install/openslam_gmapping/lib/liblog.a
  /root/ros2_ws/install/openslam_gmapping/lib/libconfigfile.a
  /root/ros2_ws/install/openslam_gmapping/lib/libsensor_range.a
  /root/ros2_ws/install/openslam_gmapping/lib/libsensor_odometry.a
  /root/ros2_ws/install/openslam_gmapping/lib/libsensor_base.a
  /root/ros2_ws/install/openslam_gmapping/lib/libutils.a
)

# Install targets
install(TARGETS slam_gmapping slam_gmapping_replay
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS slam_gmapping_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install component XML
install(FILES gmapping_component.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
