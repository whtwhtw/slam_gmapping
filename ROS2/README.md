# gmapping ROS2 Humble 迁移说明

## 概述

本项目是将原始的 ROS1 `gmapping` 包迁移到 ROS2 Humble 版本。迁移过程中对 API、构建系统和代码结构进行了全面更新。

## 主要变更

### 1. 构建系统

| ROS1 | ROS2 |
|------|------|
| catkin | ament_cmake |
| `find_package(catkin REQUIRED ...)` | `find_package(ament_cmake REQUIRED)` + 逐个 `find_package` |
| `catkin_package()` | `ament_package()` |
| `catkin_EXPORTED_TARGETS` | 不需要 |

### 2. 依赖包变化

| ROS1 | ROS2 |
|------|------|
| `roscpp` | `rclcpp` |
| `tf` | `tf2` + `tf2_ros` + `tf2_geometry_msgs` |
| `nodelet` | `rclcpp_components` |
| `rosbag_storage` | `rosbag2_cpp` + `rosbag2_storage` |
| `rostest` | `ament_lint_auto` + `ament_lint_common` |

### 3. C++ API 迁移

| ROS1 API | ROS2 API |
|----------|----------|
| `ros::NodeHandle` | `rclcpp::Node` (继承) |
| `ros::Publisher` | `rclcpp::Publisher<T>` |
| `ros::Subscriber` | `rclcpp::Subscription<T>` |
| `ros::ServiceServer` | `rclcpp::Service<T>` |
| `ros::Time` | `rclcpp::Time` |
| `ros::Duration` | `rclcpp::Duration` |
| `ros::Rate` | `rclcpp::Rate` |
| `ros::spin()` | `rclcpp::spin()` |
| `ros::ok()` | `rclcpp::ok()` |
| `ros::init()` | `rclcpp::init()` |
| `ros::shutdown()` | `rclcpp::shutdown()` |

### 4. TF2 迁移

| ROS1 tf | ROS2 tf2 |
|---------|----------|
| `tf::TransformListener` | `tf2_ros::Buffer` + `tf2_ros::TransformListener` |
| `tf::TransformBroadcaster` | `tf2_ros::TransformBroadcaster` |
| `tf::MessageFilter` | `tf2_ros::MessageFilter` |
| `tf::Stamped<tf::Pose>` | `tf2::Stamped<tf2::Transform>` |
| `tf::Transform` | `tf2::Transform` |
| `tf::Vector3` | `tf2::Vector3` |
| `tf::Quaternion` | `tf2::Quaternion` |
| `tf::getYaw()` | `tf2::getYaw()` |
| `tf::createQuaternionFromRPY()` | `tf2::Quaternion::setRPY()` |

### 5. 线程和智能指针

| ROS1 (Boost) | ROS2 (std) |
|--------------|------------|
| `boost::thread` | `std::thread` |
| `boost::mutex` | `std::mutex` |
| `boost::lock_guard` | `std::lock_guard` |
| `boost::shared_ptr` | `std::shared_ptr` |

### 6. Nodelet → Component

ROS1 的 `nodelet` 机制被 ROS2 的 `rclcpp_components` 取代：

**ROS1:**
```cpp
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

class SlamGMappingNodelet : public nodelet::Nodelet {
  virtual void onInit() { ... }
};
PLUGINLIB_EXPORT_CLASS(SlamGMappingNodelet, nodelet::Nodelet)
```

**ROS2:**
```cpp
#include <rclcpp_components/register_node_macro.hpp>

class SlamGMappingComponent : public SlamGMapping {
  explicit SlamGMappingComponent(const rclcpp::NodeOptions& options) { ... }
};
RCLCPP_COMPONENTS_REGISTER_NODE(SlamGMappingComponent)
```

### 7. Rosbag → Rosbag2

**ROS1:**
```cpp
#include <rosbag/bag.h>
#include <rosbag/view.h>
rosbag::Bag bag;
bag.open(filename, rosbag::bagmode::Read);
rosbag::View view(bag, rosbag::TopicQuery(topics));
```

**ROS2:**
```cpp
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
rosbag2_cpp::Reader reader;
reader.open(bag_filename);
while (reader.has_next()) {
  auto message = reader.read_next();
  // 反序列化消息
}
```

### 8. 参数声明

**ROS1:**
```cpp
private_nh_.getParam("param_name", value);
private_nh_.param("param_name", value, default_value);
```

**ROS2:**
```cpp
value = this->declare_parameter<T>("param_name", default_value);
```

### 9. Launch 文件

**ROS1 (XML):**
```xml
<launch>
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping">
    <param name="param" value="value"/>
    <remap from="scan" to="base_scan"/>
  </node>
</launch>
```

**ROS2 (Python):**
```python
from launch_ros.actions import Node

Node(
    package='gmapping',
    executable='slam_gmapping',
    name='slam_gmapping',
    parameters=[{'param': 'value'}],
    remappings=[('scan', 'base_scan')],
)
```

## 编译与使用

### 编译

```bash
# 在工作空间根目录下
colcon build --packages-select gmapping
source install/setup.bash
```

### 运行

#### 方式一：直接运行节点
```bash
ros2 run gmapping slam_gmapping
```

#### 方式二：使用 launch 文件
```bash
ros2 launch gmapping slam_gmapping.launch.py
```

#### 方式三：作为 Component 加载
```bash
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager gmapping SlamGMappingComponent
```

#### 回放 rosbag2
```bash
ros2 run gmapping slam_gmapping_replay --bag_filename /path/to/bag.db3
```

### 自定义参数

```bash
ros2 launch gmapping slam_gmapping.launch.py \
  scan_topic:=/scan \
  base_frame:=base_link \
  odom_frame:=odom \
  map_frame:=map \
  map_update_interval:=5.0 \
  particles:=30 \
  delta:=0.05
```

## TF 坐标帧要求

与 ROS1 版本相同，需要以下 TF 变换链：

```
map → odom → base_link → laser_frame
```

- `map` → `odom`: 由 slam_gmapping 发布
- `odom` → `base_link`: 由里程计/机器人驱动提供
- `base_link` → `laser_frame`: 由机器人 URDF/robot_state_publisher 提供

## 已知问题

1. **openslam_gmapping 依赖**: 需要确保 `openslam_gmapping` 包已针对 ROS2 编译
2. **rosbag2 格式**: 仅支持 ROS2 的 rosbag2 格式（`.db3` 文件），不支持 ROS1 的 `.bag` 文件
3. **实时回放**: replay 模式目前使用固定延迟，未实现自适应播放速度

## 测试

测试框架从 `rostest` 迁移到 `ament_lint`，原有功能测试需要重写为 ROS2 格式。

```bash
colcon test --packages-select gmapping
colcon test-result --verbose
```
