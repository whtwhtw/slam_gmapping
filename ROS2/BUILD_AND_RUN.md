# ============================================================
# ROS2 Humble gmapping 编译、启动指南
# 适用于 osrf/ros:humble-desktop-full 镜像
# ============================================================

# ============================================================
# 一、编译
# ============================================================

# 1. 进入容器（如果还没启动）
docker run -it --network host --privileged osrf/ros:humble-desktop-full bash

# 2. 创建 ROS2 工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 3. 将项目复制到工作空间
#    把 ROS2/gmapping 目录复制到 ~/ros2_ws/src/
cp -r /path/to/slam_gmapping/ROS2/gmapping ~/ros2_ws/src/

# 4. 安装系统依赖
apt-get update
apt-get install -y \
  ros-humble-openslam-gmapping \
  ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard \
  ros-humble-slam-toolbox \
  python3-colcon-common-extensions

# 5. 安装 openslam_gmapping（如果 apt 没有，需要从源码编译）
cd ~/ros2_ws/src
git clone https://github.com/ros-perception/openslam_gmapping.git -b ros2

# 6. 编译
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openslam_gmapping gmapping --cmake-args -DCMAKE_BUILD_TYPE=Release

# 7. 加载环境
source install/setup.bash

# ============================================================
# 二、启动 - 手动遥控建图模式
# ============================================================

# 终端 1：启动机器人底盘驱动（以差速机器人为例）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 终端 2：启动 gmapping
ros2 launch gmapping slam_gmapping.launch.py \
  scan_topic:=/scan \
  base_frame:=base_link \
  odom_frame:=odom \
  map_frame:=map

# 终端 3：启动 rviz2 查看地图
rviz2
# 在 rviz2 中添加 Map 显示，Topic 选择 /map

# 然后用键盘控制机器人移动，gmapping 会自动构建地图

# 建图完成后保存地图
ros2 run nav2_map_server map_saver_cli -f ~/my_map

# ============================================================
# 三、启动 - 自主探索建图模式（需要额外安装 explore_lite）
# ============================================================

# 安装 explore_lite
cd ~/ros2_ws/src
git clone https://github.com/hrnjcl/explore_lite.git
cd ~/ros2_ws
colcon build --packages-select explore_lite
source install/setup.bash

# 启动自主探索建图：
# 终端 1：启动底盘驱动 + gmapping（同上）

# 终端 2：启动 explore_lite
ros2 run explore_lite explore

# 机器人会自动探索环境并建图，完成后保存：
ros2 run nav2_map_server map_saver_cli -f ~/my_map

# ============================================================
# 四、启动 - 使用 Navigation2 完整导航栈
# ============================================================

# 建图完成后，使用 Nav2 进行导航：

# 1. 启动 Nav2（使用已保存的地图）
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  map:=~/my_map.yaml

# 2. 在 rviz2 中使用 "Nav2 Goal" 工具点击目标位置进行导航

# ============================================================
# 五、回放 rosbag2 测试
# ============================================================

# 使用已有的 rosbag2 数据测试 gmapping
ros2 run gmapping slam_gmapping_replay --bag_filename /path/to/bag.db3

# ============================================================
# 六、常见问题
# ============================================================

# Q: 编译报错找不到 openslam_gmapping
# A: 需要先编译 openslam_gmapping 依赖：
#    cd ~/ros2_ws/src && git clone https://github.com/ros-perception/openslam_gmapping.git -b ros2
#    cd ~/ros2_ws && colcon build --packages-select openslam_gmapping

# Q: 启动后收不到 /scan 数据
# A: 检查话题名是否匹配，通过 launch 文件的 scan_topic 参数修改

# Q: TF 变换树不完整
# A: 确保 robot_state_publisher 正确加载了 URDF，发布了 base_link→laser_frame

# Q: 地图不更新
# A: 检查 linearUpdate/angularUpdate 参数，确保机器人在移动
