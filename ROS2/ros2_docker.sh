#!/bin/bash
# ROS2 gmapping Docker 管理脚本

CONTAINER_NAME="ros2-gmapping"
IMAGE_NAME="osrf/ros:humble-desktop-full"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE="${PROJECT_DIR}"

# 宿主代理地址（可修改）
HTTP_PROXY="${HTTP_PROXY:-http://127.0.0.1:7897}"
HTTPS_PROXY="${HTTPS_PROXY:-http://127.0.0.1:7897}"

start_container() {
    if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
        if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
            echo "容器 $CONTAINER_NAME 已在运行"
        else
            echo "启动容器 $CONTAINER_NAME..."
            docker start $CONTAINER_NAME
            xhost +local:docker
        fi
    else
        echo "创建并启动容器 $CONTAINER_NAME..."
        xhost +local:docker
        docker run -d \
            --name $CONTAINER_NAME \
            --privileged \
            --network host \
            -e HTTP_PROXY=$HTTP_PROXY \
            -e HTTPS_PROXY=$HTTPS_PROXY \
            -e http_proxy=$HTTP_PROXY \
            -e https_proxy=$HTTPS_PROXY \
            -e DISPLAY=$DISPLAY \
            -e QT_X11_NO_MITSHM=1 \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v $WORKSPACE:/root/ros2_ws/src \
            -v ~/.ssh:/root/.ssh:ro \
            -w /root/ros2_ws \
            $IMAGE_NAME \
            tail -f /dev/null

        # 配置 apt 使用代理
        docker exec $CONTAINER_NAME bash -c "echo 'Acquire::http::Proxy \"$HTTP_PROXY\";' > /etc/apt/apt.conf.d/99custom"
    fi
}

stop_container() {
    echo "停止容器 $CONTAINER_NAME..."
    docker stop $CONTAINER_NAME 2>/dev/null
}

remove_container() {
    echo "删除容器 $CONTAINER_NAME..."
    docker rm -f $CONTAINER_NAME 2>/dev/null
}

shell() {
    echo "进入容器 $CONTAINER_NAME 的bash..."
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && bash"
}

install_deps() {
    echo "安装系统依赖..."
    # 使用重试机制，代理不稳定可能 502
    local max_retries=3
    local retry_count=0
    while [ $retry_count -lt $max_retries ]; do
        if docker exec $CONTAINER_NAME bash -c "http_proxy=$HTTP_PROXY https_proxy=$HTTPS_PROXY apt-get update && http_proxy=$HTTP_PROXY https_proxy=$HTTPS_PROXY apt-get install -y --fix-missing \
            ros-humble-nav2-bringup \
            ros-humble-teleop-twist-keyboard \
            ros-humble-slam-toolbox \
            python3-colcon-common-extensions \
            ros-humble-rclcpp-components \
            ros-humble-nav2-msgs \
            ros-humble-rosbag2-cpp \
            ros-humble-rosbag2-storage"; then
            break
        fi
        retry_count=$((retry_count + 1))
        echo "下载失败，重试 $retry_count/$max_retries..."
        sleep 5
    done

    if [ $retry_count -eq $max_retries ]; then
        echo "依赖安装失败，请检查代理或网络连接"
        return 1
    fi

    # openslam_gmapping 需要从源码编译（使用默认分支，纯 C++ 库）
    echo "克隆 openslam_gmapping 源码..."
    docker exec $CONTAINER_NAME bash -c "cd /root/ros2_ws/src && git clone https://github.com/ros-perception/openslam_gmapping.git || true"
}

build() {
    echo "编译所有包..."
    docker exec $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && cd /root/ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
}

run_gmapping() {
    echo "启动 gmapping 建图..."
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch gmapping slam_gmapping.launch.py scan_topic:=/scan base_frame:=base_link odom_frame:=odom map_frame:=map"
}

run_complete_slam() {
    echo "启动完整 SLAM 栈..."
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch gmapping complete_slam.launch.py"
}

run_teleop() {
    echo "启动键盘遥控..."
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
}

run_rviz2() {
    echo "启动 RViz2..."
    xhost +local:docker
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && rviz2"
}

save_map() {
    echo "保存地图..."
    docker exec $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/my_map"
}

run_replay() {
    local bag_path="${1:-/root/ros2_ws/test_data.bag}"
    echo "回放 rosbag: $bag_path"
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run gmapping slam_gmapping_replay --bag_filename $bag_path"
}

case "$1" in
    start)
        start_container
        ;;
    stop)
        stop_container
        ;;
    rm)
        remove_container
        ;;
    shell|sh)
        shell
        ;;
    deps)
        install_deps
        ;;
    build)
        build
        ;;
    gmapping)
        run_gmapping
        ;;
    teleop)
        run_teleop
        ;;
    rviz2)
        run_rviz2
        ;;
    save-map)
        save_map
        ;;
    replay)
        run_replay "${2:-}"
        ;;
    *)
        echo "用法: $0 {start|stop|rm|shell|deps|build|gmapping|complete_slam|teleop|rviz2|save-map|replay [bag_path]}"
        echo ""
        echo "命令说明:"
        echo "  start          - 启动容器"
        echo "  stop           - 停止容器"
        echo "  rm             - 删除容器"
        echo "  shell          - 进入容器bash"
        echo "  deps           - 安装系统依赖"
        echo "  build          - 编译工作空间"
        echo "  gmapping       - 启动 gmapping 建图（需要外部 /scan 和 odom TF）"
        echo "  complete_slam  - 启动完整 SLAM 栈（含模拟里程计 + gmapping）"
        echo "  teleop         - 启动键盘遥控"
        echo "  rviz2          - 启动 RViz2"
        echo "  save-map       - 保存地图"
        echo "  replay         - 回放 rosbag 测试"
        exit 1
        ;;
esac
