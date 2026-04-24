/*
 * laser_driver_node.cpp
 * Camsense X2M LiDAR ROS2 driver node.
 */

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

extern "C" {
#include "base/HcSDK.h"
}

using namespace std::chrono_literals;

class LaserDriverNode : public rclcpp::Node
{
public:
  LaserDriverNode()
  : Node("laser_driver_node")
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_ = this->declare_parameter<int>("baud", 115200);
    model_ = this->declare_parameter<std::string>("model", "X2M");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "scan");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "laser_frame");
    min_range_ = this->declare_parameter<double>("min_range", 0.05);
    max_range_ = this->declare_parameter<double>("max_range", 16.0);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "Initializing LiDAR on %s @ %d baud, model=%s",
                port_.c_str(), baud_, model_.c_str());

    int rtn = hcSDKInitialize(port_.c_str(), model_.c_str(), baud_, 2, false, false, true);
    if (!rtn) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize LiDAR SDK on %s", port_.c_str());
      rclcpp::shutdown();
      return;
    }

    setSDKCircleDataMode();
    setSDKLidarPowerOn(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    RCLCPP_INFO(this->get_logger(), "LiDAR ID: %s, Firmware: %s",
                getSDKLidarID(), getSDKFirmwareVersion());

    running_ = true;
    read_thread_ = std::thread(&LaserDriverNode::readLoop, this);
  }

  ~LaserDriverNode()
  {
    running_ = false;
    if (read_thread_.joinable()) read_thread_.join();
    hcSDKUnInit();
  }

private:
  void readLoop()
  {
    while (running_ && rclcpp::ok()) {
      LstPointCloud pcs;
      if (getSDKRxPointClouds(pcs) && !pcs.empty()) {
        std::sort(pcs.begin(), pcs.end(),
                  [](const tsPointCloud& a, const tsPointCloud& b) {
                    return a.dAngle < b.dAngle; });

        auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        msg->header.stamp = this->now();
        msg->header.frame_id = frame_id_;
        msg->angle_min = pcs.front().dAngle * M_PI / 180.0;
        msg->angle_max = pcs.back().dAngle * M_PI / 180.0;
        msg->angle_increment = (msg->angle_max - msg->angle_min) / (pcs.size() - 1);
        msg->time_increment = 0.0;
        msg->scan_time = 0.1;
        msg->range_min = min_range_;
        msg->range_max = max_range_;
        msg->ranges.resize(pcs.size());
        msg->intensities.resize(pcs.size());

        for (size_t i = 0; i < pcs.size(); ++i) {
          double d = pcs[i].u16Dist / 1000.0;
          msg->ranges[i] = (!pcs[i].bValid || d < min_range_ || d > max_range_)
            ? std::numeric_limits<float>::infinity() : static_cast<float>(d);
          msg->intensities[i] = static_cast<float>(pcs[i].u16Gray);
        }
        scan_pub_->publish(std::move(msg));
      }
      std::this_thread::sleep_for(1ms);
    }
  }

  std::string port_, model_, scan_topic_, frame_id_;
  int baud_;
  double min_range_, max_range_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::thread read_thread_;
  std::atomic<bool> running_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
