#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher()
  : Node("odometry_publisher"), x_(0.0), y_(0.0), yaw_(0.0),
    current_linear_(0.0), current_angular_(0.0)
  {
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 30.0);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&OdometryPublisher::cmdVelCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
      std::bind(&OdometryPublisher::publishOdometry, this));

    RCLCPP_INFO(this->get_logger(), "Odometry publisher started (simulated)");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    current_linear_ = msg->linear.x;
    current_angular_ = msg->angular.z;
  }

  void publishOdometry() {
    double dt = 1.0 / publish_rate_;
    x_ += current_linear_ * std::cos(yaw_) * dt;
    y_ += current_linear_ * std::sin(yaw_) * dt;
    yaw_ += current_angular_ * dt;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = t.transform.rotation;
    odom.twist.twist.linear.x = current_linear_;
    odom.twist.twist.angular.z = current_angular_;
    odom_pub_->publish(odom);
  }

  double x_, y_, yaw_, current_linear_, current_angular_;
  std::string odom_frame_, base_frame_;
  double publish_rate_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
