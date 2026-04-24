/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Brian Gerkey */
/* Modified for ROS2 by: Qwen Code */

#ifndef SLAM_GMAPPING_H
#define SLAM_GMAPPING_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/message_filter.h"
#include "tf2/LinearMath/Transform.h"
#include "message_filters/subscriber.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"

#include <thread>
#include <mutex>
#include <memory>

class SlamGMapping : public rclcpp::Node
{
  public:
    SlamGMapping();
    explicit SlamGMapping(const rclcpp::NodeOptions& options);
    ~SlamGMapping();

    void init();
    void startLiveSlam();
    void startReplay(const std::string& bag_fname, std::string scan_topic);
    void publishTransform();
    void publishLoop(double transform_publish_period);
    void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);
    void mapCallback(
      const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
      std::shared_ptr<nav_msgs::srv::GetMap::Response> response);

    // For replay access
    std::shared_ptr<tf2_ros::Buffer>& getTfBuffer() { return tf_buffer_; }
    const std::string& getOdomFrame() const { return odom_frame_; }

  private:
    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr entropy_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr sst_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr sstm_;
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr ss_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Message filter
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_filter_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;

    // GMapping core
    GMapping::GridSlamProcessor* gsp_;
    GMapping::RangeSensor* gsp_laser_;
    std::vector<double> laser_angles_;
    tf2::Stamped<tf2::Transform> centered_laser_pose_;
    bool do_reverse_range_;
    unsigned int gsp_laser_beam_count_;
    GMapping::OdometrySensor* gsp_odom_;

    bool got_first_scan_;
    bool got_map_;
    nav_msgs::srv::GetMap::Response map_;

    tf2::Duration map_update_interval_;
    tf2::Transform map_to_odom_;
    std::mutex map_to_odom_mutex_;
    std::mutex map_mutex_;

    int laser_count_;
    int throttle_scans_;

    std::thread* transform_thread_;

    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    void updateMap(const sensor_msgs::msg::LaserScan& scan);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t);
    bool initMapper(const sensor_msgs::msg::LaserScan& scan);
    bool addScan(const sensor_msgs::msg::LaserScan& scan, GMapping::OrientedPoint& gmap_pose);
    double computePoseEntropy();

    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;

    unsigned long int seed_;

    double transform_publish_period_;
    double tf_delay_;
};

#endif  // SLAM_GMAPPING_H
