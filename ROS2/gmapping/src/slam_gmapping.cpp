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
/* Modified by: Charles DuHadway */
/* Modified for ROS2 by: Qwen Code */

#include "slam_gmapping.h"

#include <iostream>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

using namespace std::chrono_literals;

SlamGMapping::SlamGMapping()
: SlamGMapping(rclcpp::NodeOptions())
{
}

SlamGMapping::SlamGMapping(const rclcpp::NodeOptions& options)
: Node("slam_gmapping", options),
  map_to_odom_(tf2::Transform::getIdentity()),
  laser_count_(0),
  transform_thread_(nullptr)
{
  init();
}

SlamGMapping::~SlamGMapping()
{
  if (transform_thread_) {
    transform_thread_->join();
    delete transform_thread_;
  }

  delete gsp_;
  if (gsp_laser_)
    delete gsp_laser_;
  if (gsp_odom_)
    delete gsp_odom_;
}

void SlamGMapping::init()
{
  gsp_ = new GMapping::GridSlamProcessor();
  if (!gsp_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create GridSlamProcessor");
    return;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  gsp_laser_ = nullptr;
  gsp_odom_ = nullptr;

  got_first_scan_ = false;
  got_map_ = false;

  // Parameters used by our GMapping wrapper
  throttle_scans_ = this->declare_parameter<int>("throttle_scans", 1);
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");

  transform_publish_period_ = this->declare_parameter<double>("transform_publish_period", 0.05);

  double tmp = this->declare_parameter<double>("map_update_interval", 5.0);
  map_update_interval_ = tf2::durationFromSec(tmp);

  // Parameters used by GMapping itself
  maxUrange_ = 0.0;
  maxRange_ = 0.0;
  minimum_score_ = this->declare_parameter<double>("minimumScore", 0.0);
  sigma_ = this->declare_parameter<double>("sigma", 0.05);
  kernelSize_ = this->declare_parameter<int>("kernelSize", 1);
  lstep_ = this->declare_parameter<double>("lstep", 0.05);
  astep_ = this->declare_parameter<double>("astep", 0.05);
  iterations_ = this->declare_parameter<int>("iterations", 5);
  lsigma_ = this->declare_parameter<double>("lsigma", 0.075);
  ogain_ = this->declare_parameter<double>("ogain", 3.0);
  lskip_ = this->declare_parameter<int>("lskip", 0);
  srr_ = this->declare_parameter<double>("srr", 0.1);
  srt_ = this->declare_parameter<double>("srt", 0.2);
  str_ = this->declare_parameter<double>("str", 0.1);
  stt_ = this->declare_parameter<double>("stt", 0.2);
  linearUpdate_ = this->declare_parameter<double>("linearUpdate", 1.0);
  angularUpdate_ = this->declare_parameter<double>("angularUpdate", 0.5);
  temporalUpdate_ = this->declare_parameter<double>("temporalUpdate", -1.0);
  resampleThreshold_ = this->declare_parameter<double>("resampleThreshold", 0.5);
  particles_ = this->declare_parameter<int>("particles", 30);
  xmin_ = this->declare_parameter<double>("xmin", -100.0);
  ymin_ = this->declare_parameter<double>("ymin", -100.0);
  xmax_ = this->declare_parameter<double>("xmax", 100.0);
  ymax_ = this->declare_parameter<double>("ymax", 100.0);
  delta_ = this->declare_parameter<double>("delta", 0.05);
  occ_thresh_ = this->declare_parameter<double>("occ_thresh", 0.25);
  llsamplerange_ = this->declare_parameter<double>("llsamplerange", 0.01);
  llsamplestep_ = this->declare_parameter<double>("llsamplestep", 0.01);
  lasamplerange_ = this->declare_parameter<double>("lasamplerange", 0.005);
  lasamplestep_ = this->declare_parameter<double>("lasamplestep", 0.005);
  tf_delay_ = this->declare_parameter<double>("tf_delay", transform_publish_period_);

  seed_ = std::chrono::system_clock::now().time_since_epoch().count();
}

void SlamGMapping::startLiveSlam()
{
  entropy_publisher_ = this->create_publisher<std_msgs::msg::Float64>("entropy", rclcpp::QoS(1).transient_local());
  sst_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
  sstm_ = this->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", rclcpp::QoS(1).transient_local());
  ss_ = this->create_service<nav_msgs::srv::GetMap>("dynamic_map",
    std::bind(&SlamGMapping::mapCallback, this, std::placeholders::_1, std::placeholders::_2));

  scan_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
    this, "scan", rmw_qos_profile_sensor_data);

  scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *scan_filter_sub_, *tf_buffer_, odom_frame_, 5, this->get_node_logging_interface(),
    this->get_node_clock_interface());

  scan_filter_->registerCallback(
    std::bind(&SlamGMapping::laserCallback, this, std::placeholders::_1));

  transform_thread_ = new std::thread(
    std::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
}

void SlamGMapping::publishLoop(double transform_publish_period)
{
  if (transform_publish_period == 0)
    return;

  rclcpp::Rate r(1.0 / transform_publish_period);
  while (rclcpp::ok()) {
    publishTransform();
    r.sleep();
  }
}

bool
SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t)
{
  geometry_msgs::msg::PoseStamped laser_pose;
  laser_pose.header.frame_id = laser_frame_;
  laser_pose.header.stamp = t;
  laser_pose.pose.position.x = 0;
  laser_pose.pose.position.y = 0;
  laser_pose.pose.position.z = 0;
  laser_pose.pose.orientation.x = 0;
  laser_pose.pose.orientation.y = 0;
  laser_pose.pose.orientation.z = 0;
  laser_pose.pose.orientation.w = 1;

  try
  {
    laser_pose = tf_buffer_->transform(laser_pose, odom_frame_);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_WARN(this->get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }

  double yaw = tf2::getYaw(laser_pose.pose.orientation);

  gmap_pose = GMapping::OrientedPoint(laser_pose.pose.position.x,
                                      laser_pose.pose.position.y,
                                      yaw);
  return true;
}

bool
SlamGMapping::initMapper(const sensor_msgs::msg::LaserScan& scan)
{
  laser_frame_ = scan.header.frame_id;

  // Get the laser's pose in the base frame
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = laser_frame_;
  ident.header.stamp = scan.header.stamp;
  ident.pose.position.x = 0;
  ident.pose.position.y = 0;
  ident.pose.position.z = 0;
  ident.pose.orientation.x = 0;
  ident.pose.orientation.y = 0;
  ident.pose.orientation.z = 0;
  ident.pose.orientation.w = 1;

  geometry_msgs::msg::PoseStamped laser_pose;
  try
  {
    laser_pose = tf_buffer_->transform(ident, base_frame_);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_WARN(this->get_logger(), "Failed to compute laser pose, aborting initialization (%s)",
               e.what());
    return false;
  }

  // Determine if laser is mounted planar
  tf2::Vector3 v(0, 0, 1 + laser_pose.pose.position.z);
  geometry_msgs::msg::Vector3Stamped up;
  up.vector.x = v.x();
  up.vector.y = v.y();
  up.vector.z = v.z();
  up.header.frame_id = base_frame_;
  up.header.stamp = scan.header.stamp;

  geometry_msgs::msg::Vector3Stamped up_transformed;
  try
  {
    up_transformed = tf_buffer_->transform(up, laser_frame_);
    RCLCPP_DEBUG(this->get_logger(), "Z-Axis in sensor frame: %.3f", up_transformed.vector.z);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_WARN(this->get_logger(), "Unable to determine orientation of laser: %s", e.what());
    return false;
  }

  if (std::fabs(std::fabs(up_transformed.vector.z) - 1) > 0.001)
  {
    RCLCPP_WARN(this->get_logger(),
               "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
               up_transformed.vector.z);
    return false;
  }

  gsp_laser_beam_count_ = scan.ranges.size();

  double angle_center = (scan.angle_min + scan.angle_max) / 2;

  if (up_transformed.vector.z > 0)
  {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    tf2::Quaternion q;
    q.setRPY(0, 0, angle_center);
    centered_laser_pose_ = tf2::Stamped<tf2::Transform>(
      tf2::Transform(q, tf2::Vector3(0, 0, 0)), tf2_ros::fromMsg(scan.header.stamp), laser_frame_);
    RCLCPP_INFO(this->get_logger(), "Laser is mounted upwards.");
  }
  else
  {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, -angle_center);
    centered_laser_pose_ = tf2::Stamped<tf2::Transform>(
      tf2::Transform(q, tf2::Vector3(0, 0, 0)), tf2_ros::fromMsg(scan.header.stamp), laser_frame_);
    RCLCPP_INFO(this->get_logger(), "Laser is mounted upside down.");
  }

  laser_angles_.resize(scan.ranges.size());
  double theta = -std::fabs(scan.angle_min - scan.angle_max) / 2;
  for (unsigned int i = 0; i < scan.ranges.size(); ++i)
  {
    laser_angles_[i] = theta;
    theta += std::fabs(scan.angle_increment);
  }

  RCLCPP_DEBUG(this->get_logger(),
              "Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f",
              scan.angle_min, scan.angle_max, scan.angle_increment);
  RCLCPP_DEBUG(this->get_logger(),
              "Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f",
              laser_angles_.front(), laser_angles_.back(), std::fabs(scan.angle_increment));

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // Get maxRange and maxUrange from parameters or scan
  maxRange_ = this->declare_parameter<double>("maxRange", scan.range_max - 0.01);
  maxUrange_ = this->declare_parameter<double>("maxUrange", maxRange_);

  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                          gsp_laser_beam_count_,
                                          std::fabs(scan.angle_increment),
                                          gmap_pose,
                                          0.0,
                                          maxRange_);
  if (!gsp_laser_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create RangeSensor");
    return false;
  }

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  if (!gsp_odom_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create OdometrySensor");
    return false;
  }

  GMapping::OrientedPoint initialPose;
  if (!getOdomPose(initialPose, scan.header.stamp))
  {
    RCLCPP_WARN(this->get_logger(),
               "Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }

  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  GMapping::sampleGaussian(1, seed_);

  RCLCPP_INFO(this->get_logger(), "Initialization complete");

  return true;
}

bool
SlamGMapping::addScan(const sensor_msgs::msg::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
  if (!getOdomPose(gmap_pose, scan.header.stamp))
    return false;

  if (scan.ranges.size() != gsp_laser_beam_count_)
    return false;

  double* ranges_double = new double[scan.ranges.size()];
  if (do_reverse_range_)
  {
    RCLCPP_DEBUG(this->get_logger(), "Inverting scan");
    int num_ranges = scan.ranges.size();
    for (int i = 0; i < num_ranges; i++)
    {
      if (scan.ranges[num_ranges - i - 1] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  }
  else
  {
    for (unsigned int i = 0; i < scan.ranges.size(); i++)
    {
      if (scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 rclcpp::Time(scan.header.stamp).seconds());

  delete[] ranges_double;

  reading.setPose(gmap_pose);

  RCLCPP_DEBUG(this->get_logger(), "processing scan");

  return gsp_->processScan(reading);
}

void
SlamGMapping::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static rclcpp::Time last_map_update(0, 0, RCL_ROS_TIME);

  if (!got_first_scan_)
  {
    if (!initMapper(*scan))
      return;
    got_first_scan_ = true;
  }

  GMapping::OrientedPoint odom_pose;

  if (addScan(*scan, odom_pose))
  {
    RCLCPP_DEBUG(this->get_logger(), "scan processed");

    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    RCLCPP_DEBUG(this->get_logger(), "new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    RCLCPP_DEBUG(this->get_logger(), "odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    RCLCPP_DEBUG(this->get_logger(), "correction: %.3f %.3f %.3f",
                mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf2::Transform laser_to_map = tf2::Transform(
      tf2::Quaternion(tf2::Vector3(0, 0, 1), mpose.theta),
      tf2::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf2::Transform odom_to_laser = tf2::Transform(
      tf2::Quaternion(tf2::Vector3(0, 0, 1), odom_pose.theta),
      tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));

    std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();

    rclcpp::Time scan_time = rclcpp::Time(scan->header.stamp);
    if (!got_map_ || (scan_time - last_map_update) > rclcpp::Duration(map_update_interval_))
    {
      updateMap(*scan);
      last_map_update = scan_time;
      RCLCPP_DEBUG(this->get_logger(), "Updated the map");
    }
  }
  else
    RCLCPP_DEBUG(this->get_logger(), "cannot process scan");
}

double
SlamGMapping::computePoseEntropy()
{
  double weight_total = 0.0;
  for (std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
       it != gsp_->getParticles().end();
       ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for (std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
       it != gsp_->getParticles().end();
       ++it)
  {
    if (it->weight / weight_total > 0.0)
      entropy += it->weight / weight_total * log(it->weight / weight_total);
  }
  return -entropy;
}

void
SlamGMapping::updateMap(const sensor_msgs::msg::LaserScan& scan)
{
  RCLCPP_DEBUG(this->get_logger(), "Update map");
  std::lock_guard<std::mutex> map_lock(map_mutex_);
  GMapping::ScanMatcher matcher;

  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
    gsp_->getParticles()[gsp_->getBestParticleIndex()];
  std_msgs::msg::Float64 entropy;
  entropy.data = computePoseEntropy();
  if (entropy.data > 0.0)
    entropy_publisher_->publish(entropy);

  if (!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  GMapping::Point center;
  center.x = (xmin_ + xmax_) / 2.0;
  center.y = (ymin_ + ymax_) / 2.0;

  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, delta_);

  RCLCPP_DEBUG(this->get_logger(), "Trajectory tree:");
  for (GMapping::GridSlamProcessor::TNode* n = best.node;
       n;
       n = n->parent)
  {
    RCLCPP_DEBUG(this->get_logger(), "  %.3f %.3f %.3f", n->pose.x, n->pose.y, n->pose.theta);
    if (!n->reading)
    {
      RCLCPP_DEBUG(this->get_logger(), "Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  if (map_.map.info.width != (unsigned int)smap.getMapSizeX() ||
      map_.map.info.height != (unsigned int)smap.getMapSizeY())
  {
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;

    RCLCPP_DEBUG(this->get_logger(),
                "map size is now %dx%d pixels (%f,%f)-(%f, %f)",
                smap.getMapSizeX(), smap.getMapSizeY(),
                xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    RCLCPP_DEBUG(this->get_logger(), "map origin: (%f, %f)",
                map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  for (int x = 0; x < smap.getMapSizeX(); x++)
  {
    for (int y = 0; y < smap.getMapSizeY(); y++)
    {
      GMapping::IntPoint p(x, y);
      double occ = smap.cell(p);
      assert(occ <= 1.0);
      if (occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if (occ > occ_thresh_)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  map_.map.header.stamp = this->get_clock()->now();
  map_.map.header.frame_id = map_frame_;

  sst_->publish(map_.map);
  sstm_->publish(map_.map.info);
}

void
SlamGMapping::mapCallback(
  const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
  std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (got_map_ && map_.map.info.width && map_.map.info.height)
  {
    *response = map_;
  }
}

void SlamGMapping::publishTransform()
{
  std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
  rclcpp::Time tf_expiration = this->get_clock()->now() + rclcpp::Duration::from_seconds(tf_delay_);
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = tf_expiration;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;
  transform_stamped.transform = tf2::toMsg(map_to_odom_);
  tf_broadcaster_->sendTransform(transform_stamped);
}
