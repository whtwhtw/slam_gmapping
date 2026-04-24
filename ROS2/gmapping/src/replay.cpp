/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Modified for ROS2 by: Qwen Code
 */

#include "slam_gmapping.h"

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::string bag_filename;
  std::string scan_topic = "/scan";
  unsigned long int seed = 0;

  // Parse command line arguments
  for (int i = 1; i < argc; i++)
  {
    if (std::string(argv[i]) == "--bag_filename" && i + 1 < argc)
    {
      bag_filename = argv[++i];
    }
    else if (std::string(argv[i]) == "--scan_topic" && i + 1 < argc)
    {
      scan_topic = argv[++i];
    }
    else if (std::string(argv[i]) == "--seed" && i + 1 < argc)
    {
      seed = std::stoul(argv[++i]);
    }
    else if (std::string(argv[i]) == "--help")
    {
      std::cout << "Basic Command Line Parameter App" << std::endl
                << "--bag_filename <path>    rosbag2 database path" << std::endl
                << "--scan_topic <topic>     laser scan topic (default: /scan)" << std::endl
                << "--seed <value>           random seed (default: 0)" << std::endl
                << "--help                   print help" << std::endl;
      return 0;
    }
  }

  if (bag_filename.empty())
  {
    std::cerr << "ERROR: --bag_filename is required" << std::endl;
    return -1;
  }

  auto node = std::make_shared<SlamGMapping>();

  // Open rosbag2 database
  rosbag2_cpp::Reader reader;
  reader.open(bag_filename);

  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back("/tf");
  filter.topics.push_back(scan_topic);

  RCLCPP_INFO(node->get_logger(), "Playing bag file: %s", bag_filename.c_str());

  while (reader.has_next())
  {
    auto message = reader.read_next();

    if (message->topic_name == "/tf")
    {
      // Deserialize TF message
      rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serializer;
      tf2_msgs::msg::TFMessage tf_msg;
      rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
      tf_serializer.deserialize_message(&serialized_msg, &tf_msg);

      for (const auto& transform : tf_msg.transforms)
      {
        geometry_msgs::msg::TransformStamped stamped_tf;
        stamped_tf.header = transform.header;
        stamped_tf.child_frame_id = transform.child_frame_id;
        stamped_tf.transform = transform.transform;
        node->getTfBuffer()->setTransform(stamped_tf, "rosbag_authority", false);
      }
    }
    else if (message->topic_name == scan_topic)
    {
      // Deserialize LaserScan message
      rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_serializer;
      sensor_msgs::msg::LaserScan scan_msg;
      rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
      scan_serializer.deserialize_message(&serialized_msg, &scan_msg);

      rclcpp::Time stamp(scan_msg.header.stamp);
      if (stamp.seconds() != 0)
      {
        // Wait for TF to be available
        try
        {
          node->getTfBuffer()->lookupTransform(
            scan_msg.header.frame_id,
            node->getOdomFrame(),
            tf2_ros::fromMsg(scan_msg.header.stamp),
            tf2::durationFromSec(1.0));

          // Convert to ConstSharedPtr for callback
          auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan_msg);
          node->laserCallback(scan_ptr);
        }
        catch (tf2::TransformException& e)
        {
          RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", e.what());
        }
      }
    }

    // Small delay to simulate real-time playback
    std::this_thread::sleep_for(10ms);
  }

  reader.close();

  RCLCPP_INFO(node->get_logger(), "Replay finished.");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
