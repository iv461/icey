/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// Reference implementation for benchmarking
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tf_lookup_reference");

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_node_clock_interface()->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface());
  tf_buffer->setCreateTimerInterface(timer_interface);
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/icey/test_pcl", rclcpp::SystemDefaultsQoS(),
      [&](sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {
        try {
          geometry_msgs::msg::TransformStamped transform_to_map = tf_buffer->lookupTransform(
              "map", point_cloud->header.frame_id, point_cloud->header.stamp, 200ms);
          RCLCPP_INFO(node->get_logger(), "Got transform %f",
                      transform_to_map.transform.rotation.w);
        } catch (const tf2::TransformException &e) {
          RCLCPP_INFO_STREAM(node->get_logger(), "Transform lookup error " << e.what());
        }
      });

  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 8};
  exec.add_node(node->get_node_base_interface());
  exec.spin();
}