/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example shows how to lookup transforms on TF asynchronously.
/// We look for each message that we get the transform and await it.
/// This example is therefore equivalent to the synchronize_with_transform() filter (except that the
/// input messages are unbuffered)
#include <icey/icey.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icey::Node>("icey_tf_lookup_async_await_example");

  icey::TransformBuffer tf_buffer = node->icey().create_transform_buffer();

  node->icey()
      // Use a coroutine (an asynchronous function) as a callback for the subscription:
      .create_subscription<sensor_msgs::msg::PointCloud2>(
          "/icey/test_pcl",
          [&tf_buffer,
           &node](sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) -> icey::Promise<void> {
            icey::Result<geometry_msgs::msg::TransformStamped, std::string> tf_result =
                co_await tf_buffer.lookup("map", point_cloud->header.frame_id,
                                          point_cloud->header.stamp, 200ms);

            if (tf_result.has_value()) {
              geometry_msgs::msg::TransformStamped transform_to_map = tf_result.value();
              RCLCPP_INFO(node->get_logger(), "Got transform %f",
                          transform_to_map.transform.rotation.w);
            } else {
              RCLCPP_INFO_STREAM(node->get_logger(),
                                 "Transform lookup error " << tf_result.error());
            }
            co_return;
          });

  rclcpp::spin(node);
}