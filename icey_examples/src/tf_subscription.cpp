/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example shows another way to obtain transfroms in ICEY:
/// Directly subscribing to a transform between two coordinate systems: A callback gets called each
/// time the transform changes.
#include <icey/icey.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icey::Node>("icey_tf_subscription_example");

  node->icey()
      .create_transform_subscription("map", "base_link")
      .unwrap_or([&](std::string error) {
        RCLCPP_INFO_STREAM(node->get_logger(), "Transform subscription failed: " << error);
      })
      .then([&](const geometry_msgs::msg::TransformStamped &new_transform) {
        Eigen::Matrix4d tf_mat = tf2::transformToEigen(new_transform.transform).matrix();
        RCLCPP_INFO_STREAM(node->get_logger(), "Received a new transform:\n" << tf_mat);
      });

  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 8};
  exec.add_node(node->get_node_base_interface());
  exec.spin();
}