/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// Driver for the tf_subscription test: It publishes a point cloud with a synchronized transform.
#include <icey/icey.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<icey::Node>("icey_tf_pub_example");
  auto pub = node->icey().create_publisher<sensor_msgs::msg::PointCloud2>("/icey/test_pcl");
  auto tf_pub = node->icey().create_transform_publisher();

  std::size_t cnt{0};
  icey::Time base_time{1700000000s};

  node->icey().create_timer(100ms, [&](std::size_t) {
    sensor_msgs::msg::PointCloud2 pcl;
    pcl.header.frame_id = "lidar";
    pcl.header.stamp = icey::rclcpp_from_chrono(base_time + cnt * 10ms);
    pub.publish(pcl);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = icey::rclcpp_from_chrono(base_time + cnt * 10ms);
    tf.header.frame_id = "map";
    tf.child_frame_id = "lidar";

    tf.transform.rotation.z = std::sin(0.001 * cnt);
    tf.transform.rotation.w = std::cos(0.001 * cnt);
    tf_pub.publish(tf);
    cnt++;
  });

  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 8};
  exec.add_node(node->get_node_base_interface());
  exec.spin();
}
