/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved. 
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example shows how to synchronize topics. Both for synchronizing two topics approximately
/// as well as obtaining the transform from TF (via lookupTransform) for a given
/// topic, which is another form of synchronization.

#include <icey/icey.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
using namespace std::chrono_literals;

int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "icey_synchronization_example");

  auto camera_image = node->icey().create_subscription<sensor_msgs::msg::Image>("camera");
  auto point_cloud = node->icey().create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud");

  /// Synchronize with a transform: This will yield the message and the transform from the
  /// child_frame_id of the header message and the given target_frame ("map") at the time of the
  /// header stamp.
  camera_image.synchronize_with_transform("map", 100ms)
      .then([](sensor_msgs::msg::Image::SharedPtr image,
               const geometry_msgs::msg::TransformStamped &transform_to_map) {
        std::cout << "image width: " << image->width
                  << " tf w: " << transform_to_map.transform.rotation.w << std::endl;
      });

  /// Or synchronize approx time:
  icey::synchronize_approx_time(100, camera_image, point_cloud)
      .then([&](sensor_msgs::msg::Image::SharedPtr,
               sensor_msgs::msg::PointCloud2::SharedPtr) {
                /*
                RCLCPP_INFO_STREAM(node->get_logger(), "Received camera with time " << image->header.stamp 
                << " and point cloud with time " << point_cloud->header.stamp);
                */
      });
  icey::spin(node);
}