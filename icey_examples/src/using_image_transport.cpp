/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// Example showing how to use the image_transport Subscriptions/Publishers for subscribing
/// simultaneously at the image as well as the CameraInfo message. The CameraInfo message holds
/// information about the pinhole camera model that is required for computer vision algorithms.
/// But one of the most
/// important use cases for image_transport is to compress a camera image before publishing it. This
/// is useful to be able to record images into a rosbag and prevent it from growing to tens of
/// gigabytes after just a few minutes.

#include <icey/icey.hpp>
#include <icey/icey_image_transport.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icey::Node>("icey_image_transport_example");

  /// Create an image_transport::CameraSubscription. The second argument is the transport, i.e. the
  /// compression algorithm to use. Common ones are "raw", "theora" etc.
  auto camera_center_sub = node->icey().create_stream<icey::CameraSubscription>(
      "camera_center", "raw", rclcpp::SensorDataQoS());

  camera_center_sub
      .then([&](sensor_msgs::msg::Image::ConstSharedPtr /*image*/,
                sensor_msgs::msg::CameraInfo::ConstSharedPtr /*camera_info*/) {
        RCLCPP_INFO_STREAM(node->get_logger(), "Received image and info: ");
      })
      /// Optionally, you can handle the exception that the transport plugin could not be loaded
      /// (otherwise it will just throw the exception unhandled)
      .except([&](const image_transport::TransportLoadException &exception) {
        RCLCPP_ERROR_STREAM(node->get_logger(),
                            "Could not load plugin, got exception: " << exception.what());
      });

  /// Create a regular image_transport::Subscription to receive compressed images
  auto camera_left_sub = node->icey().create_stream<icey::ImageTransportSubscription>(
      "camera_left", "raw", rclcpp::SensorDataQoS());

  camera_left_sub
      /// We receive here the image after image_transport has decompressed it:
      .then([](sensor_msgs::msg::Image::ConstSharedPtr image) {
        /// Process your image, e.g. paint it here:
        // As an example, we simply copy the image, output must be a shared_ptr
        return std::make_shared<sensor_msgs::msg::Image>(*image);
      })
      /// And now publish again using image_transport, creating an image_transport::Publisher:
      .publish<icey::ImageTransportPublisher>("camera_left_painted", rclcpp::SensorDataQoS());

  rclcpp::spin(node);
}