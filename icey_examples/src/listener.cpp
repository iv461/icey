/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved. 
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example shows how to write a simple subscription.
/// It receives messages from the talker example.
#include <icey/icey.hpp>
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icey::Node>("icey_listener_example");
  
  node->icey().create_subscription<std_msgs::msg::String>("my_string", 
    [&](std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO_STREAM(node->get_logger(), "Got value: " << msg->data);
     });
  rclcpp::spin(node);
}