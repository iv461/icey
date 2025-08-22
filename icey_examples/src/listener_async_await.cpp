/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved. 
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example shows how to use subscriptions with async/await syntax:
/// Instead of a callback, we call co_await on the subscription stream. 
/// we also must introduce a spinning loop so that we receive values continuously.
#include <icey/icey.hpp>

#include "std_msgs/msg/string.hpp"

icey::Promise<void> receive(std::shared_ptr<icey::Node> node) {
  icey::SubscriptionStream<std_msgs::msg::String> sub =
      node->icey().create_subscription<std_msgs::msg::String>("/strings");

  while (true) {
    std_msgs::msg::String::SharedPtr message = co_await sub;
    RCLCPP_INFO_STREAM(node->get_logger(), "Got message: " << message->data);
  }
  co_return;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icey::Node>("icey_listener_node");
  receive(node);
  rclcpp::spin(node);
}