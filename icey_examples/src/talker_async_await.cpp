/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example shows how to use a publisher with async/await syntax.
/// The synchronous call to .publish() simply calls publish on the ROS publisher.
#include <icey/icey.hpp>

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

icey::Promise<void> talk(std::shared_ptr<icey::Node> &node) {
  auto timer = node->icey().create_timer(100ms);
  auto pub = node->icey().create_publisher<std_msgs::msg::String>("/strings");
  while (true) {
    size_t ticks = co_await timer;
    std_msgs::msg::String message;
    message.data = "hello " + std::to_string(ticks);
    RCLCPP_INFO_STREAM(node->get_logger(), "Publishing: " << message.data);
    pub.publish(message);
  }
  co_return;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icey::Node>("icey_talker_async_await_example");

  (void)talk(node);

  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 8};
  exec.add_node(node->get_node_base_interface());
  exec.spin();
}
