
/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved. 
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example demonstrates how to use the class-based API of ICEY
/// It essentially works the same as the plain ROS API, but we use an icey::Node instead of a
/// rclcpp::Node. It also shows that we do not need to store the ROS entities in the class
#include <icey/icey.hpp>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

using namespace std::chrono_literals;

class MyNode : public icey::Node {
public:
  explicit MyNode(const std::string& name) : icey::Node(name) {
    auto timer_signal = icey().create_timer(500ms, [this](size_t ticks) { on_tick(ticks); });
  }

  void on_tick(size_t ticks) {
    RCLCPP_INFO_STREAM(get_logger(), "Timer ticked: " << ticks);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>("icey_class_based_node_example");
  rclcpp::spin(node);
}