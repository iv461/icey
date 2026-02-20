/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example demonstrates how to use the icey Context for gradually adopting ICEY in an existing
/// Node.
// It also shows that we do not need to store the ROS entities in the class
#include <icey/icey.hpp>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node {
public:
  MyNode() : rclcpp::Node("icey_context_example_node") {
    icey_context_ = std::make_shared<icey::Context>(this);
    icey_context_->create_timer(500ms, [this](size_t ticks) { on_tick(ticks); });
  }

  void on_tick(size_t ticks) { RCLCPP_INFO_STREAM(get_logger(), "Timer ticked: " << ticks); }

  std::shared_ptr<icey::Context> icey_context_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 8};
  exec.add_node(node->get_node_base_interface());
  exec.spin();
}