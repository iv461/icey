/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example shows that ICEY supports lifecycle nodes:
/// It is similar to this example:
/// https://github.com/ros2/demos/blob/rolling/lifecycle/src/lifecycle_talker.cpp
#include <icey/icey.hpp>

#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/// This example shows how the icey::Context can be initialized using an
/// rclcpp_lifecycle::LifecycleNode as well. You can also use the convenience class
/// icey::LifecycleNode.
class ExampleLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using Base = rclcpp_lifecycle::LifecycleNode;

  ExampleLifecycleNode(std::string name) : Base(name) {
    icey_context_ = std::make_shared<icey::Context>(this);
    timer_ = icey_context_->create_timer(100ms);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "on_activate() was called");
    /// The base class of an icey::LifecycleNode is a rclcpp::LifecycleNode, so it has all the
    /// lifecycle methods:
    Base::on_activate(state);
    /// Reset the timer again, starting the loop:
    timer_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "on_cleanup() was called");
    /// The base class of an icey::LifecycleNode is a rclcpp::LifecycleNode, so it has all the
    /// lifecycle methods:
    Base::on_cleanup(state);
    timer_.cancel();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Spin the node: we do some work here, other callbacks get called
  icey::Promise<void> run() {
    while (true) {
      std::size_t ticks = co_await timer_;
      RCLCPP_INFO_STREAM(get_logger(), "Spinning node for the " << ticks << "th time...");
    }
    co_return;
  }

  /// We store the timer here only to be able to cancel it
  icey::TimerStream timer_;
  std::shared_ptr<icey::Context> icey_context_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleLifecycleNode>("icey_lifecycle_node_example");
  node->run();
  return 0;
}