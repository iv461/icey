/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// A more complex async/await examples that shows how to wait on a timer a
#include <icey/icey.hpp>

#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

icey::Promise<void> create_and_spin_node(std::shared_ptr<icey::Node> node) {
  auto frequency = node->icey().declare_parameter<double>("frequency", 10.);  // Hz, i.e. 1/s
  auto amplitude = node->icey().declare_parameter<double>("amplitude", 2.);

  auto period_time = 100ms;
  auto timer = node->icey().create_timer(period_time);

  auto rectangle_pub = node->icey().create_publisher<std_msgs::msg::Float32>(
      "rectangle_signal", rclcpp::SystemDefaultsQoS());
  auto sine_pub = node->icey().create_publisher<std_msgs::msg::Float32>(
      "sine_signal", rclcpp::SystemDefaultsQoS());

  std::cout << "Starting loop .. " << std::endl;
  /// Main spinning loop
  while (true) {
    /// Receive timer updates
    size_t ticks = co_await timer;

    RCLCPP_INFO_STREAM(node->get_logger(), "Timer ticked: " << ticks);

    if (ticks % 10 == 0) {  /// Publish with 1/10th of the frequency
      std_msgs::msg::Float32 result;
      result.data = (ticks % 20 == 0) ? 1.f : 0.f;
      rectangle_pub.publish(result);
    }

    /// Add another computation for the timer
    std_msgs::msg::Float32 float_val;
    double period_time_s = 0.1;
    /// We can access parameters in callbacks using .value() because parameters are always
    /// initialized first.
    double y = amplitude * std::sin((period_time_s * ticks) * frequency * 2 * M_PI);
    float_val.data = y;
    RCLCPP_INFO_STREAM(node->get_logger(), "Publishing sine... " << y);
    sine_pub.publish(float_val);
  }
  co_return;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icey::Node>("icey_signal_generator_async_await_example");
  (void)create_and_spin_node(node);
  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 8};
  exec.add_node(node->get_node_base_interface());
  exec.spin();
}
