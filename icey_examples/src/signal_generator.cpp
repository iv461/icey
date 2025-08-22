/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved. 
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#include <icey/icey.hpp>

#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icey::Node>("icey_signal_generator_example");

  auto frequency = node->icey().declare_parameter<double>("frequency", 10., icey::Interval(0., 100.));  // Hz, i.e. 1/s
  auto amplitude = node->icey().declare_parameter<double>("amplitude", 2., icey::Interval(0., 10.));

  auto period_time = 100ms;
  auto timer = node->icey().create_timer(period_time);

  /// Add a computation and publish the result:
  timer
      .then([](size_t ticks) {
        std::optional<std_msgs::msg::Float32> result;
        if (ticks % 10 == 0) {  /// Publish with 1/10th of the frequency
          result = std_msgs::msg::Float32();
          result->data = (ticks % 20 == 0) ? 1.f : 0.f;
        }
        return result;
      })
      .publish("rectangle_signal");

  /// Add another computation for the timer
  timer
      .then([&](size_t ticks) {
        std_msgs::msg::Float32 float_val;
        double period_time_s = 0.1;
        /// We can access the parameter value by implicit conversion (or explicitly using .value())
        double y = amplitude * std::sin((2 * M_PI) * frequency * (period_time_s * ticks));
        float_val.data = y;
        RCLCPP_INFO_STREAM(node->get_logger(), "Publishing sine... " << y);
        return float_val;
      })
      .publish("sine_signal");

  rclcpp::spin(node);
}