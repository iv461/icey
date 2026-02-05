/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// Action client example using ICEY async/await.
///
/// Key differences vs regular rclcpp_action API:
/// - No manual SendGoalOptions wiring for result/feedback callbacks when using co_await.
/// - Per-goal timeout is explicit and automatic cleanup on timeout.
/// - Looks synchronous (co_await), but uses rclcpp under the hood without threads.

#include <example_interfaces/action/fibonacci.hpp>
#include <icey/icey_async_await.hpp>

using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("icey_action_client_async_await_example");
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());

  // Create ICEY action client
  auto client = ctx->create_action_client<Fibonacci>("/fibonacci");

  // Periodically send a goal and co_await result
  ctx->create_timer_async(1s, [node, client](std::size_t) -> icey::Promise<void> {
    Fibonacci::Goal goal;
    goal.order = 7;

    // Await final result with 2s timeout; error string on timeout or interruption
    auto result = co_await client.send_goal(goal, 2s);
    if (result.has_error()) {
      RCLCPP_WARN(node->get_logger(), "Action error: %s", result.error().c_str());
      co_return;
    }

    const auto &wrapped = result.value();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node->get_logger(), "Fibonacci done. Size: %zu", wrapped.result->sequence.size());
    } else {
      RCLCPP_WARN(node->get_logger(), "Action finished with code %d",
                  static_cast<int>(wrapped.code));
    }
    co_return;
  });

  rclcpp::spin(node);
  return 0;
}
