/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// Action server example using ICEY coroutine execute callback.
///
/// Key differences vs regular rclcpp_action API:
/// - Provide a single execute callback which can be a coroutine (impl::Promise<void>).
/// - You don’t need to wire goal/feedback/result lambdas separately unless you want to customize.
/// - Inside the execute callback you can co_await timers/services, making async flows explicit.

#include <example_interfaces/action/fibonacci.hpp>
#include <icey/icey_async_await.hpp>

using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("icey_action_server_async_await_example");
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());

  // Create server with async execute callback
  auto server = ctx->create_action_server<Fibonacci>(
      "/fibonacci",
      [ctx, node](
          std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> gh) -> icey::Promise<void> {
        // Read goal and compute sequence gradually, publishing feedback on a timer
        auto goal = gh->get_goal();
        int32_t a = 0, b = 1;
        std::vector<int32_t> seq;
        seq.reserve(goal->order);

        for (int i = 0; i < static_cast<int>(goal->order); ++i) {
          if (gh->is_canceling()) {
            auto res = std::make_shared<Fibonacci::Result>();
            res->sequence = seq;
            gh->canceled(res);
            co_return;
          }

          seq.push_back(a);
          int32_t next = a + b;
          a = b;
          b = next;

          Fibonacci::Feedback fb;
          fb.sequence = seq;
          gh->publish_feedback(std::make_shared<Fibonacci::Feedback>(fb));
        }

        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = seq;
        gh->succeed(result);
        co_return;
      });

  (void)server;
  rclcpp::spin(node);
  return 0;
}
