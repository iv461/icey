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
#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("icey_action_server_async_await_example");
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());

  using Upstream = std_srvs::srv::SetBool;
  auto upstream = ctx->create_client<Upstream>("set_bool_service1");

  // Create server with async execute callback that calls an upstream service
  ctx->create_action_server<Fibonacci>(
      "/icey_test_action_fibonacci",
      [ctx, node, upstream](
          std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> gh) -> icey::Promise<void> {
        // Call an upstream service first (demonstrates co_await inside action server)
        /*
        auto req = std::make_shared<Upstream::Request>();
        req->data = true;
        auto upstream_result = co_await upstream.call(req, 1s);
        if (upstream_result.has_error()) {
          RCLCPP_WARN(node->get_logger(), "Upstream service error: %s",
          upstream_result.error().c_str());
        } else {
          RCLCPP_INFO(node->get_logger(), "Upstream response: %s",
        upstream_result.value()->success ? "true" : "false");
      }
      */

        // Read goal and compute sequence gradually, publishing feedback on a timer
        auto goal = gh->get_goal();
        RCLCPP_INFO_STREAM(node->get_logger(), "Received goal");
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
          RCLCPP_INFO_STREAM(node->get_logger(), "Sending feedback ..");

          gh->publish_feedback(std::make_shared<Fibonacci::Feedback>(fb));
        }

        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = seq;
        RCLCPP_INFO_STREAM(node->get_logger(), "Sending succeed");
        gh->succeed(result);
        co_return;
      });

  RCLCPP_INFO_STREAM(node->get_logger(), "Created action sever");
  rclcpp::spin(node);
  return 0;
}
