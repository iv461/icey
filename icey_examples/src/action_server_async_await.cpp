/// Copyright © 2026 Technische Hochschule Augsburg
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
using GoalHandleFibonacci = icey::rclcpp_action::ClientGoalHandle<Fibonacci>;
using ServerGoalHandleFibonacci = icey::rclcpp_action::ServerGoalHandle<Fibonacci>;
using namespace icey::rclcpp_action;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("icey_action_server_async_await_example");
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());

  using Upstream = std_srvs::srv::SetBool;
  auto upstream = ctx->create_client<Upstream>("set_bool_service1");

  auto handle_goal = [&](const GoalUUID &,
                         std::shared_ptr<const Fibonacci::Goal>) -> icey::Promise<GoalResponse> {
    std::cout << "got goal request" << std::endl;

    /// This will just timeout. Note that you always need at lease one co_await inside a coroutine
    auto upstream_result =
        co_await upstream.call(std::make_shared<std_srvs::srv::SetBool::Request>(), 1s);

    co_return GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_cancel =
      [](std::shared_ptr<ServerGoalHandleFibonacci>) -> icey::Promise<CancelResponse> {
    std::cout << "got reject request" << std::endl;
    co_return CancelResponse::REJECT;
  };

  std::shared_ptr<ServerGoalHandleFibonacci> stored_gh;
  auto handle_accepted =
      [&stored_gh](std::shared_ptr<ServerGoalHandleFibonacci> gh) -> icey::Promise<void> {
    std::cout << "got  request accepted" << std::endl;
    stored_gh = gh;
    co_return;
  };

  ctx->create_action_server<Fibonacci>("/icey_server_async_test", handle_goal, handle_cancel,
                                       handle_accepted);

  RCLCPP_INFO_STREAM(node->get_logger(), "Created action sever");
  rclcpp::spin(node);
  return 0;
}
