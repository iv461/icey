/// Copyright © 2026 Ivo Ivanov
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// Action server example that demonstrates how callbacks of the action server can be coroutines. In
/// this case, the handle_goal-callback is a coroutine and calls a service. Once the service
/// response, the goal is accepted. The handle_cancel and handle_accepted could be coroutines as
/// well.

#include <example_interfaces/action/fibonacci.hpp>
#include <icey/icey_async_await.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>

using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = icey::rclcpp_action::ClientGoalHandle<Fibonacci>;
using ServerGoalHandleFibonacci = icey::rclcpp_action::ServerGoalHandle<Fibonacci>;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("icey_action_server_async_await_example");
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());

  using Upstream = std_srvs::srv::SetBool;
  auto upstream = ctx->create_client<Upstream>("set_bool_service1");

  auto handle_goal =
      [&](const rclcpp_action::GoalUUID &,
          std::shared_ptr<const Fibonacci::Goal>) -> icey::Promise<rclcpp_action::GoalResponse> {
    RCLCPP_INFO_STREAM(node->get_logger(), "Got goal request, sending upstream service request ..");

    /// This will just timeout. Note that you always need at lease one co_await inside a coroutine.
    auto upstream_result =
        co_await upstream.call(std::make_shared<std_srvs::srv::SetBool::Request>(), 3s);

    RCLCPP_INFO_STREAM(node->get_logger(), "Received upstream service result, accepting goal ..");
    co_return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_cancel =
      [](std::shared_ptr<ServerGoalHandleFibonacci>) -> rclcpp_action::CancelResponse {
    return rclcpp_action::CancelResponse::REJECT;
  };

  auto handle_accepted = [&](std::shared_ptr<ServerGoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Executing goal ..");
    std::this_thread::sleep_for(5s);
    RCLCPP_INFO_STREAM(node->get_logger(), "Finished executing goal.");
    goal_handle->succeed(std::make_shared<Fibonacci::Result>());
  };

  ctx->create_action_server<Fibonacci>("/icey_test_action_fibonacci", handle_goal, handle_cancel,
                                       handle_accepted);

  RCLCPP_INFO_STREAM(node->get_logger(), "Created action sever");
  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 8};
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  return 0;
}
