/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#include <fmt/core.h>
#include <fmt/ranges.h>

#include <std_srvs/srv/set_bool.hpp>

#include "example_interfaces/action/fibonacci.hpp"
#include "node_fixture.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = icey::rclcpp_action::ClientGoalHandle<Fibonacci>;
using ServerGoalHandleFibonacci = icey::rclcpp_action::ServerGoalHandle<Fibonacci>;

using namespace icey::rclcpp_action;

struct ActionsAsyncAwait : TwoNodesFixture {
  bool async_completed{false};
};

TEST_F(ActionsAsyncAwait, ActionServerWithAsyncCallbacks) {
  /// Use coroutines as callbacks for the server
  auto upstream_service_client =
      receiver_->icey().create_client<std_srvs::srv::SetBool>("set_bool_service_upstream");
  auto handle_goal = [&](const GoalUUID &,
                         std::shared_ptr<const Fibonacci::Goal>) -> icey::Promise<GoalResponse> {
    std::cout << "got goal request" << std::endl;

    /// This will just timeout. Note that you always need at lease one co_await inside a coroutine
    auto upstream_result = co_await upstream_service_client.call(
        std::make_shared<std_srvs::srv::SetBool::Request>(), 1s);

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

  auto server = receiver_->icey().create_action_server<Fibonacci>(
      "/icey_server_async_test", handle_goal, handle_cancel, handle_accepted);

  auto client = icey::rclcpp_action::create_client<Fibonacci>(
      receiver_->get_node_base_interface(), receiver_->get_node_graph_interface(),
      receiver_->get_node_logging_interface(), receiver_->get_node_waitables_interface(),
      "/icey_server_async_test");
  Fibonacci::Goal goal;
  goal.order = 2;
  typename icey::rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
  options.goal_response_callback = [](auto) {
    std::cout << "options.goal_response_callback" << std::endl;
  };
  options.feedback_callback = [](auto, auto) {
    std::cout << "options.feedback_callback" << std::endl;
  };
  bool result_received = false;
  options.result_callback = [&](const auto &) {
    std::cout << "options.result_callback" << std::endl;
    result_received = true;
  };
  client->async_send_goal(goal, options);
  spin(1000ms);
  EXPECT_TRUE(result_received);
}
