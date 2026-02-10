/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#define ICEY_PROMISE_LIFETIMES_DEBUG_PRINT

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include "example_interfaces/action/fibonacci.hpp"
#include "node_fixture.hpp"

using namespace std::chrono_literals;

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
using ServerGoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

struct ActionsAsyncAwait : TwoNodesFixture {
  bool async_completed{false};
};


TEST_F(ActionsAsyncAwait, ActionServerWithAsyncCallbacks) {
  const auto l = [this]() -> icey::Promise<void> {
    auto client = rclcpp_action::create_client<Fibonacci>(
        receiver_->get_node_base_interface(), receiver_->get_node_graph_interface(),
        receiver_->get_node_logging_interface(), receiver_->get_node_waitables_interface(),
        "/icey_8ufg23");

    /// Use coroutines as callbacks for the server
    auto handle_goal =
        [](const rclcpp_action::GoalUUID &,
           std::shared_ptr<const Fibonacci::Goal>) -> icey::Promise<rclcpp_action::GoalResponse> {
      co_return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
    auto handle_cancel = [](std::shared_ptr<ServerGoalHandleFibonacci>)
        -> icey::Promise<rclcpp_action::CancelResponse> {
      co_return rclcpp_action::CancelResponse::REJECT;
    };
    std::shared_ptr<ServerGoalHandleFibonacci> stored_gh;
    auto handle_accepted =
        [&stored_gh](std::shared_ptr<ServerGoalHandleFibonacci> gh) -> icey::Promise<void> {
      stored_gh = gh;
      co_return;
    };

    auto server = sender_->icey().create_action_server<Fibonacci>("/icey_8ufg23", handle_goal,
                                                                  handle_cancel, handle_accepted);

    Fibonacci::Goal goal;
    goal.order = 2;

    typename rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
    options.goal_response_callback = [](auto goal_handle) {};
    options.feedback_callback = [](auto goal_handle, auto feedback) {
      std::cout << "options.feedback_callback" << std::endl;
    };
    bool result_received = false;
    options.result_callback = [&](const auto &) { result_received = true; };
    client->async_send_goal(goal, options);

    EXPECT_TRUE(result_received);

    async_completed = true;
    co_return;
  };
  l();
  spin(1000ms);
  ASSERT_TRUE(async_completed);
}
