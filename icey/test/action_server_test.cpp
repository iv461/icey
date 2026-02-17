/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#include <fmt/core.h>
#include <fmt/ranges.h>

#include <atomic>
#include <std_srvs/srv/set_bool.hpp>

#include "example_interfaces/action/fibonacci.hpp"
#include "node_fixture.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = icey::rclcpp_action::ClientGoalHandle<Fibonacci>;
using ServerGoalHandleFibonacci = icey::rclcpp_action::ServerGoalHandle<Fibonacci>;

struct ActionsAsyncAwait : TwoNodesFixture {
  bool async_completed{false};
};

TEST_F(ActionsAsyncAwait, ActionServerWithAsyncCallbacks) {
  /// Use coroutines as callbacks for the server
  auto upstream_service_client =
      receiver_->icey().create_client<std_srvs::srv::SetBool>("set_bool_service_upstream");

  auto handle_goal =
      [&](const GoalUUID &,
          std::shared_ptr<const Fibonacci::Goal>) -> icey::Promise<rclcpp_action::GoalResponse> {
    std::cout << "got goal request" << std::endl;
    /// This will just timeout. Note that you always need at least one co_await inside a coroutine
    auto upstream_result = co_await upstream_service_client.call(
        std::make_shared<std_srvs::srv::SetBool::Request>(), 50ms);

    co_return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_cancel = [](std::shared_ptr<ServerGoalHandleFibonacci>)
      -> icey::Promise<rclcpp_action::CancelResponse> {
    std::cout << "got reject request" << std::endl;
    co_return rclcpp_action::CancelResponse::REJECT;
  };

  std::shared_ptr<ServerGoalHandleFibonacci> stored_gh;
  auto handle_accepted = [&stored_gh](std::shared_ptr<ServerGoalHandleFibonacci> gh) {
    std::cout << "got request accepted" << std::endl;
    stored_gh = gh;
    auto result = std::make_shared<Fibonacci::Result>();
    gh->succeed(result);
  };

  auto server = receiver_->icey().create_action_server<Fibonacci>(
      "/icey_server_async_test", handle_goal, handle_cancel, handle_accepted);

  auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_server_async_test");

  const auto l = [this, client]() -> icey::Promise<void> {
    Fibonacci::Goal goal;
    goal.order = 2;

    auto res1 = co_await client.send_goal(
        goal, 200ms, [](auto, auto) { std::cout << "Got feedback" << std::endl; });

    EXPECT_FALSE(res1.has_error());
    if (res1.has_error())
      std::cout << "Goal error: " << res1.error() << std::endl;
    else
      std::cout << "Got goal" << std::endl;

    auto goal_handle = res1.value();
    auto ares = co_await goal_handle.result(200ms);
    if (ares.has_error())
      std::cout << "Goal result error: " << ares.error() << std::endl;
    else
      std::cout << "Goal result success: " << int(ares.value().code) << std::endl;
    EXPECT_FALSE(ares.has_error());
    EXPECT_EQ(ares.value().code, rclcpp_action::ResultCode::SUCCEEDED);
    async_completed = true;
    co_return;
  };
  l();
  spin(1000ms);
  ASSERT_TRUE(async_completed);
}

TEST_F(ActionsAsyncAwait, ActionServerWithSyncCallbacks) {
  std::atomic<bool> goal_called{false};
  std::atomic<bool> accepted_called{false};
  std::atomic<bool> cancel_called{false};

  auto handle_goal = [&goal_called](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
    goal_called.store(true);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_cancel = [&cancel_called](std::shared_ptr<ServerGoalHandleFibonacci>) {
    cancel_called.store(true);
    return rclcpp_action::CancelResponse::REJECT;
  };

  auto handle_accepted = [&accepted_called](std::shared_ptr<ServerGoalHandleFibonacci> gh) {
    accepted_called.store(true);
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = {0, 1, 1, 2};
    gh->succeed(result);
  };

  auto server = receiver_->icey().create_action_server<Fibonacci>(
      "/icey_server_sync_callbacks_test", handle_goal, handle_cancel, handle_accepted);

  auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_server_sync_callbacks_test");

  const auto l = [this, client, &goal_called, &accepted_called,
                  &cancel_called]() -> icey::Promise<void> {
    Fibonacci::Goal goal;
    goal.order = 4;

    auto send_res = co_await client.send_goal(goal, 200ms, [](auto, auto) {});
    EXPECT_TRUE(send_res.has_value());

    auto result = co_await send_res.value().result(200ms);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result.value().code, rclcpp_action::ResultCode::SUCCEEDED);

    EXPECT_TRUE(goal_called.load());
    EXPECT_TRUE(accepted_called.load());
    EXPECT_FALSE(cancel_called.load());
    async_completed = true;
    co_return;
  };

  l();
  spin(1000ms);
  ASSERT_TRUE(async_completed);
}

TEST_F(ActionsAsyncAwait, ActionServerWithAsyncGoalAndSyncCancelCallbacks) {
  std::atomic<bool> goal_called{false};
  std::atomic<bool> accepted_called{false};
  std::atomic<bool> cancel_called{false};
  auto handle_goal = [this, &goal_called](const GoalUUID &,
                                         std::shared_ptr<const Fibonacci::Goal>)
      -> icey::Promise<rclcpp_action::GoalResponse> {
    goal_called.store(true);
    co_await receiver_->icey().create_timer(2ms);
    co_return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_cancel = [&cancel_called](std::shared_ptr<ServerGoalHandleFibonacci>) {
    cancel_called.store(true);
    return rclcpp_action::CancelResponse::REJECT;
  };

  auto handle_accepted = [&accepted_called](std::shared_ptr<ServerGoalHandleFibonacci> gh) {
    accepted_called.store(true);
    auto result = std::make_shared<Fibonacci::Result>();
    gh->succeed(result);
  };

  auto server = receiver_->icey().create_action_server<Fibonacci>(
      "/icey_server_mixed_goal_async_test", handle_goal, handle_cancel, handle_accepted);

  auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_server_mixed_goal_async_test");

  const auto l = [this, client, &goal_called, &accepted_called,
                  &cancel_called]() -> icey::Promise<void> {
    Fibonacci::Goal goal;
    goal.order = 3;

    auto send_res = co_await client.send_goal(goal, 250ms, [](auto, auto) {});
    EXPECT_TRUE(send_res.has_value());

    auto result = co_await send_res.value().result(250ms);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result.value().code, rclcpp_action::ResultCode::SUCCEEDED);

    EXPECT_TRUE(goal_called.load());
    EXPECT_TRUE(accepted_called.load());
    EXPECT_FALSE(cancel_called.load());
    async_completed = true;
    co_return;
  };

  l();
  spin(1200ms);
  ASSERT_TRUE(async_completed);
}

TEST_F(ActionsAsyncAwait, ActionServerWithSyncGoalAndAsyncCancelCallbacks) {
  std::atomic<bool> goal_called{false};
  std::atomic<bool> accepted_called{false};
  std::atomic<bool> cancel_called{false};
  auto handle_goal = [&goal_called](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
    goal_called.store(true);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_cancel = [this, &cancel_called](std::shared_ptr<ServerGoalHandleFibonacci>)
      -> icey::Promise<rclcpp_action::CancelResponse> {
    cancel_called.store(true);
    co_await receiver_->icey().create_timer(2ms);
    co_return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_accepted = [&accepted_called](std::shared_ptr<ServerGoalHandleFibonacci> gh) {
    accepted_called.store(true);
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = {0, 1, 1};
    gh->succeed(result);
  };

  auto server = receiver_->icey().create_action_server<Fibonacci>(
      "/icey_server_mixed_cancel_async_test", handle_goal, handle_cancel, handle_accepted);

  auto client =
      receiver_->icey().create_action_client<Fibonacci>("/icey_server_mixed_cancel_async_test");

  const auto l = [this, client, &goal_called, &accepted_called,
                  &cancel_called]() -> icey::Promise<void> {
    Fibonacci::Goal goal;
    goal.order = 3;

    auto send_res = co_await client.send_goal(goal, 200ms, [](auto, auto) {});
    EXPECT_TRUE(send_res.has_value());

    auto result = co_await send_res.value().result(300ms);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result.value().code, rclcpp_action::ResultCode::SUCCEEDED);

    EXPECT_TRUE(goal_called.load());
    EXPECT_TRUE(accepted_called.load());
    EXPECT_FALSE(cancel_called.load());
    async_completed = true;
    co_return;
  };

  l();
  spin(1500ms);
  ASSERT_TRUE(async_completed);
}
