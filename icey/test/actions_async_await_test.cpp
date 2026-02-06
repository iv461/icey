/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#include "example_interfaces/action/fibonacci.hpp"
#include "node_fixture.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
using ServerGoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

struct ActionsAsyncAwaitTwoNodeTest : TwoNodesFixture {
  bool async_completed{false};
};

TEST_F(ActionsAsyncAwaitTwoNodeTest, ActionSendGoalTest) {
  const auto l = [this]() -> icey::Promise<void> {
    auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib");

    Fibonacci::Goal goal;
    goal.order = 5;

    // No server yet -> expect TIMEOUT
    auto res1 = co_await client.send_goal(goal, 40ms, [](auto gh, auto fb) {});
    std::cout << "got here" << std::endl;
    EXPECT_TRUE(res1.has_error());
    EXPECT_EQ(res1.error(), "TIMEOUT");

    // Create server
    auto handle_goal = [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
    auto handle_cancel = [](std::shared_ptr<ServerGoalHandleFibonacci>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    };
    auto handle_accepted = [this](std::shared_ptr<ServerGoalHandleFibonacci> gh) {
      std::thread{[gh]() {
        auto goal = gh->get_goal();
        std::vector<int32_t> seq;
        seq.reserve(goal->order);
        int32_t a = 0, b = 1;
        for (int i = 0; i < static_cast<int>(goal->order); ++i) {
          seq.push_back(a);
          int32_t next = a + b;
          a = b;
          b = next;
          // publish minimal feedback
          Fibonacci::Feedback feedback;
          feedback.sequence = seq;
          gh->publish_feedback(std::make_shared<Fibonacci::Feedback>(feedback));
        }
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = seq;
        gh->succeed(result);
      }}.detach();
    };

    auto server = rclcpp_action::create_server<Fibonacci>(
        sender_->get_node_base_interface(), sender_->get_node_clock_interface(),
        sender_->get_node_logging_interface(), sender_->get_node_waitables_interface(),
        "/icey_test_fib", handle_goal, handle_cancel, handle_accepted);

    // With server available -> expect success
    auto res2 = co_await client.send_goal(goal, 200ms, [](auto goal_handle, auto feedback) {});
    std::cout << "got 2here" << std::endl;
    EXPECT_TRUE(res2.has_value()) << (res2.has_error() ? res2.error() : "");
    std::cout << "got here23, goal handle status: " << int(res2.value().get_goal_handle()->get_status()) << std::endl;
    auto ares = co_await res2.value().result(200ms);
    std::cout << "got r" << std::endl;
    EXPECT_EQ(ares.value().code, rclcpp_action::ResultCode::SUCCEEDED);
    auto seq = ares.value().result->sequence;
    std::vector<int32_t> expected{0, 1, 1, 2, 3};
    EXPECT_EQ(seq, expected);

    async_completed = true;
    co_return;
  };
  l();
  spin(1000ms);
  ASSERT_TRUE(async_completed);
}

TEST_F(ActionsAsyncAwaitTwoNodeTest, ActionTimeoutAndMultipleGoalsTest) {
  const auto l = [this]() -> icey::Stream<int> {
    auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib2");

    // Create a server that sleeps on first goal only
    std::atomic<size_t> call_idx{0};
    auto handle_goal = [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
    auto handle_cancel = [](std::shared_ptr<ServerGoalHandleFibonacci>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    };
    auto handle_accepted = [&call_idx](std::shared_ptr<ServerGoalHandleFibonacci> gh) {
      std::thread{[gh, &call_idx]() {
        auto idx = call_idx.fetch_add(1);
        if (idx == 0) std::this_thread::sleep_for(100ms);
        auto goal = gh->get_goal();
        std::vector<int32_t> seq;
        int32_t a = 0, b = 1;
        for (int i = 0; i < static_cast<int>(goal->order); ++i) {
          seq.push_back(a);
          int32_t next = a + b;
          a = b;
          b = next;
        }
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = seq;
        gh->succeed(result);
      }}.detach();
    };

    auto server = rclcpp_action::create_server<Fibonacci>(
        sender_->get_node_base_interface(), sender_->get_node_clock_interface(),
        sender_->get_node_logging_interface(), sender_->get_node_waitables_interface(),
        "/icey_test_fib2", handle_goal, handle_cancel, handle_accepted);

    Fibonacci::Goal goal;
    goal.order = 5;

    // First call should timeout
    auto first = co_await client.send_goal(goal, 40ms, [](auto gh, auto fb) {});
    EXPECT_TRUE(first.has_error());
    EXPECT_EQ(first.error(), "TIMEOUT");

    // Then fire two parallel requests and await both
    auto r2 = co_await client.send_goal(goal, 40ms, [](auto gh, auto fb) {});
    auto r1 = co_await client.send_goal(goal, 40ms, [](auto gh, auto fb) {});

    EXPECT_TRUE(r2.has_value()) << (r2.has_error() ? r2.error() : "");
    auto ares = co_await r2.value().result(200ms);
    EXPECT_EQ(ares.value().code, rclcpp_action::ResultCode::SUCCEEDED);

    EXPECT_TRUE(r1.has_value()) << (r1.has_error() ? r1.error() : "");
    auto ares1 = co_await r1.value().result(200ms);
    EXPECT_EQ(ares1.value().code, rclcpp_action::ResultCode::SUCCEEDED);

    async_completed = true;
    co_return 0;
  };
  l();
  spin(1200ms);
  ASSERT_TRUE(async_completed);
}
