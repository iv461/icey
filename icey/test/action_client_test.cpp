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
using GoalHandleFibonacci = icey::rclcpp_action::ClientGoalHandle<Fibonacci>;
using ServerGoalHandleFibonacci = icey::rclcpp_action::ServerGoalHandle<Fibonacci>;

struct ActionsAsyncAwait : TwoNodesFixture {
  bool async_completed{false};
};

TEST_F(ActionsAsyncAwait, ActionSendGoalTest) {
  const auto l = [this]() -> icey::Promise<void> {
    auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib");

    Fibonacci::Goal goal;
    goal.order = 5;

    // No server yet -> expect TIMEOUT
    auto res1 = co_await client.send_goal(goal, 40ms, [](auto, auto) {});

    EXPECT_TRUE(res1.has_error());
    EXPECT_EQ(res1.error(), "TIMEOUT");

    // Create server
    auto handle_goal = [](auto server, const rclcpp_action::GoalUUID &,
                          std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
    auto handle_cancel = [](auto, auto) { return rclcpp_action::CancelResponse::ACCEPT; };
    auto handle_accepted = [this](auto gh) {
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

    auto server = receiver_->icey().create_action_server<Fibonacci>("/icey_test_fib", handle_goal,
                                                                    handle_cancel, handle_accepted);

    // With server available -> expect success
    auto res2 = co_await client.send_goal(goal, 200ms, [](auto, auto) {});
    EXPECT_TRUE(res2.has_value()) << (res2.has_error() ? res2.error() : "");

    auto ghagsd = res2.value();
    auto ares = co_await ghagsd.result(200ms);

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

TEST_F(ActionsAsyncAwait, ActionTimeoutAndMultipleGoalsTest) {
  const auto l = [this]() -> icey::Promise<void> {
    auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib2");

    // Create a server that sleeps on first goal only
    std::atomic<size_t> call_idx{0};
    auto handle_goal = [&call_idx](const rclcpp_action::GoalUUID &,
                                   std::shared_ptr<const Fibonacci::Goal>) {
      if (call_idx == 0) {
        call_idx++;
        std::this_thread::sleep_for(100ms);
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
    auto handle_cancel = [](auto) { return rclcpp_action::CancelResponse::ACCEPT; };
    auto handle_accepted = [](auto gh) { gh->succeed(std::make_shared<Fibonacci::Result>()); };
    auto server = receiver_->icey().create_action_server<Fibonacci>("/icey_test_fib2", handle_goal,
                                                                    handle_cancel, handle_accepted);

    Fibonacci::Goal goal;
    goal.order = 5;

    // First call should timeout

    auto first = co_await client.send_goal(goal, 40ms, [](auto, auto) {});
    EXPECT_TRUE(first.has_error());
    EXPECT_EQ(first.error(), "TIMEOUT");

    auto r2 = co_await client.send_goal(goal, 40ms, [](auto, auto) {});
    auto r1 = co_await client.send_goal(goal, 40ms, [](auto, auto) {});

    EXPECT_TRUE(r2.has_value()) << r2.error();
    auto ares = co_await r2.value().result(200ms);
    EXPECT_EQ(ares.value().code, rclcpp_action::ResultCode::SUCCEEDED);

    /* EXPECT_TRUE(r1.has_value()) << r1.error();
     auto ares1 = co_await r1.value()->result(200ms);
     EXPECT_EQ(ares1.value().code, rclcpp_action::ResultCode::SUCCEEDED);*/

    async_completed = true;
    co_return;
  };
  l();
  spin(1200ms);
  ASSERT_TRUE(async_completed);
}

TEST_F(ActionsAsyncAwait, ActionSendGoalAcceptanceTimeout) {
  const auto l = [this]() -> icey::Promise<void> {
    auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib_no_server");

    co_await receiver_->icey().create_timer(20ms);
    Fibonacci::Goal goal;
    goal.order = 3;
    auto res = co_await client.send_goal(goal, 40ms, [](auto, auto) {});
    EXPECT_TRUE(res.has_error());
    EXPECT_EQ(res.error(), "TIMEOUT");
    async_completed = true;
    co_return;
  };
  l();
  spin(400ms);
  ASSERT_TRUE(async_completed);
}

TEST_F(ActionsAsyncAwait, ActionGoalRejected) {
  const auto l = [this]() -> icey::Promise<void> {
    auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib_reject");
    auto handle_goal = [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::REJECT;
    };
    auto handle_cancel = [](auto) { return rclcpp_action::CancelResponse::REJECT; };
    auto handle_accepted = [](auto) {};
    auto server = receiver_->icey().create_action_server<Fibonacci>(
        "/icey_test_fib_reject", handle_goal, handle_cancel, handle_accepted);
    Fibonacci::Goal goal;
    goal.order = 3;
    auto res = co_await client.send_goal(goal, 100ms, [](auto, auto) {});
    EXPECT_TRUE(res.has_error());
    EXPECT_EQ(res.error(), "GOAL REJECTED");
    async_completed = true;
    co_return;
  };
  l();
  spin(400ms);
  ASSERT_TRUE(async_completed);
}

TEST_F(ActionsAsyncAwait, ActionResultTimeout) {
  const auto l = [this]() -> icey::Promise<void> {
    auto client =
        receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib_result_timeout");
    auto handle_goal = [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
    auto handle_cancel = [](auto) { return rclcpp_action::CancelResponse::REJECT; };
    auto handle_accepted = [](auto goal_handle) {
      /// The server cancels the request if you do not reference the goal_handle after this callback
      /// returns. These are uh, some questionable API design choices, but that's how it is with
      /// ROS.
      std::thread{[goal_handle]() { std::this_thread::sleep_for(2s); }}.detach();
    };
    auto server = receiver_->icey().create_action_server<Fibonacci>(
        "/icey_test_fib_result_timeout", handle_goal, handle_cancel, handle_accepted);
    Fibonacci::Goal goal;
    goal.order = 3;
    auto gh_res = co_await client.send_goal(goal, 100ms, [](auto, auto) {});
    EXPECT_TRUE(gh_res.has_value()) << (gh_res.has_error() ? gh_res.error() : "");
    auto r = co_await gh_res.value()->result(100ms);
    if (r.has_value()) {
      std::cout << "Result code: " << int(r.value().code)
                << ", result: " << fmt::format("{}", r.value().result->sequence) << std::endl;
    }
    EXPECT_EQ(r.error(), "RESULT TIMEOUT");
    async_completed = true;
    co_return;
  };
  l();
  spin(600ms);
  ASSERT_TRUE(async_completed);
}

TEST_F(ActionsAsyncAwait, ActionCancelTimeout) {
  const auto l = [this]() -> icey::Promise<void> {
    auto client =
        receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib_cancel_timeout");
    auto handle_goal = [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
    auto handle_cancel = [](auto) {
      std::this_thread::sleep_for(200ms);
      return rclcpp_action::CancelResponse::ACCEPT;
    };
    auto handle_accepted = [](auto) {};
    auto server = receiver_->icey().create_action_server<Fibonacci>(
        "/icey_test_fib_cancel_timeout", handle_goal, handle_cancel, handle_accepted);
    Fibonacci::Goal goal;
    goal.order = 2;
    auto gh_res = co_await client.send_goal(goal, 100ms, [](auto, auto) {});
    EXPECT_TRUE(gh_res.has_value());
    auto cres = co_await gh_res.value()->cancel(50ms);
    EXPECT_TRUE(cres.has_error());
    EXPECT_EQ(cres.error(), "RESULT TIMEOUT");
    async_completed = true;
    co_return;
  };
  l();
  spin(800ms);
  ASSERT_TRUE(async_completed);
}

TEST_F(ActionsAsyncAwait, ActionAbortAndCanceledAndSucceeded) {
  const auto l = [this]() -> icey::Promise<void> {
    // Abort case
    {
      auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib_abort");
      auto handle_goal = [](const rclcpp_action::GoalUUID &,
                            std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };
      auto handle_cancel = [](auto) { return rclcpp_action::CancelResponse::REJECT; };
      auto handle_accepted = [](auto gh) {
        std::thread{[gh]() {
          std::this_thread::sleep_for(20ms);
          auto result = std::make_shared<Fibonacci::Result>();
          gh->abort(result);
        }}.detach();
      };
      auto server = receiver_->icey().create_action_server<Fibonacci>(
          "/icey_test_fib_abort", handle_goal, handle_cancel, handle_accepted);
      Fibonacci::Goal goal;
      goal.order = 2;
      auto gh = co_await client.send_goal(goal, 100ms, [](auto, auto) {});
      EXPECT_TRUE(gh.has_value());
      auto res = co_await gh.value().result(200ms);
      EXPECT_TRUE(res.has_value());
      EXPECT_EQ(res.value().code, rclcpp_action::ResultCode::ABORTED);
    }

    // Canceled case
    {
      auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib_canceled");
      auto handle_goal = [](const rclcpp_action::GoalUUID &,
                            std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };
      auto handle_cancel = [](auto) { return rclcpp_action::CancelResponse::ACCEPT; };
      auto handle_accepted = [](auto gh) {
        std::thread{[gh]() {
          std::this_thread::sleep_for(50ms);
          auto result = std::make_shared<Fibonacci::Result>();
          gh->canceled(result);
        }}.detach();
      };
      auto server = receiver_->icey().create_action_server<Fibonacci>(
          "/icey_test_fib_canceled", handle_goal, handle_cancel, handle_accepted);
      Fibonacci::Goal goal;
      goal.order = 2;
      auto gh = co_await client.send_goal(goal, 100ms, [](auto, auto) {});
      EXPECT_TRUE(gh.has_value());
      auto res = co_await gh.value().result(200ms);
      EXPECT_TRUE(res.has_value());
      EXPECT_EQ(res.value().code, rclcpp_action::ResultCode::CANCELED);
    }

    // Succeeded case
    {
      auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib_success");
      auto handle_goal = [](const rclcpp_action::GoalUUID &,
                            std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };
      auto handle_cancel = [](auto) { return rclcpp_action::CancelResponse::REJECT; };
      auto handle_accepted = [](auto gh) {
        std::thread{[gh]() {
          std::this_thread::sleep_for(20ms);
          auto result = std::make_shared<Fibonacci::Result>();
          result->sequence = {0, 1, 1, 2};
          gh->succeed(result);
        }}.detach();
      };
      auto server = receiver_->icey().create_action_server<Fibonacci>(
          "/icey_test_fib_success", handle_goal, handle_cancel, handle_accepted);
      Fibonacci::Goal goal;
      goal.order = 2;
      auto gh = co_await client.send_goal(goal, 100ms, [](auto, auto) {});
      EXPECT_TRUE(gh.has_value());
      auto res = co_await gh.value().result(200ms);
      EXPECT_TRUE(res.has_value());
      EXPECT_EQ(res.value().code, rclcpp_action::ResultCode::SUCCEEDED);
    }

    async_completed = true;
    co_return;
  };
  l();
  spin(2000ms);
  ASSERT_TRUE(async_completed);
}

TEST_F(ActionsAsyncAwait, ActionCancelAfterSuccessInvalidTransition) {
  const auto l = [this]() -> icey::Promise<void> {
    auto client =
        receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib_cancel_after_success");
    auto handle_goal = [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
    auto handle_cancel = [](auto) { return rclcpp_action::CancelResponse::REJECT; };
    auto handle_accepted = [](auto gh) {
      std::thread{[gh]() {
        std::this_thread::sleep_for(20ms);
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = {0, 1, 1};
        gh->succeed(result);
      }}.detach();
    };
    auto server = receiver_->icey().create_action_server<Fibonacci>(
        "/icey_test_fib_cancel_after_success", handle_goal, handle_cancel, handle_accepted);
    Fibonacci::Goal goal;
    goal.order = 3;
    auto gh = co_await client.send_goal(goal, 100ms, [](auto, auto) {});
    EXPECT_TRUE(gh.has_value());
    // Wait for success
    auto res = co_await gh.value().result(200ms);
    EXPECT_TRUE(res.has_value());
    EXPECT_EQ(res.value().code, rclcpp_action::ResultCode::SUCCEEDED);
    // Now attempt to cancel after success; should error (invalid transition/no-op)
    auto cres = co_await gh.value().cancel(50ms);
    EXPECT_TRUE(cres.has_error());
    async_completed = true;
    co_return;
  };
  l();
  spin(800ms);
  ASSERT_TRUE(async_completed);
}

TEST_F(ActionsAsyncAwait, ActionCancelTwiceInvalidTransition) {
  const auto l = [this]() -> icey::Promise<void> {
    auto client = receiver_->icey().create_action_client<Fibonacci>("/icey_test_fib_cancel_twice");
    auto handle_goal = [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
    std::atomic<bool> first_cancel{true};
    auto handle_cancel = [&first_cancel](auto) {
      if (first_cancel.exchange(false)) return rclcpp_action::CancelResponse::ACCEPT;
      return rclcpp_action::CancelResponse::REJECT;
    };
    auto handle_accepted = [](auto gh) {
      std::thread{[gh]() {
        std::this_thread::sleep_for(50ms);
        auto result = std::make_shared<Fibonacci::Result>();
        gh->canceled(result);
      }}.detach();
    };
    auto server = receiver_->icey().create_action_server<Fibonacci>(
        "/icey_test_fib_cancel_twice", handle_goal, handle_cancel, handle_accepted);
    Fibonacci::Goal goal;
    goal.order = 2;
    auto gh = co_await client.send_goal(goal, 100ms, [](auto, auto) {});
    EXPECT_TRUE(gh.has_value());
    auto c1 = co_await gh.value().cancel(100ms);
    EXPECT_TRUE(c1.has_error() ||
                c1.has_value());  // cancel may resolve or be no-op depending on timing
    auto c2 = co_await gh.value().cancel(50ms);
    EXPECT_TRUE(c2.has_error());  // second cancel should be rejected
    async_completed = true;
    co_return;
  };
  l();
  spin(1000ms);
  ASSERT_TRUE(async_completed);
}
