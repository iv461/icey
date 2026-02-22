/// Copyright © 2026 Ivo Ivanov
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// Action client example using ICEY async/await. It demonstrates an async/await API for sending the
/// goal and obtaining the result. Feedback messages are received via a callback because ROS does
/// not guarantee that feedback cannot be received before or after the result request
/// (https://github.com/ros2/rclcpp/issues/2782#issuecomment-2761123049). To prevent losing any
/// feedback messages, ICEY does not provide an async/await API for obtaining feedback and instead
/// uses a callback. See https://github.com/iv461/icey/pull/17 for an alternative API proposal once
/// this issue is fixed in rclcpp.

#include <example_interfaces/action/fibonacci.hpp>
#include <icey/icey_async_await.hpp>

using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
using ActionResult = typename GoalHandle::WrappedResult;

icey::Promise<void> call_action(std::shared_ptr<rclcpp::Node> node) {
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());
  auto client = ctx->create_action_client<Fibonacci>("/icey_test_action_fibonacci");

  Fibonacci::Goal goal;
  goal.order = 7;

  const auto on_feedback = [node](auto, auto) {
    RCLCPP_WARN(node->get_logger(), "Got some feedback");
  };

  RCLCPP_INFO_STREAM(node->get_logger(), "Sending goal.. ");
  // Request a goal with 2 seconds timeout, pass a callback for obtaining feedback messages
  icey::Result<icey::AsyncGoalHandle<Fibonacci>, std::string> maybe_goal_handle =
      co_await client.send_goal(goal, 5s, on_feedback);

  if (maybe_goal_handle.has_error()) {
    RCLCPP_WARN_STREAM(node->get_logger(),
                       "Goal request was rejected: " << maybe_goal_handle.error());
    co_return;
  }
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "Goal request was accepted, starting to wait for the result..");
  icey::AsyncGoalHandle<Fibonacci> goal_handle = maybe_goal_handle.value();
  /// Now wait for the result of the action for 20 seconds.
  icey::Result<ActionResult, std::string> maybe_result = co_await goal_handle.result(20s);
  if (maybe_result.has_error()) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Action failed: " << maybe_result.error());

  } else {
    const ActionResult &result = maybe_result.value();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node->get_logger(), "Fibonacci done. Size: %zu", result.result->sequence.size());
    } else {
      RCLCPP_WARN(node->get_logger(), "Action finished with code %d",
                  static_cast<int>(result.code));
    }
  }
  co_return;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("icey_action_client_async_await_example");
  (void)call_action(node);
  rclcpp::spin(node);
  return 0;
}
