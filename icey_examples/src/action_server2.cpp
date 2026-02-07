#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <example_interfaces/action/fibonacci.hpp>
#include <icey/icey_async_await.hpp>
using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
using ServerGoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

icey::Promise<void> call(std::shared_ptr<rclcpp::Node> node) {
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());

  //&auto action_client = ctx->create_action_client<Fibonacci>("/icey_test_fib_result_timeout");
  auto action_client =
      rclcpp_action::create_client<Fibonacci>(node, "/icey_test_fib_result_timeout");

  auto handle_goal = [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  auto handle_cancel = [](std::shared_ptr<ServerGoalHandleFibonacci>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  auto handle_accepted = [](std::shared_ptr<ServerGoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(rclcpp::get_logger("server"), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto &sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(rclcpp::get_logger("server"), "Publish Feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Succeeded");
    }
  };

  auto server = rclcpp_action::create_server<Fibonacci>(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_waitables_interface(),
      "/icey_test_fib_result_timeout", handle_goal, handle_cancel, handle_accepted);

  // Populate a goal
  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 10;

  auto goal_handle_future = action_client->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
    co_return;
  }

  rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    co_return;
  }

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
    co_return;
  }

  rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      co_return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      co_return;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      co_return;
  }

  RCLCPP_INFO(node->get_logger(), "result received");
  std::cout << fmt::format("{}", wrapped_result.result->sequence) << std::endl;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("icey_action_server_async_await_example");
  call(node);
  rclcpp::spin(node);
  return 0;
}
