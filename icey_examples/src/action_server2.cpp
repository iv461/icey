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

  auto client = ctx->create_action_client<Fibonacci>("/icey_test_fib_result_timeout");

  auto handle_goal = [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  auto handle_cancel = [](std::shared_ptr<ServerGoalHandleFibonacci>) {
    return rclcpp_action::CancelResponse::REJECT;
  };
  auto handle_accepted = [](std::shared_ptr<ServerGoalHandleFibonacci>) {
    // Do not send any result to force client result(timeout)
  };

  auto server = rclcpp_action::create_server<Fibonacci>(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_waitables_interface(),
      "/icey_test_fib_result_timeout", handle_goal, handle_cancel, handle_accepted);

  Fibonacci::Goal goal;
  goal.order = 3;
  auto gh_res = co_await client.send_goal(goal, 100ms, [](auto, auto) {});

  auto r = co_await gh_res.value()->result(60ms);
  if (r.has_value()) {
    std::cout << "Result code: " << int(r.value().code)
              << ", result: " << fmt::format("{}", r.value().result->sequence) << std::endl;
  }
  
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("icey_action_server_async_await_example");
  call(node);
  rclcpp::spin(node);
  return 0;
}
