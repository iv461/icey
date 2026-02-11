#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#define ICEY_CORO_DEBUG_PRINT

#include <example_interfaces/action/fibonacci.hpp>
#include <icey/icey_async_await.hpp>

#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "std_srvs/srv/set_bool.hpp"
using namespace std::chrono_literals;

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
using ServerGoalHandleFibonacci = icey::rclcpp_action::ServerGoalHandle<Fibonacci>;

using namespace icey::rclcpp_action;
using ExampleService = std_srvs::srv::SetBool;
using Request = ExampleService::Request;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("icey_action_server_async_await_example");
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());
  /// Create a service client for an upstream service that is actually capable of answering the
  /// request.
  auto upstream_service_client = ctx->create_client<ExampleService>("set_bool_service_upstream");

  //&auto action_client = ctx->create_action_client<Fibonacci>("/icey_test_fib_result_timeout");
  auto handle_goal = [&](const GoalUUID &,
                         std::shared_ptr<const Fibonacci::Goal>) -> icey::Promise<GoalResponse> {
    std::cout << "got goal request" << std::endl;
    /// Call the upstream service with 1s timeout asynchronously:
    auto request = std::make_shared<Request>();
    auto upstream_result = co_await upstream_service_client.call(request, 1s);

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

  auto server = ctx->create_action_server<Fibonacci>("/icey_server_async_test", handle_goal,
                                                     handle_cancel, handle_accepted);

  auto client = rclcpp_action::create_client<Fibonacci>(
      node->get_node_base_interface(), node->get_node_graph_interface(),
      node->get_node_logging_interface(), node->get_node_waitables_interface(),
      "/icey_server_async_test");

  Fibonacci::Goal goal;
  goal.order = 2;
  typename rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
  options.goal_response_callback = [](auto goal_handle) {
    std::cout << "options.goal_response_callback" << std::endl;
  };
  options.feedback_callback = [](auto goal_handle, auto feedback) {
    std::cout << "options.feedback_callback" << std::endl;
  };
  bool result_received = false;
  options.result_callback = [&](const auto &) {
    std::cout << "options.result_callback" << std::endl;
    result_received = true;
  };
  client->async_send_goal(goal, options);

  rclcpp::spin(node);
  return 0;
}
