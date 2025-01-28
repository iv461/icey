/// This example shows how by using async/await we achieve a synchronously-looking service call.
/// Under the hood, everything is asynchronous, we actually call async_send_request, but promises
/// combined with await-syntax make the code look synchronous and easier to see what is going on.
#include <icey/icey.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;

icey::Stream<int> create_and_spin_node(int argc, char **argv) {
  auto timer = icey::create_timer(1s);

  auto service1 = icey::create_client<ExampleService>("set_bool_service1", 1s);
  auto service2 = icey::create_client<ExampleService>("set_bool_service2", 1s);
  auto service3 = icey::create_client<ExampleService>("set_bool_service3", 1s);

  // The response we are going to receive from the service call:
  using Response = ExampleService::Response::SharedPtr;

  auto node = icey::create_node(argc, argv, "signal_generator_async_await_example");
  /// TODO rem
  node->create_executor_in_context();

  /// Main spinning loop
  while (rclcpp::ok()) {
    /// Receive timer updates
    co_await timer;

    auto request = std::make_shared<ExampleService::Request>();
    request->data = 1;
    RCLCPP_INFO_STREAM(icey::node->get_logger(),
                       "Timer ticked, sending request: " << request->data);

    icey::Result<Response, std::string> result1 = co_await service1.call(request);

    if (result1.has_error()) {
      RCLCPP_INFO_STREAM(node->get_logger(), "Service1 got error: " << result1.error());
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(), "Got response1: " << result1.value()->success);
    }

    /// Then call the second service after we got the response from the first one:
    auto result2 = co_await service2.call(request);

    if (result2.has_error()) {
      RCLCPP_INFO_STREAM(node->get_logger(), "Service2 got error: " << result2.error());
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(), "Got response1: " << result2.value()->success);
    }
  }
  co_return 0;
}

int main(int argc, char **argv) {
  create_and_spin_node(argc, argv);
  /// TODO rem
  icey::destroy();
}
