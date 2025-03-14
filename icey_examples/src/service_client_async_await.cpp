/// This example shows how to use service clients in icey. 
/// By using async/await syntax (i.e. C++20 coroutines) we achieve a synchronously-looking service calls.
/// Under the hood, everything is asynchronous, ICEY actually calls client->async_send_request.
/// ICEY gives you a clean and simple service API: no manual spinning of the event-loop, no threads, no manual cleanups needed. 
#include <icey/icey.hpp>
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;

/// This function creates and spins the node (the main cannot be a coroutine)
icey::Promise<int> run(std::shared_ptr<icey::Node> node) {
  /// Create the service clients beforehand
  auto service1 = node->icey().create_client<ExampleService>("set_bool_service1");
  auto service2 = node->icey().create_client<ExampleService>("set_bool_service2");  

  auto timer = node->icey().create_timer(1s);

  /// Main spinning loop
  while(true) {
    /// First, wait until it's time to make the request
    co_await timer;
    
    auto request = std::make_shared<ExampleService::Request>();
    request->data = 1;
    RCLCPP_INFO_STREAM(node->get_logger(), "Timer ticked, sending request: " << request->data);

    using Response = ExampleService::Response::SharedPtr;

    /// Call the service and await it's response with a 1s timeout: (for both discovery and the actual service call)
    icey::Result<Response, std::string> result1 = co_await service1.call(request, 1s);

    if (result1.has_error()) {
      /// Handle errors: (possibly "SERVICE_UNAVAILABLE", "TIMEOUT" or "INTERRUPTED")
      RCLCPP_INFO_STREAM(node->get_logger(), "Service1 got error: " << result1.error());
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(), "Got response1: " << result1.value()->success);
    }

    /// We can chain service calls: Call a second service after we got the response from the first one:
    auto result2 = co_await service2.call(request, 1s);
    if (result2.has_error()) {
      RCLCPP_INFO_STREAM(node->get_logger(), "Service2 got error: " << result2.error());
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(), "Got response1: " << result2.value()->success);
    }
  }
  co_return 0; // All coroutines must have co_return
}

int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "service_client_async_await_example");
  run(node);
  icey::spin(node);
}
