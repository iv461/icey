/// This example shows how to use service clients in icey. 
/// By using async/await syntax (i.e. C++20 coroutines) we achieve a synchronously-looking service calls.
/// Under the hood, everything is asynchronous, ICEY actually calls client->async_send_request.
/// ICEY gives you a clean and simple service API: no manual spinning of the event-loop, no threads, no manual cleanups needed. 
#include <iostream>
#include <icey/icey.hpp>
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;

/// This function creates and spins the node (the main cannot be a coroutine)
icey::Stream<int> run(std::shared_ptr<icey::Node> node) {
  /// Create the service clients beforehand
  auto service1 = node->icey().create_client<ExampleService>("set_bool_service1");
  std::cout << "1. In  run " << std::endl;
    auto request = std::make_shared<ExampleService::Request>();
    request->data = 1;
    RCLCPP_INFO_STREAM(node->get_logger(), "sending request: " << request->data);

    using Response = ExampleService::Response::SharedPtr;

    /// Call the service and await it's response with a 1s timeout: (for both discovery and the actual service call)
    std::cout << "2. before await " << std::endl;
    auto call_p = service1.call(request, 1s);
    icey::Result<Response, std::string> result1 = co_await call_p;
    std::cout << "3. After await" << std::endl;
    if (result1.has_error()) {
      /// Handle errors: (possibly "SERVICE_UNAVAILABLE", "TIMEOUT" or "INTERRUPTED")
      RCLCPP_INFO_STREAM(node->get_logger(), "Service1 got error: " << result1.error());
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(), "Got response1: " << result1.value()->success);
    }

  co_return 0; // All coroutines must have co_return
}

int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "service_client_async_await_example");
  std::cout << "4. Before run" << std::endl;
  run(node);
  std::cout << "5. After run" << std::endl;
  icey::spin(node);
}
