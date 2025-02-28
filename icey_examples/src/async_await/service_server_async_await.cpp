/// This example shows how to create an asynchronously responding service server.
/// After it receives a request, it calls asynchronously another upstream service 
/// that is actually capable of anwsering the request. Once it receives the result, it responds.
#include <icey/icey.hpp>
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;
using Response = ExampleService::Response::SharedPtr;

icey::Stream<int> service_call(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "service_service_async_await_example");

  /// Create the service server, without giving it a (synchronous) callback.
  auto service_server = node->icey().create_service<ExampleService>("set_bool_service");

  /// Create a service client for an upstream service that is actually capable of anwsering the request. (1 second timeout)
  auto upstream_service_client = node->icey().create_client<ExampleService>("set_bool_service_upstream", 1s);  

  RCLCPP_INFO_STREAM(node->get_logger(), "Created, starting wait... ");

  /// Wait until a request comes in
  auto [request_id, request] = co_await service_server;
  RCLCPP_INFO_STREAM(node->get_logger(), "Received request: " << request->data << ", calling upstream service ... ");

  icey::Result<Response, std::string> upstream_result = co_await upstream_service_client.call(request);

  if (upstream_result.has_error()) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Upstream service returned error: " << upstream_result.error());
    co_return 0;
  }

  Response upstream_response = upstream_result.value();
  RCLCPP_INFO_STREAM(node->get_logger(), "Got response from upstream service: " << upstream_result.value()->success << ", responding ...");

  /// Now send back the response synchronously: 
  service_server.respond(request_id, upstream_response);

  co_return 0;
}

int main(int argc, char **argv) {
  service_call(argc, argv);
}
