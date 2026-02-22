/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example shows how to use service clients in ICEY: You make an asynchronous call and
/// co_await the response. By using async/await syntax (i.e. C++20 coroutines) we achieve a
/// synchronously-looking code. Under the hood, everything is asynchronous, ICEY actually
/// calls client->async_send_request. ICEY gives you a clean and simple service API: no manual
/// spinning of the event-loop, no threads, no manual cleanups needed.

#include <icey/icey_async_await.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;
using Response = ExampleService::Response::SharedPtr;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("icey_service_client_async_await_example");
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());

  /// Create the service client beforehand
  auto client = ctx->create_client<ExampleService>("set_bool_service");
  auto timer = ctx->create_timer_async(500ms, [&]() -> icey::Promise<void> {
    RCLCPP_INFO_STREAM(node->get_logger(), "Timer ticked");

    auto request = std::make_shared<ExampleService::Request>();
    request->data = 1;
    /// Call the service and await it's response with a 1s timeout: (for both discovery and the
    /// actual service call)
    icey::Result<Response, std::string> result = co_await client.call(request, 1s);

    if (result.has_error()) {
      /// Handle errors: (possibly "TIMEOUT" or "INTERRUPTED")
      RCLCPP_INFO_STREAM(node->get_logger(), "Got error: " << result.error());
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(), "Got response: " << result.value()->success);
    }
    co_return;
  });
  rclcpp::spin(node);
}
