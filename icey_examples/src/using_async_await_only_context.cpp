/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example shows how to use the more lightweight async/await-only context
/// "icey::ContextAsyncAwait". Otherwise this example is the same as the
/// service_client_async_await_example.
/// If you only need async/await and no reactive programming, use only the icey_async_await header
/// to get faster compile times.
#include <icey/icey_async_await.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;
using Response = ExampleService::Response::SharedPtr;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("icey_service_client_async_await_example2");
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());

  /// Create the service client beforehand
  auto service = ctx->create_client<ExampleService>("set_bool_service");

  auto timer = ctx->create_timer_async(1s, [&](std::size_t) -> icey::Promise<void> {
    auto request = std::make_shared<ExampleService::Request>();
    request->data = 1;
    RCLCPP_INFO_STREAM(node->get_logger(), "Timer ticked, sending request: " << request->data);

    /// Call the service and await it's response with a 1s timeout: (for both discovery and the
    /// actual service call)
    icey::Result<Response, std::string> result = co_await service.call(request, 1s);

    if (result.has_error()) {
      /// Handle errors: (possibly "TIMEOUT" or "INTERRUPTED")
      RCLCPP_INFO_STREAM(node->get_logger(), "Got error: " << result.error());
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(), "Got response: " << result.value()->success);
    }
    co_return;
  });
  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 8};
  exec.add_node(node->get_node_base_interface());
  exec.spin();
}
