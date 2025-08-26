/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example shows how to create an asynchronously responding service server.
/// After it receives a request, it calls asynchronously another upstream service
/// that is actually capable of answering the request. Once it receives the result, it responds.
#include <icey/icey.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;
using Request = ExampleService::Request::SharedPtr;
using Response = ExampleService::Response::SharedPtr;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icey::Node>("icey_service_service_async_await_example");

  /// Create a service client for an upstream service that is actually capable of answering the
  /// request.
  auto upstream_service_client =
      node->icey().create_client<ExampleService>("set_bool_service_upstream");

  /// Create the service server and give it a asynchronous callback (containing keyword co_await)
  node->icey().create_service<ExampleService>(
      "set_bool_service", [&](auto request) -> icey::Promise<Response> {
        RCLCPP_INFO_STREAM(
            node->get_logger(),
            "Received request: " << request->data << ", calling upstream service ... ");
        /// Call the upstream service with 1s timeout asynchronously:
        icey::Result<Response, std::string> upstream_result =
            co_await upstream_service_client.call(request, 1s);

        if (upstream_result.has_error()) {
          RCLCPP_INFO_STREAM(node->get_logger(),
                             "Upstream service returned error: " << upstream_result.error());
          /// Return nothing: This will simply not respond to the client, leading to a timeout
          co_return nullptr;
        } else {
          Response upstream_response = upstream_result.value();
          RCLCPP_INFO_STREAM(node->get_logger(), "Got response from upstream service: "
                                                     << upstream_result.value()->success
                                                     << ", responding ...");
          /// Respond to the client with the upstream response:
          co_return upstream_response;
        }
      });

  RCLCPP_INFO_STREAM(node->get_logger(), "Created service server, waiting for requests ... ");
  rclcpp::spin(node);
}
