/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved. 
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This example shows how to use service clients in ICEY: You make an asynchronous call and
/// co_await the response. By using async/await syntax (i.e. C++20 coroutines) we achieve a
/// synchronously-looking code. Under the hood, everything is asynchronous, ICEY actually
/// calls client->async_send_request. ICEY gives you a clean and simple service API: no manual
/// spinning of the event-loop, no threads, no manual cleanups needed.
#include <icey/icey.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;
using Response = ExampleService::Response::SharedPtr;

/// Here we initially the icey::Context manually. Alternatively, you can use the icey::Node that is like a regular rclcpp::Node 
/// but initializes the icey::Context for you.
class MyNode : public rclcpp::Node {
public:
  explicit MyNode(const std::string& name) : rclcpp::Node(name) {
    this->icey_context_ = std::make_shared<icey::Context>(this);
  }

  icey::Context &icey() { return *this->icey_context_; }

  /// This function creates and spins the node (the main cannot be a coroutine)
  icey::Promise<void> run() {
    /// Create the service client beforehand
    auto service = icey().create_client<ExampleService>("set_bool_service");
    auto timer = icey().create_timer(1s);
    
    /// Main spinning loop
    while (true) {
      /// First, await until it's time to make the request:
      co_await timer;

      auto request = std::make_shared<ExampleService::Request>();
      request->data = 1;
      RCLCPP_INFO_STREAM(get_logger(), "Timer ticked, sending request: " << request->data);

      /// Call the service and await it's response with a 1s timeout: (for both discovery and the
      /// actual service call)
      icey::Result<Response, std::string> result = co_await service.call(request, 1s);

      if (result.has_error()) {
        /// Handle errors: (possibly "TIMEOUT" or "INTERRUPTED")
        RCLCPP_INFO_STREAM(get_logger(), "Got error: " << result.error());
      } else {
        RCLCPP_INFO_STREAM(get_logger(), "Got response: " << result.value()->success);
      }
    }
    co_return;  // All coroutines must have co_return
  }

  std::shared_ptr<icey::Context> icey_context_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>("icey_service_client_async_await_exampl");
  
  node->run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
