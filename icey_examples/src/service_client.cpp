#include <icey/icey.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;


/// TODO THIS DOES NOT WORK BECAUSE THE CALLS ARE NOT AWAITED (this needs support for async callbacks)
int main(int argc, char **argv) {
  auto node = icey::create_node<icey::Node>(argc, argv, "service_client_node");
  
  std::cout << "main This threadId: " << std::this_thread::get_id() << std::endl;
  auto service_response =
      node->icey().create_timer(1s)
          /// Build a request when the timer ticks
          .then([&](size_t) {
            std::cout << "cb1 This threadId: " << std::this_thread::get_id() << std::endl;
            auto request = std::make_shared<ExampleService::Request>();
            request->data = true;
            RCLCPP_INFO_STREAM(node->get_logger(),
                               "Timer ticked, sending request: " << request->data);
            return request;
          })
          /// Create a service client, the service is called every time the timer ticks. We set
          /// additionally a timeout for waiting until the service becomes available.
          .call_service<ExampleService>("set_bool_service1", 1s)
          .then([&](ExampleService::Response::SharedPtr response) {
            std::cout << "cb3 This threadId: " << std::this_thread::get_id() << std::endl;
            RCLCPP_INFO_STREAM(node->get_logger(), "Got response1: " << response->success);
            auto request = std::make_shared<ExampleService::Request>();
            request->data = false;
            return request;
          })
          .call_service<ExampleService>("set_bool_service2", 1s)
          .then([&](ExampleService::Response::SharedPtr response) {
            std::cout << "cb4 This threadId: " << std::this_thread::get_id() << std::endl;
            RCLCPP_INFO_STREAM(node->get_logger(), "Got response2: " << response->success);
            auto request = std::make_shared<ExampleService::Request>();
            request->data = true;
            return request;
          })
          .call_service<ExampleService>("set_bool_service3", 1s)
          .then([&](ExampleService::Response::SharedPtr response) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Got response3: " << response->success);
            auto request = std::make_shared<ExampleService::Request>();
            request->data = false;
            return request;
          })
          /// Here we catch timeout errors as well as unavailability of the service:
          .except([&](const std::string& error_code) {
            std::cout << "cb5 This threadId: " << std::this_thread::get_id() << std::endl;
            /// Possible values for error_code are "TIMEOUT", "SERVICE_UNAVAILABLE", or "INTERRUPTED", in case
            /// Ctrl+C was pressed
            RCLCPP_INFO_STREAM(node->get_logger(), "Service got error: " << error_code);
          });

  icey::spin(node);  
}
