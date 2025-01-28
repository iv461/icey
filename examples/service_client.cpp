#include <icey/icey.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;

int main(int argc, char **argv) {
  auto service_response =
      icey::create_timer(1s)
          /// Build a request when the timer ticks
          .then([](size_t) {
            auto request = std::make_shared<ExampleService::Request>();
            request->data = 1;
            RCLCPP_INFO_STREAM(icey::node->get_logger(),
                               "Timer ticked, sending request: " << request->data);
            return request;
          })
          /// Create a service client, the service is called every time the timer ticks. We set
          /// additionally a timeout for waiting until the service becomes available.
          .call_service<ExampleService>("set_bool_service1", 1s)
          .then([](ExampleService::Response::SharedPtr response) {
            RCLCPP_INFO_STREAM(icey::node->get_logger(), "Got response1: " << response->success);
            auto request = std::make_shared<ExampleService::Request>();
            request->data = 0;
            return request;
          })

          .call_service<ExampleService>("set_bool_service2", 1s)
          .then([](ExampleService::Response::SharedPtr response) {
            RCLCPP_INFO_STREAM(icey::node->get_logger(), "Got response2: " << response->success);
            auto request = std::make_shared<ExampleService::Request>();
            request->data = 1;
            return request;
          })
          .call_service<ExampleService>("set_bool_service3", 1s)
          .then([](ExampleService::Response::SharedPtr response) {
            RCLCPP_INFO_STREAM(icey::node->get_logger(), "Got response3: " << response->success);
            auto request = std::make_shared<ExampleService::Request>();
            request->data = 0;
            return request;
          })
          /// Here we catch timeout errors as well as unavailability of the service:
          .except([](std::string error_code) {
            /// Possible values for error_code are "SERVICE_UNAVAILABLE", or "INTERRUPTED" (in case
            /// we got interrupted while waiting for the service to become available) or
            /// "rclcpp::FutureReturnCode::INTERRUPTED" or "rclcpp::FutureReturnCode::TIMEOUT"
            RCLCPP_INFO_STREAM(icey::node->get_logger(), "Service got error: " << error_code);
          });

  icey::spawn(argc, argv, "service_client_node");
}
