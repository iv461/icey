/// This example demonstrates how to use the class-based API of ICEY:
/// It is a thin wrapper around the rclrpp::Node 
#include <icey/icey.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;


class MyNode : public icey::Node {
public:
    using Base =  icey::Node;
    MyNode(std::string name) : Base(name) {
        
      icey().create_timer(1s)
          /// Build a request when the timer ticks
          .then([this](size_t) {
            auto request = std::make_shared<ExampleService::Request>();
            request->data = 1;
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Timer ticked, sending request: " << request->data);
            return request;
          })
          /// Create a service client, the service is called every time the timer ticks. We set
          /// additionally a timeout for waiting until the service becomes available.
          .call_service<ExampleService>("set_bool_service1", 1s)
          .then([this](ExampleService::Response::SharedPtr response) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Got response1: " << response->success);
            auto request = std::make_shared<ExampleService::Request>();
            request->data = 0;
            return request;
          })
          
          .call_service<ExampleService>("set_bool_service2", 1s)
          .then([this](ExampleService::Response::SharedPtr response) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Got response2: " << response->success);
            auto request = std::make_shared<ExampleService::Request>();
            request->data = 1;
            return request;
          })
          .call_service<ExampleService>("set_bool_service3", 1s)
          .then([this](ExampleService::Response::SharedPtr response) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Got response3: " << response->success);
            auto request = std::make_shared<ExampleService::Request>();
            request->data = 0;
            return request;
          })
          /// Here we catch timeout errors as well as unavailability of the service:
          .except([this](std::string error_code) {
            /// Possible values for error_code are "SERVICE_UNAVAILABLE", or "INTERRUPTED" (in case
            /// we got interrupted while waiting for the service to become available) or
            /// "rclcpp::FutureReturnCode::INTERRUPTED" or "rclcpp::FutureReturnCode::TIMEOUT"
            RCLCPP_INFO_STREAM(this->get_logger(), "Service got error: " << error_code);
          });

        icey_initialize();
    }

    void after_parameters_are_initialized() {
        /// Initialize here you algorithms, are parameters are already available
    }
    /// Put here your code that you would normally put in the destructor
    // (this will called immeditally after ~MyNode() is called actually, due to destruction order rules )
    void on_destruction() {

    }
};


int main(int argc, char **argv) {
   rclcpp::init(argc, argv);
    
   auto node = std::make_shared<MyNode>("service_client_class_api");

   icey::spawn(node);
   return 0;           
}