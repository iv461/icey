/// This example shows how to use service servers with a synchronous callback: 
/// Meaning, on request the service returns a response. 
/// This is how a regular ROS service works more or less.
#include <icey/icey.hpp>
#include "std_srvs/srv/set_bool.hpp"

using ExampleService = std_srvs::srv::SetBool;

int main(int argc, char **argv) {  
  auto node = icey::create_node(argc, argv, "service_server");
  auto service_name = node->icey().declare_parameter<std::string>("service_name", "set_bool_service1");
  
  /// The synchronous callback of the service server: We receive the request and return the response.
  auto service_cb = [node](std::shared_ptr<ExampleService::Request> request) {
    auto response = std::make_shared<ExampleService::Response>();
    response->success = !request->data;
    RCLCPP_INFO_STREAM(
      node->get_logger(), "Got request: " << request->data << ", returning response: " << response->success);
    return response;
  };
  
  /// Create the service and give it the callback:
  node->icey().create_service<ExampleService>(service_name, service_cb);
  node->icey().create_service<ExampleService>("set_bool_service2", service_cb);
  node->icey().create_service<ExampleService>("set_bool_service3", service_cb);
  
  RCLCPP_INFO_STREAM(node->get_logger(), "Started services " << service_name.value());
  icey::spin(node);  
}
