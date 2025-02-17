#include <icey/icey.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

void set_bool_service(const std::shared_ptr<ExampleService::Request>& request,
                      const std::shared_ptr<ExampleService::Response>& response) {
  response->success = !request->data;
}

int main(int argc, char **argv) {  
  auto node = icey::create_node<icey::Node>(argc, argv, "service_server");
  auto &icey = node->icey();  // Get the ICEY context

  /// Now create the service server node
  auto service_cb = [&](auto request, auto response) {
    set_bool_service(request, response);
    RCLCPP_INFO_STREAM(
      node->get_logger(), "Got request: " << request->data << ", returning response: " << response->success);
  };


  icey.create_service<ExampleService>("set_bool_service1").then(service_cb);
  icey.create_service<ExampleService>("set_bool_service2").then(service_cb);
  icey.create_service<ExampleService>("set_bool_service3").then(service_cb);

  icey::spin(node);  
}
