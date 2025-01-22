#include <icey/icey.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

void set_bool_service(std::shared_ptr<ExampleService::Request> request,
                      std::shared_ptr<ExampleService::Response> response) {
  response->success = !request->data;
  RCLCPP_INFO_STREAM(
      icey::node->get_logger(),
      "Got request: " << request->data << ", returning response: " << response->success);
}

int main(int argc, char **argv) {
  icey::icey_debug_print = true;


  /// Now create the service server node
  icey::create_service<ExampleService>("set_bool_service1")->then(set_bool_service);

  icey::create_service<ExampleService>("set_bool_service2")->then(set_bool_service);
  icey::create_service<ExampleService>("set_bool_service3")->then(set_bool_service);

  icey::spawn(argc, argv, "service_server_node");
  return 0;
}
