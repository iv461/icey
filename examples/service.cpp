#include <icey/icey_ros2.hpp>

#include "std_srvs/srv/set_bool.hpp"

using ExampleService = std_srvs::srv::SetBool;

int main(int argc, char **argv) {
    
    auto service = icey::create_service<ExampleService>("set_bool_service");

    service->then([](std::shared_ptr<ExampleService::Request> request,
            std::shared_ptr<ExampleService::Response> response) {
            response->success = !request->data;

            RCLCPP_INFO_STREAM(icey::node->get_logger(), "Got request: " << request->data);
    });

    icey::spawn(argc, argv, "service_example"); /// Create and start node
}
