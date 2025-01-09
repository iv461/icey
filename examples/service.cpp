#include <icey/icey_ros2.hpp>

#include "std_srvs/srv/set_bool.hpp"

using ExampleService = std_srvs::srv::SetBool;

int main(int argc, char **argv) {
    const auto cb = [](const std::shared_ptr<ExampleService::Request> request,
            std::shared_ptr<ExampleService::Response> response) {
            response->success = request->data;

            RCLCPP_INFO_STREAM(icey::node->get_logger(), "Got request: " << request->data);
    };
    icey::create_service<ExampleService>("add_two_ints", 
        cb);

    icey::spawn(argc, argv, "service_example"); /// Create and start node
}
