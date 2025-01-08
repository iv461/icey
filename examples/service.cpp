#include <icey/icey_ros2.hpp>

#include "std_srvs/srv/set_bool.hpp"

using ExampleService = std_srvs::srv::SetBool;

int main(int argc, char **argv) {
    const auto cb = [](const std::shared_ptr<ExampleService::Request> request,
            std::shared_ptr<ExampleService::Response> response) {
            response->success = request->data;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got request");
    };
    icey::create_service<ExampleService>("add_two_ints", 
        cb);

    icey::spawn(argc, argv, "icey_example_service"); /// Create and start node
}
