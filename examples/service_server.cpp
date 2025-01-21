#include <icey/icey_ros2.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

int main(int argc, char **argv) {
    icey::icey_debug_print = true;
    
    /// Now create the service server node
    auto service = icey::create_service<ExampleService>("set_bool_service");

    service->then([](std::shared_ptr<ExampleService::Request> request,
            std::shared_ptr<ExampleService::Response> response) {
            response->success = !request->data;
            RCLCPP_INFO_STREAM(icey::node("service_server_node")->get_logger(), 
                "Got request: " << request->data 
                << ", returning response: " << response->success);
            
    });

    icey::spawn(argc, argv, "service_server_node"); 
    return 0;
}
