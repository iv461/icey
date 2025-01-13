#include <icey/icey_ros2.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

int main(int argc, char **argv) {

    auto timer_signal = icey::create_timer(1s);

    auto timed_request = timer_signal->then([](size_t ticks) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Preparing request... ");
        auto request = std::make_shared<ExampleService::Request>();
        request->data = 1;
        return request;
    });

    auto service_response = icey::create_client(timed_request, "set_bool_service");

    service_response->then([](std::shared_ptr<ExampleService::Response> response) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Got response: " << response->success);
    });
    
    icey::spawn(argc, argv, "client_example"); 
}
