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

    /// 1s timeout
    auto service_response = icey::create_client<ExampleService>(timed_request, "set_bool_service", 1s);

    service_response
        ->then([](ExampleService::Response::SharedPtr response) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Got response: " << response->success);
        });

    service_response->except([](rclcpp::FutureReturnCode retcode) {
        std::cout << "Service got error: " << retcode << std::endl;
    });
    
    icey::spawn(argc, argv, "client_example"); 
}
