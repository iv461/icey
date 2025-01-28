#include <icey/icey.hpp>

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

int main(int argc, char **argv) {
    icey::icey_debug_print = true;
    auto timer_signal = icey::create_timer(1s);

    auto timed_request = timer_signal.then([](size_t  /*ticks*/) {
        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;
        RCLCPP_INFO_STREAM(icey::node("service_client_node")->get_logger(), "Timer ticked, sending request: " <<
                request->data);
        return request;
    });

    /// Create a service client, the service is called every time the timer ticks. We set additionally a timeout for waiting untill the service becomes available.
    auto service_response = icey::create_client<ExampleService>(timed_request, "set_bool_service", 1s);

    service_response
        .then([](ExampleService::Response::SharedPtr response) {
        RCLCPP_INFO_STREAM(icey::node("service_client_node")->get_logger(), "Got response: " << response->success);
        });

    /// Here we catch timeout errors as well as unavailability of the service
    service_response.except([](const std::string& error_code) {
        /// Possible values for error_code are "SERVICE_UNAVAILABLE", "rclcpp::FutureReturnCode::INTERRUPTED" or "rclcpp::FutureReturnCode::TIMEOUT"
        std::cout << "Service got error: " << error_code << std::endl;
    });
    
    auto service_client_node = icey::create_node(argc, argv, "service_client_node"); 

    /// Now create the service node
    auto service = icey::create_service<ExampleService>("set_bool_service");

    service.then([](const std::shared_ptr<ExampleService::Request>& request,
            const std::shared_ptr<ExampleService::Response>& response) {
            response->success = !request->data;
            RCLCPP_INFO_STREAM(icey::node("service_server_node")->get_logger(), 
                "Got request: " << request->data 
                << ", returning response: " << response->success);
            
    });

    auto service_server_node = icey::create_node(argc, argv, "service_server_node"); 
    
    icey::spin_nodes({service_client_node, service_server_node});
}
