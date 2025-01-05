#include <icey/icey_ros2.hpp>

#include "example_interfaces/srv/add_two_ints.hpp"

int main(int argc, char **argv) {

    const auto cb = [](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
            response->sum = request->a + request->b;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                            request->a, request->b);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
    };

    icey::create_service<std_msgs::msg::Float32>("add_two_ints", cb);

    icey::spawn(argc, argv, "add_two_ints_server"); /// Create and start node
}