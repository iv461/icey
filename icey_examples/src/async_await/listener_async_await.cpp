/// This example shows how to use subscribers with async/await syntax.
#include <icey/icey.hpp>
#include "std_msgs/msg/string.hpp"

icey::Stream<int> create_and_spin_node(int argc, char **argv) {
    auto listener_node = icey::create_node(argc, argv, "listener_node");
    auto sub = listener_node->icey().create_subscription<std_msgs::msg::String>("/strings");
    
    while(true) {
        std_msgs::msg::String::SharedPtr message = co_await sub;
        RCLCPP_INFO_STREAM(listener_node->get_logger(), "Got message: " << message->data);
    }

    co_return 0;
}

int main(int argc, char **argv) {
  
  create_and_spin_node(argc, argv);
  rclcpp::shutdown();
}