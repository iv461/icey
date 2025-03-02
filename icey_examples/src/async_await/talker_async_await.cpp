/// This example shows how to use a publisher with async/await syntax.
/// The synchronous call to .publish() simply calls publish on the ROS publisher.
#include <icey/icey.hpp>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

icey::Stream<int> create_and_spin_node(int argc, char **argv) {    
    auto talker = icey::create_node(argc, argv, "talker_node");
    
    auto timer = talker->icey().create_timer(100ms);
    auto pub = talker->icey().create_publisher<std_msgs::msg::String>("/strings");
    
    
    /// We can use coroutines:
    auto async_create_message = [timer]() -> icey::Stream<std_msgs::msg::String> {
        size_t ticks = co_await timer;
        std_msgs::msg::String message;
        message.data = "hello " + std::to_string(ticks);
        co_return message;
    };
    
    while(true) {
        std_msgs::msg::String message = co_await async_create_message();
        RCLCPP_INFO_STREAM(talker->get_logger(), "Publishing: " << message.data);
        pub.publish(message);
    }
    co_return 0;
}

int main(int argc, char **argv) {
  create_and_spin_node(argc, argv);
  rclcpp::shutdown();
}