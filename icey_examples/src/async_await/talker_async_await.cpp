/// This example shows how to use a publisher with async/await syntax.
/// The synchronous call to .publish() simply calls publish on the ROS publisher.
#include <icey/icey.hpp>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/// We can use coroutines:
icey::Future<std_msgs::msg::String> async_create_message(auto &timer) {
    std::cout << "1. In async_create_message " << std::endl;
    size_t ticks = co_await timer;
    std::cout << "2. After await timer " << std::endl;
    std_msgs::msg::String message;
    message.data = "hello " + std::to_string(ticks);
    co_return message;
}

icey::Stream<int> talk(std::shared_ptr<icey::Node> node) {    
    auto timer = node->icey().create_timer(100ms);
    auto pub = node->icey().create_publisher<std_msgs::msg::String>("/strings");

    
    while(true) {
        std::cout << "3. B4 await " << std::endl;
        std_msgs::msg::String message = co_await async_create_message(timer);
        std::cout << "4. After await async_create_message " << std::endl;
        RCLCPP_INFO_STREAM(node->get_logger(), "Publishing: " << message.data);
        pub.publish(message);
    }
    co_return 0;
}


int main(int argc, char **argv) {
  icey::icey_coro_debug_print = true;
  auto node = icey::create_node(argc, argv, "talker_node");
  talk(node);
  std::cout << "5. After talks " << std::endl;
  icey::spin(node);
}