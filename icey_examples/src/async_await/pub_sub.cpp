#include <icey/icey.hpp>

#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

icey::Stream<int> create_and_spin_node() {
    
    auto sender = std::make_shared<icey::Node>("icey_test_sender_node");
    auto receiver = std::make_shared<icey::Node>("icey_test_receiver_node");

    sender->icey().get_executor()->remove_node(sender);
    receiver->icey().get_executor()->add_node(sender->get_node_base_interface());
    /// Assing the context executor
    sender->icey().get_executor() = receiver->icey().get_executor();
    
    std::size_t received_cnt{0};
    
    receiver->icey().create_subscription<std_msgs::msg::Float32>("/icey_test/sine_signal")
        .then([&](auto msg) {
            received_cnt++;
        });
        
    auto timer = sender->icey().create_timer(100ms);
    auto pub = sender->icey().create_publisher<std_msgs::msg::Float32>("/icey_test/sine_signal", 1);
    
    
    auto coro = [timer]() -> icey::Stream<std_msgs::msg::Float32> {
        size_t ticks = co_await timer;
        std_msgs::msg::Float32 float_val;
        float_val.data = ticks;
        co_return float_val;
    };
    
    for(int i= 0; i < 10; i++) {
        std::cout << "Pulibshing .. " << std::endl;
        pub.publish(co_await coro());
    }   
    

    co_return 0;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  create_and_spin_node();
  rclcpp::shutdown();
}