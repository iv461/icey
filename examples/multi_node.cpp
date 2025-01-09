#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {

    auto timer_signal = icey::create_timer(500ms);

    /// Timers can have multiple destinations: First, a callback that simply prints 
    icey::then(timer_signal, [](size_t ticks) {
        RCLCPP_INFO_STREAM(icey::node("generator_node")->get_logger(), "Timer ticked: " << ticks);
    });

    /// Add another computation for the timer
    auto sine_signal = icey::then(timer_signal, [](size_t ticks) {
        std_msgs::msg::Float32 float_val;
        float_val.data = std::sin(ticks / 10.);
        return float_val;
    });
    icey::create_publisher(sine_signal, "sine_generator");

    auto generator_node = icey::create_node(argc, argv, "generator_node"); 

    auto received_sine_signal = icey::create_subscription<std_msgs::msg::Float32>("sine_generator");

    icey::then(received_sine_signal, [](const auto signal_value) {
        RCLCPP_INFO_STREAM(icey::node("receiver_node")->get_logger(), "Received the sine signal: " << signal_value.data);
    });

    auto receiver_node = icey::create_node(argc, argv, "receiver_node");

    icey::spin_nodes({generator_node, receiver_node});
}