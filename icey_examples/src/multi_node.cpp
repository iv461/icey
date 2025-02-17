/// This example shows how multiple nodes can be created and launched in a single process.
#include <icey/icey.hpp>
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    auto generator_node = icey::create_node(argc, argv, "generator_node");
    auto timer_signal = generator_node->icey().create_timer(500ms);

    timer_signal.then([&](size_t ticks) {
        RCLCPP_INFO_STREAM(generator_node->get_logger(), "Timer ticked: " << ticks);
    });

    /// Add another computation for the timer
    timer_signal
        .then([](size_t ticks) {
            std_msgs::msg::Float32 float_val;
            float_val.data = std::sin(ticks / 10.);
            return float_val;
        })
        .publish("sine_generator");

    auto receiver_node = icey::create_node(argc, argv, "receiver_node");
    auto received_sine_signal = receiver_node->icey().create_subscription<std_msgs::msg::Float32>("sine_generator");

    received_sine_signal.then([&](std_msgs::msg::Float32::SharedPtr signal_value) {
        RCLCPP_INFO_STREAM(receiver_node->get_logger(), "Received the sine signal: " << signal_value->data);
    });

    icey::spin_nodes({generator_node, receiver_node});
}