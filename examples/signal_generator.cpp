#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {

    auto timer_signal = icey::create_timer(500ms);

    icey::then(timer_signal, [](size_t ticks) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Timer ticked: " << ticks);
    });

    /// Add another computation for the timer
    auto sine_signal = icey::then(timer_signal, [](size_t ticks) {
        std_msgs::msg::Float32 float_val;
        float_val.data = std::sin(ticks / 10.);
        return float_val;
    });

    icey::create_publisher(sine_signal, "sine_generator");

    icey::spawn(argc, argv, "signal_generator_example"); 
}