#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {

    icey::icey_debug_print = true;

    auto period_time = 100ms;

    auto frequency = icey::declare_parameter<double>("frequency", 10.); // Hz, i.e. 1/s
    auto amplitude = icey::declare_parameter<double>("amplitude", 2.);

    auto timer_signal = icey::create_timer(period_time);

    icey::then(timer_signal, [](size_t ticks) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Timer ticked: " << ticks);
    });

    /// Add another computation for the timer
    auto sine_signal = icey::then(timer_signal, [&](size_t ticks) {
        std_msgs::msg::Float32 float_val;
        double period_time_s = std::chrono::duration_cast<std::chrono::seconds>(period_time).count();
        /// We can .get() parameters since they are always initialized first, so at this point they are alreay there
        double y = amplitude->get() * std::sin((period_time_s * ticks) / frequency->get() * 2 * M_PI);
        float_val.data = y;
        return float_val;
    });

    icey::create_publisher(sine_signal, "sine_generator");

    icey::spawn(argc, argv, "signal_generator_example"); 
}