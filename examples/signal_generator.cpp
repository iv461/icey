#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    /// Use graph mode
    icey::get_global_context().use_eager_mode_ = true;
    icey::icey_debug_print = true;

    auto period_time = 100ms;

    getchar();
    auto frequency = icey::declare_parameter<double>("frequency", 10.); // Hz, i.e. 1/s
    auto amplitude = icey::declare_parameter<double>("amplitude", 2.);


    /// You cannot use parameters yet, this will throw an exception:
    // std::cout << "parameter  frequency is:: " << frequency->get() << std::endl;

    /// Receive parameter updates
    amplitude->then([](double new_value) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "amplitude parameter changed: " << new_value);
    });

    auto timer_signal = icey::create_timer(period_time);

    /// Receive timer updates
    timer_signal->then([](size_t ticks) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Timer ticked: " << ticks);
    });

    /// Optional publishing
    auto rectangle_sig = timer_signal->then([](size_t ticks) { 
        std::optional<std_msgs::msg::Float32> result; 
        if(ticks % 10 == 0) { /// Publish with 1/10th of the frequency
            result = std_msgs::msg::Float32();
            result->data = (ticks % 20 == 0) ? 1.f : 0.f;
        }
        
        return result;        
    });
    rectangle_sig->publish("rectangle_signal");

    /// Add another computation for the timer
    auto sine_signal = timer_signal->then([&](size_t ticks) {
        std_msgs::msg::Float32 float_val;
        double period_time_s = 0.1;
        /// We can access parameters in callbacks using .value() because parameters are always initialized first.
        double y = amplitude->value() * std::sin((period_time_s * ticks) / frequency->value() * 2 * M_PI);
        float_val.data = y;
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Publishing sine... " << y);
        return float_val;
    });

    sine_signal->publish("sine_generator");

    icey::spawn(argc, argv, "signal_generator_example"); 
}