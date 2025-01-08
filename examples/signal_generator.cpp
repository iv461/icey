#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {

    auto timer_signal = icey::create_timer(500ms);

    icey::then(timer_signal, [](size_t ticks) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Timer ticked: " << ticks);
    });

    icey::spawn(argc, argv, "signal_generator_example"); 
}