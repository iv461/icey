/// This example demonstrates how to use the class-based API of ICEY:
/// It is a thin wrapper around the rclrpp::Node
#include <icey/icey.hpp>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

using namespace std::chrono_literals;

class MyNode : public icey::Node {
public:
  explicit MyNode(const std::string& name) : icey::Node(name) {
    auto timer_signal = icey().create_timer(500ms);

    timer_signal.then(
        [this](size_t ticks) { RCLCPP_INFO_STREAM(get_logger(), "Timer ticked: " << ticks); });

    timer_signal
        .then([](size_t ticks) {
          std_msgs::msg::Float32 float_val;
          float_val.data = std::sin(ticks / 10.);
          return float_val;
        })
        .publish("sine_generator");
  }  
};

int main(int argc, char **argv) {
  icey::spin(icey::create_node<MyNode>(argc, argv, "class_based_node_example"));
}