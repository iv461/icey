#include <icey/icey.hpp>
#include <icey/parameters_struct.hpp>

#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/// All parameters of the node
struct NodeParameters {
  /// We set a default value, allowed interval and a description
  icey::DynParameter<double> frequency{10., icey::Interval(0., 25.),
                                       std::string("The frequency of the sine")};
  icey::DynParameter<double> amplitude{3};
};

icey::Stream<int> create_and_spin_node(int argc, char **argv) {
    std::cout << "Starting node .. " << std::endl;
  auto period_time = 100ms;
  NodeParameters params;
  /// Declare parameter struct and receive updates each time
  icey::declare_parameter_struct(
      icey::get_global_context(), params, [](const std::string &changed_parameter) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(),
                           "Parameter " << changed_parameter << " changed, params are now:\n");
      });

  auto timer = icey::create_timer(period_time);
  auto rectangle_pub = icey::create_publisher<std_msgs::msg::Float32>("rectangle_signal", rclcpp::SystemDefaultsQoS());
  auto sine_pub = icey::create_publisher<std_msgs::msg::Float32>("sine_signal", rclcpp::SystemDefaultsQoS());

  auto node = icey::create_node(argc, argv, "signal_generator_async_await_example");
  node->create_executor_in_context();
    std::cout << "Starting loop .. " << std::endl;
  /// Main spinning loop
  while (rclcpp::ok()) {
    /// Receive timer updates
    size_t ticks = co_await timer;

    RCLCPP_INFO_STREAM(node->get_logger(), "Timer ticked: " << ticks);
    
    if (ticks % 10 == 0) {  /// Publish with 1/10th of the frequency
      std_msgs::msg::Float32 result;
      result.data = (ticks % 20 == 0) ? 1.f : 0.f;  
      rectangle_pub.publish(result);
    } 
    
    /// Add another computation for the timer
    std_msgs::msg::Float32 float_val;
    double period_time_s = 0.1;
    /// We can access parameters in callbacks using .value() because parameters are always
    /// initialized first.
    double y = params.amplitude * std::sin((period_time_s * ticks) * params.frequency * 2 * M_PI);
    float_val.data = y;
    RCLCPP_INFO_STREAM(node->get_logger(), "Publishing sine... " << y);
    sine_pub.publish(float_val);
  
  }
  co_return 0;
}

int main(int argc, char **argv) {
  create_and_spin_node(argc, argv);
  
  /// TODO do not store the node in the context so that this is not needed
  icey::destroy();
}