/// This example shows that ICEY supports lifecycle nodes: 
/// It is similar to this example: 
/// https://github.com/ros2/demos/blob/rolling/lifecycle/src/lifecycle_talker.cpp
#include <icey/icey.hpp>
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class ExampleLifecycleNode : public icey::LifecycleNode {
public:
  using Base = icey::LifecycleNode;

  ExampleLifecycleNode(std::string name) : Base(name) {
    /// And as well parameters with constraints and a description:
    icey().declare_parameter<double>("amplitude", 2.);

  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(get_logger(), "on_activate() was called");
    Base::on_activate(state);
    /// Reset the timer again, starting the loop:
    timer_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(get_logger(), "on_deactivate() was called");
    Base::on_deactivate(state);
    timer_.cancel();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Spin the node: we do some work here, other callbacks get called 
  icey::Stream<int> spin() {
    timer_ = node->icey().create_timer(period_time);

    while(true) {
      std::size_t ticks = co_await timer;
      RCLCPP_INFO(get_logger(), "Spinning node for the " << ticks << "th time...");
    }
    co_return 0;
  }

  /// We store the timer here only to be able to cancel it
  icey::TimerStream timer_;

};

int main(int argc, char **argv) {
  auto node = icey::create_node<icey::ExampleLifecycleNode>(argc, argv, "lifecycle_node_example");
  node->spin();
  return 0;
}