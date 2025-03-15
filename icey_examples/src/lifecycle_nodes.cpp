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
    timer_ = icey().create_timer(100ms);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(get_logger(), "on_activate() was called");
    /// The base class of an icey::LifecycleNode is a rclcpp::LifecycleNode, so it has all the lifecycle methods:
    Base::on_activate(state);
    /// Reset the timer again, starting the loop:
    timer_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(get_logger(), "on_cleanup() was called");
    /// The base class of an icey::LifecycleNode is a rclcpp::LifecycleNode, so it has all the lifecycle methods:
    Base::on_cleanup(state);
    timer_.cancel();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Spin the node: we do some work here, other callbacks get called 
  icey::Promise<void> run() {
    
    while(true) {
      std::size_t ticks = co_await timer_;
      RCLCPP_INFO_STREAM(get_logger(), "Spinning node for the " << ticks << "th time...");
    }
    co_return;
  }

  /// We store the timer here only to be able to cancel it
  icey::TimerStream timer_;

};

int main(int argc, char **argv) {
  auto node = icey::create_node<ExampleLifecycleNode>(argc, argv, "lifecycle_node_example");
  node->run();
  icey::spin(node);
  return 0;
}