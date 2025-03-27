/// This example shows how to use subscriptions with async/await syntax:
/// Instead of a callback, we call co_await on the subscription stream. 
/// we also must introduce a spinning loop so that we receive values continuously.
#include <icey/icey.hpp>

#include "std_msgs/msg/string.hpp"

icey::Promise<void> receive(std::shared_ptr<icey::Node> node) {
  icey::SubscriptionStream<std_msgs::msg::String> sub =
      node->icey().create_subscription<std_msgs::msg::String>("/strings");

  while (true) {
    std_msgs::msg::String::SharedPtr message = co_await sub;
    RCLCPP_INFO_STREAM(node->get_logger(), "Got message: " << message->data);
  }
  co_return;
}

int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "icey_listener_node");
  receive(node);
  icey::spin(node);
}