/// This example shows how to write a simple subscription.
/// It receives messages from the talker example.
#include <icey/icey.hpp>
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "icey_listener_example");
  
  node->icey().create_subscription<std_msgs::msg::String>("my_string", 
    [&](std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO_STREAM(node->get_logger(), "Got value: " << msg->data);
     });
  icey::spin(node);
}