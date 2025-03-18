/// This example is for a simple listener, i.e. subscriber in promise style.
#include <icey/icey.hpp>

#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "listener_example");
  auto my_string = node->icey().create_subscription<StringMsg>("my_string", 1);

  /// This callback gets called for each message (it is like the subscriber callback)
  auto derived_value =
      my_string.then([](StringMsg::SharedPtr my_string_val) { return my_string_val->data; });

  /// We can add a continuation:
  derived_value.then([&](const std::string &derived_string) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Got value: " << derived_string);
  });

  icey::spin(node);
}