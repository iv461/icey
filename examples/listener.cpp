#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

int main(int argc, char **argv) {

    auto my_string = icey::create_subscription<StringMsg>("my_string", 1);

    auto derived_value = icey::then(my_string, [](const StringMsg &my_string_val) {
        std::cout << "Computing .. " << std::endl;
        return my_string_val.data;
    });

    icey::then(derived_value, [](const std::string &derived_string) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Got value: " << derived_string);
    });

    icey::spawn(argc, argv, "listener_example");
    return 0;           
}