#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

int main(int argc, char **argv) {

    auto my_string = icey::create_signal<StringMsg>("my_string");

    auto derived_value = icey::compute_based_on([](const auto &my_string_val) {
            //return my_string_val.toUpper();
            return my_string_val;
    },
    my_string);

    icey::spawn(argc, argv, "listener");

    return 0;           
}