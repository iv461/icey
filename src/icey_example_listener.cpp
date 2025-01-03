#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

int main(int argc, char **argv) {

    auto my_string = icey::SubscribedState<StringMsg>("my_string");

    auto derived_value = icey::compute_based_on([](const auto &my_string_val) {
            return ing_val.toUpper();
    }, 
    my_string);

    icey::spawn(argc, argv, "listener");

    return 0;           
}