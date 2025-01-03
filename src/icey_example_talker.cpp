#include <icey/icey.hpp>

#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

int main(int argc, char **argv) {

    auto my_string = icey::PublishedState<StringMsg>("my_string");

    icey::create_timer(100ms, []() {
        my_string.set_value()
    });

    icey::spawn(argc, argv, "talker_node");

    return 0;           
}