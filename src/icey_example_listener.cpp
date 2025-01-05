#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

int main(int argc, char **argv) {

    auto my_string = icey::create_signal<StringMsg>("my_string");

    auto derived_value = icey::compute_based_on([](const StringMsg &my_string_val) {
            std::cout << "Computing .. " << std::endl;
            StringMsg result;
            result.data = my_string_val.data;
            result.data.data()[0] = std::toupper(result.data.data()[0]);
            return result;
    },
    my_string);


    derived_value->on_change([](const StringMsg &new_computed_value) {
        std::cout << "derived_value changed: " << new_computed_value.data << std::endl;
    });

    icey::spawn(argc, argv, "listener");

    return 0;           
}