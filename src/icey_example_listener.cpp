#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

int main(int argc, char **argv) {

    auto my_string = icey::create_signal<StringMsg>("my_string");

    auto derived_value = icey::compute_based_on([](const auto &my_string_val) {
            //return my_string_val.toUpper();
            std::cout << "Computing .. " << std::endl;
            int result = 0;
            //if(my_string_val) {
                result = my_string_val.data[0];
            //}
            return result;
            //return my_string_val;
    },
    my_string);

    derived_value->on_change([](const auto &new_computed_value) {
        std::cout << "derived_value changed: " << new_computed_value << std::endl;
    });

    icey::spawn(argc, argv, "listener");

    return 0;           
}