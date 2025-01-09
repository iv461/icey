/// This example demonstrates how to use the class-based API of ICEY:
/// It is a thin wrapper around the rclrpp::Node 
#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

class MyNode : public icey::Node {
public:
    using Base =  icey::Node;
    MyNode(std::string name) : Base(name) {

        auto my_string = icey().create_subscription<StringMsg>("my_string");

        auto derived_value = then(my_string, [](const StringMsg &my_string_val) {
                std::cout << "Computing .. " << std::endl;
                StringMsg result;
                result.data = my_string_val.data;
                result.data.data()[0] = std::toupper(result.data.data()[0]);
                return result;
        });


        derived_value->on_change([](const StringMsg &new_computed_value) {
            std::cout << "derived_value changed: " << new_computed_value.data << std::endl;
        });

        /// Finally, create all the needed subsciptions, publications etc. for the ICEY-observables we just declared.
        icey_initialize();
    }
};


int main(int argc, char **argv) {
   rclcpp::init(argc, argv);

   auto node = std::make_shared<MyNode>("class_based_node_example");

   icey::spawn(argc, argv, node);
   return 0;           
}