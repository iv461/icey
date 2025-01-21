/// This example demonstrates how to use the class-based API of ICEY:
/// It is a thin wrapper around the rclrpp::Node 
#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using StringMsg = std_msgs::msg::String;

using namespace std::chrono_literals;


class MyNode : public icey::Node {
public:
    using Base =  icey::Node;
    MyNode(std::string name) : Base(name) {

       auto timer_signal = icey().create_timer(500ms);

        
        timer_signal->then([this](size_t ticks) {
            RCLCPP_INFO_STREAM(get_logger(), "Timer ticked: " << ticks);
        });
        
        timer_signal->then([](size_t ticks) {
            std_msgs::msg::Float32 float_val;
            float_val.data = std::sin(ticks / 10.);
            return float_val;
        })->publish("sine_generator");

        /// Register callbacks
        icey().register_after_parameter_initialization_cb([this](){after_parameters_are_initialized(); });
        /// Register callback when this node is destructed (this is still valid at this point)
        icey().register_on_node_destruction_cb([this](){ on_destruction(); });

        /// Finally, call this at the end of the constructor to create all the needed subsciptions, publications etc. 
        icey_initialize();
    }

    void after_parameters_are_initialized() {
        /// Initialize here you algorithms, are parameters are already available
    }
    /// Put here your code that you would normally put in the destructor
    // (this will called immeditally after ~MyNode() is called actually, due to destruction order rules )
    void on_destruction() {

    }
};


int main(int argc, char **argv) {
   rclcpp::init(argc, argv);

   auto node = std::make_shared<MyNode>("class_based_node_example");

   icey::spawn(node);
   return 0;           
}