#include <icey/icey_ros2.hpp>
#include "std_msgs/msg/float32.hpp"

int main(int argc, char **argv) {
    auto current_velocity = icey::create_signal<std_msgs::msg::Float32>("current_velocity");

    auto result = icey::then(current_velocity, [](std_msgs::msg::Float32 new_velocity) {
            std_msgs::msg::Float32 out_msg;
            out_msg.data = 2. * new_velocity.data;
            return out_msg;
        }
        );

    result->publish("new_velocity");

    icey::spawn(argc, argv, "ppc_controller_node"); /// Create and start node
}