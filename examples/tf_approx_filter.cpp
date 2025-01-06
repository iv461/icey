#include <icey/icey_ros2.hpp>
#include "std_msgs/msg/float32.hpp"

int main(int argc, char **argv) {

    auto map_base_link_tf = icey::create_transform_signal("map", "base_link");

    auto result = icey::then(map_base_link_tf, [](auto new_msg) {
            std_msgs::msg::Float32 out_msg;
            
            return out_msg;
        }
        );

    result->publish("new_velocity");

    icey::spawn(argc, argv, "tf_listener_tf"); /// Create and start node
}