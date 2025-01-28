#include <icey/icey.hpp>
#include "std_msgs/msg/float32.hpp"

int main(int argc, char **argv) {

    auto map_base_link_tf = icey::create_transform_subscription("map", "base_link");

    map_base_link_tf.except([](auto err) { 
            RCLCPP_INFO_STREAM(icey::node->get_logger(), "Transform subscriber failed: " << err);
        });
    
    auto result = map_base_link_tf.then([](geometry_msgs::msg::TransformStamped::SharedPtr new_transform) {
            std_msgs::msg::Float32 out_msg;
            auto cos_theta_half = new_transform->transform.rotation.z;
            RCLCPP_INFO_STREAM(icey::node->get_logger(), "Received a new transform, cos_theta_half was: " << cos_theta_half);
            return out_msg;
        })/*.except([](auto err) { the publish after the .except would not compile
            RCLCPP_INFO_STREAM(icey::node->get_logger(), "Transform subscriber failed: " << err);
        })*/ ; 
        
    result.publish("new_velocity");

    icey::spawn(argc, argv, "tf_listener_example"); /// Create and start node
}