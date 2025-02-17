#include <icey/icey.hpp>
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    auto node = icey::create_node(argc, argv, "tf_listener_example");
    auto map_base_link_tf = node->icey().create_transform_subscription("map", "base_link");

    map_base_link_tf.except([&](auto err) { 
            RCLCPP_INFO_STREAM(node->get_logger(), "Transform subscriber failed: " << err);
        });
    
    /// Register timeout detection
    map_base_link_tf
        .timeout(500ms)
        .except([&](auto current_time, auto message_time, auto max_age) {
            auto message_age_sec = (current_time - message_time).seconds();
            RCLCPP_INFO_STREAM(node->get_logger(), "Timeout, message is " << message_age_sec << "s old, but the maximum allowed is " << max_age.seconds());
        });
    
    auto result = map_base_link_tf.then([&](geometry_msgs::msg::TransformStamped::SharedPtr new_transform) {
            std_msgs::msg::Float32 out_msg;
            auto cos_theta_half = new_transform->transform.rotation.z;
            RCLCPP_INFO_STREAM(node->get_logger(), "Received a new transform, cos_theta_half was: " << cos_theta_half);
            return out_msg;
        })/*.except([](auto err) { the publish after the .except would not compile
            RCLCPP_INFO_STREAM(icey::node->get_logger(), "Transform subscriber failed: " << err);
        })*/ ; 
        
    result.publish("new_velocity");

    icey::spin(node);
}