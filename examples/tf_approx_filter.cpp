#include <icey/icey_ros2.hpp>
#include "std_msgs/msg/float32.hpp"

int main(int argc, char **argv) {

    auto map_base_link_tf = icey::create_transform_signal("map", "base_link");

    auto result = icey::then(map_base_link_tf, [](const geometry_msgs::msg::TransformStamped &new_transform) {
            std_msgs::msg::Float32 out_msg;
            /// TODO get yaw
            auto cos_theta_half = new_transform.transform.rotation.z;
            RCLCPP_INFO_STREAM(icey::node->get_logger(), "Received a new transform, cos_theta_half was: " << cos_theta_half);
            return out_msg;
        }
        );

    result->publish("new_velocity");

    icey::spawn(argc, argv, "tf_listener_example"); /// Create and start node
}