/// This example shows how to get transfroms from TF:  
/// 1. Synchronizing a topic with a transfrom, which is equivalent to looking it up with the header stamp
/// 2. Directly subscribing to a transform between two coordinate systems
#include <icey/icey.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    auto node = icey::create_node(argc, argv, "tf_subscription_example");
    
    /// Synchronize with a transform: This will yield the message and the transform from the child_frame_id of the header message 
    /// and the given target_frame ("map") at the time of the header stamp. It will wait up to 200ms for the transform.
    node->icey().create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points")
        .synchronize_with_transform("map", 200ms)
        .unwrap_or([&](std::string error) { RCLCPP_INFO(node->get_logger(), "Transform lookup error %s", error); })
        .then([](sensor_msgs::msg::PointCloud2::SharedPtr image, const geometry_msgs::msg::TransformStamped &transform_to_map) {
            std::cout << "image width: " << image->width 
            << " tf w: " << transform_to_map.transform.rotation.w << std::endl;
        });
    
    /// Or subscribe directly:
    node->icey().create_transform_subscription("map", "base_link")
        .unwrap_or([&](std::string error) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Transform subscriber failed: " << error);
        })
        .then([&](geometry_msgs::msg::TransformStamped::SharedPtr new_transform) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Received a new transform: ");
        });

    icey::spin(node);
}