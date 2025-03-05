/// This example shows how to lookup transforms on TF asynchronously. 
/// We look for each message that we get the transform and await it.
/// This example is therefore equivalent to the synchronize_with_transform() filter (except that the input messages are unbuffered)
#include <icey/icey.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

icey::Stream<int> spin(int argc, char **argv) {
    auto node = icey::create_node(argc, argv, "tf_lookup_async_await_example");
    
    auto sub = node->icey().create_subscription<sensor_msgs::msg::PointCloud2>("/icey/test_pcl");
    auto tf_subscriber = node->icey().create_transform_subscription();

    while(true) {
        sensor_msgs::msg::PointCloud2::SharedPtr point_cloud = co_await sub;
        icey::Result<geometry_msgs::msg::TransformStamped, std::string> tf_result = co_await tf_subscriber.lookup("map", point_cloud->header.frame_id, 200ms);

        if(tf_result.has_value()) {
            geometry_msgs::msg::TransformStamped transform_to_map = tf_result.value();
            RCLCPP_INFO(node->get_logger(), "Got transform %f", transform_to_map.rotation.w);
        } else {
            RCLCPP_INFO(node->get_logger(), "Transform lookup error %s", tf_result.error());
        }

    }
    co_return 0;
}
    
int main(int argc, char **argv) {
    spin(argc, argv);
}