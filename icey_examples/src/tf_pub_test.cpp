/// Driver for the tf_subscriber test
#include <icey/icey.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

icey::Stream<int> spin(int argc, char **argv) {
    
    auto node = icey::create_node(argc, argv, "tf_pub_test");
    
    /// Synchronize with a transform: This will yield the message and the transform from the child_frame_id of the header message 
    /// and the given target_frame ("map") at the time of the header stamp. It will wait up to 200ms for the transform.
    auto pub = node->icey().create_publisher<sensor_msgs::msg::PointCloud2>("/icey/test_pcl");
    
    auto timer = node->icey().create_timer(100ms);
    auto tf_pub = node->icey().create_transform_publisher();
    
    std::size_t cnt{0};
    icey::Time base_time{1700000000s};
    
    while(true) {
        co_await timer;
        sensor_msgs::msg::PointCloud2 pcl;
        pcl.header.frame_id = "lidar";
        pcl.header.stamp = icey::rclcpp_from_chrono(base_time + cnt * 10ms);
        pub.publish(pcl);
    
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = icey::rclcpp_from_chrono(base_time + cnt * 10ms);
        tf.header.frame_id = "map";
        tf.child_frame_id = "lidar";
        
        tf.transform.rotation.z = std::sin(0.001*cnt);
        tf.transform.rotation.w = std::cos(0.001*cnt);
        cnt++;
        tf_pub.publish(tf);
    }
    co_return 0;
}
int main(int argc, char **argv) {
    spin(argc, argv);
}
