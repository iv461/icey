/// Driver for the tf_subscription test: It publishes a point cloud with a synchronized transform.
#include <icey/icey.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

icey::Promise<void> run(std::shared_ptr<icey::Node> node) {
  /// Synchronize with a transform: This will yield the message and the transform from the
  /// child_frame_id of the header message and the given target_frame ("map") at the time of the
  /// header stamp. It will wait up to 200ms for the transform.
  auto pub = node->icey().create_publisher<sensor_msgs::msg::PointCloud2>("/icey/test_pcl");
  auto tf_pub = node->icey().create_transform_publisher();

  auto timer = node->icey().create_timer(100ms);

  std::size_t cnt{0};
  icey::Time base_time{1700000000s};

  while (true) {
    co_await timer;
    sensor_msgs::msg::PointCloud2 pcl;
    pcl.header.frame_id = "lidar";
    pcl.header.stamp = icey::rclcpp_from_chrono(base_time + cnt * 10ms);
    pub.publish(pcl);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = icey::rclcpp_from_chrono(base_time + cnt * 10ms);
    tf.header.frame_id = "map";
    tf.child_frame_id = "lidar";

    tf.transform.rotation.z = std::sin(0.001 * cnt);
    tf.transform.rotation.w = std::cos(0.001 * cnt);
    tf_pub.publish(tf);
    cnt++;
  }
  co_return;
}

int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "icey_tf_pub_example");
  run(node);
  icey::spin(node);
}
