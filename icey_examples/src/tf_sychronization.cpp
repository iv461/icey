/// This example shows two other ways to obtain transfroms in ICEY
/// 1. Synchronizing a topic with a transfrom, which is equivalent to looking it up with the header
/// stamp and header frame_id as a source_frame
/// 2. Directly subscribing to a transform between two coordinate systems: A callback gets called each time the transform changes.
#include <icey/icey.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <fmt/format.h>
#include <fmt/chrono.h>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "icey_tf_synchronization_example");

  /// Synchronize with a transform: This will yield the message and the transform from the
  /// child_frame_id of the header message and the given target_frame ("map") at the time of the
  /// header stamp. It will wait up to 200ms for the transform. (currently uses tf2_ros::MessageFilter for this)
  node->icey()
      .create_subscription<sensor_msgs::msg::PointCloud2>("/icey/test_pcl")
      .synchronize_with_transform("map", 200ms)
      .unwrap_or([&](std::string error) {
        RCLCPP_INFO_STREAM(node->get_logger(), "Transform lookup error: " << error);
      })
      .then([&](sensor_msgs::msg::PointCloud2::SharedPtr point_cloud,
               const geometry_msgs::msg::TransformStamped &transform_to_map) {

        Eigen::Matrix4d tf_mat = tf2::transformToEigen(transform_to_map.transform).matrix();

        RCLCPP_INFO_STREAM(node->get_logger(), "Received point cloud with header time: " << fmt::format("{:%H:%M:%S}\n", icey::rclcpp_to_chrono(point_cloud->header.stamp))
                  << "\nAnd transform at time: " << fmt::format("{:%H:%M:%S}\n", icey::rclcpp_to_chrono(transform_to_map.header.stamp))
                  << "\nTransform matrix is:\n" << tf_mat);
      });

  icey::spin(node);
}