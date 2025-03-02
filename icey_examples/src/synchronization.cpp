/// This example shows how to synchronize topics. Both for synchronizing two topics approximatelly
/// as well as obtaining the transform from TF (via lookupTransform) for a given
/// topic, which is another form of synchronization.

#include <icey/icey.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "synchronization_example");  

  auto camera_image = node->icey().create_subscription<sensor_msgs::msg::Image>("camera");
  auto point_cloud = node->icey().create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud");

  /// Synchronize with a transform: This will yield the message and the transform from the child_frame_id of the header message 
  /// and the given target_frame ("map") at the time of the header stamp.
  camera_image.synchronize_with_transform("map")
    .then([](sensor_msgs::msg::Image::SharedPtr image, const geometry_msgs::msg::TransformStamped &transform_to_map) {

    });
  
  /// Or synchronize approx time:
  node->icey().synchronize_approx_time(100, camera_image, point_cloud)
    .then([](sensor_msgs::msg::Image::SharedPtr image, sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {

    });
  icey::spin(node);  
}