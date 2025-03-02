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


  auto camera_image = icey.create_subscription<sensor_msgs::msg::Image>("camera");
  auto point_cloud = icey.create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud");

  /// Synchronize with a transform: This will yield the message and the transform from the child_frame_id of the header message 
  /// and the given target_frame ("map") at the time of the header stamp.
  camera_image.synchronize_with_transform("map")
    .then([](sensor_msgs::msg::Image::SharedPtr image, const geometry_msgs::msg::TransformStamped &transform_to_map) {

    });


  /// Synchronize, uses approx time TODO does not work, no stamp
  //
  /*auto float_tfed = icey::synchronize(float_sig, map_base_link_tf);

  /// We always have to take a ConstPtr to the message:
  auto pipe1 = float_tfed.then([](std_msgs::msg::Float32::SharedPtr float_val,
          geometry_msgs::msg::TransformStamped::SharedPtr tf_val) -> float {
          return float(float_val->data * tf_val->transform.rotation.z);
      });*/

  /// this has type Stream<std::tuple<int, float>>
  icey::spin(node);  
}