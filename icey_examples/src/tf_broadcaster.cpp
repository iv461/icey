#include <icey/icey.hpp>
using namespace std::chrono_literals;

/// You can put the creation of times/subscriptions/publishers etc. into a separate function, you do
/// not have to store them in variables explicitly:
auto create_yaw_rotation(icey::Node &node,
                         const icey::ParameterStream<std::string> &base_frame_param) {
  /// Note we do not assign the timer to a variable here, instead we just call then() on it:
  return node.icey().create_timer(1s).then([&node, base_frame_param](size_t ticks) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node.get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = base_frame_param.value();  /// Get the current value of the parameter
    t.transform.translation.x = ticks * .1;
    t.transform.translation.y = ticks * -1.;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.;
    t.transform.rotation.y = 0.;
    t.transform.rotation.z = std::sin(ticks * .1);
    t.transform.rotation.w = std::cos(ticks * .1);
    return t;
  });
}
int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "icey_tf_broadcaster_example");

  icey::ParameterStream<std::string> base_frame_param =
      node->icey().declare_parameter<std::string>("base_frame", "base_link");
  /// We can simply pass here the parameter so that the frame_id of the published message
  // gets updated dynamically when the parameter changes.
  create_yaw_rotation(*node, base_frame_param).publish_transform();
  icey::spin(node);
}