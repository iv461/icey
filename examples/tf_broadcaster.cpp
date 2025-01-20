#include <icey/icey_ros2.hpp>
using namespace std::chrono_literals;

/// You can put the creation of times/subscribers/publishers etc. into a separate function, you do not have
/// to store them in variables explicity:
auto create_yaw_rotation(std::shared_ptr<icey::Parameter<std::string>> base_frame_param) {
    /// Note we do not assign the timer to a variable here, instead we just call then() on it:
    return icey::create_timer(1s)->then([base_frame_param](size_t ticks) { 
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = icey::node->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = base_frame_param->value(); /// Get the current value of the parameter
        t.transform.translation.x = ticks * .1;
        t.transform.translation.y = ticks * -1.;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.;
        t.transform.rotation.y = 0.;
        t.transform.rotation.z = std::sin(ticks * .1);
        t.transform.rotation.w = std::cos(ticks * .1);
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Timer ticked, publishing new transform with t.child_frame_id: " << t.child_frame_id);
        return t;
    });
}
int main(int argc, char **argv) {
    icey::icey_debug_print = true;
    /// We use here the Parameter-type explicitly:
    std::shared_ptr<icey::Parameter<std::string>> base_frame_param = icey::declare_parameter<std::string>("base_frame", "base_link");

    icey::after_parameter_initialization([&] () {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Parameters arrived");
        /// We can simply pass here the parameter so that the frame_id of the published message 
        // gets updated when the parameter updated dynamically.
        create_yaw_rotation(base_frame_param)->publish_transform();
    });
    icey::spawn(argc, argv, "tf_broadcaster_example"); /// Create and start node
}