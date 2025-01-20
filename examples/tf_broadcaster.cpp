#include <icey/icey_ros2.hpp>
using namespace std::chrono_literals;

/// You can put the creation of times/subscribers/publishers etc. into a separate function, you do not have
/// to store them in variables explicity:
auto create_yaw_rotation() {
    /// Note we do not assign the timer to a variable here, instead we just call then() on it:
    return icey::create_timer(1s)->then([](size_t ticks) { 
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = icey::node->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
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
    create_yaw_rotation()->publish_transform();
    icey::spawn(argc, argv, "tf_broadcaster_example"); /// Create and start node
}