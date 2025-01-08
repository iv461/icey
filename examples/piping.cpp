/// A more complex mutliple-input, multiple-output (MIMO) example
#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

int main(int argc, char **argv) {

    /*
    auto float_sig = icey::create_signal<std_msgs::msg::Float32>("my_float");
    auto map_base_link_tf = icey::create_transform_signal("map", "base_link");

    auto float_tfed = icey::fuse(float_sig, map_base_link_tf);

    auto pipe1 = icey::then(float_tfed,
        [](const auto vals) -> float {
            const auto [float_val, tf_val] = vals;
            return float(float_val.data * tf_val.transform.rotation.z);
        });

    /// Currently this has type Observable<std::tuple<int, float>> :(
    auto int_result_float_result = icey::then(pipe1, 
        [](const float val) {
            std_msgs::msg::Int32 int_val;
            int_val.data = int(val * 10) % 5;

            std_msgs::msg::Float32 float_val;
            float_val.data = val * 10;
            
            RCLCPP_INFO_STREAM(icey::node->get_logger(), "HELLOOOO ");

            return std::make_tuple(int_val, float_val);
        });
    */

    //int_result.publish("int_result");
    //float_result.publish("float_result");

    icey::spawn(argc, argv, "piping_example"); /// Create and start node
}