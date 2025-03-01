/// Test here Result
/// unpack
/// test timeout filter.
/// synchronize_with_transform
/// test serialize
auto timer_signal = icey.create_timer(period_time);

  /// Receive timer updates
  const auto [a_float, a_strin, an_int] =
      timer_signal
          .then([node](size_t ticks) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Timer ticked: " << ticks);
            return std::make_tuple(3., "hellooo", 66);
          }).unpack();


          #include <icey/icey.hpp>

          #include "sensor_msgs/msg/image.hpp"
          #include "sensor_msgs/msg/point_cloud2.hpp"
          #include "std_msgs/msg/float32.hpp"
          #include "std_msgs/msg/int32.hpp"
          
          int main(int argc, char **argv) {
            auto node = icey::create_node(argc, argv, "signal_generator");
            auto &icey = node->icey();
          
            auto map_base_link_tf = icey.create_transform_subscription("map", "base_link");
          
            auto float_sig = icey.create_subscription<std_msgs::msg::Float32>("my_float");
            auto camera_image = icey.create_subscription<sensor_msgs::msg::Image>("camera");
            auto point_cloud = icey.create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud");
          
            /// This will do synchronize_with_reference(camera_image, map_base_link_tf)
            auto cam_sync = icey.synchronize(camera_image, map_base_link_tf);
          
            /// Synchronize, uses approx time TODO does not work, no stamp
            //
            /*auto float_tfed = icey::synchronize(float_sig, map_base_link_tf);
          
            /// We always have to take a ConstPtr to the message:
            auto pipe1 = float_tfed.then([](std_msgs::msg::Float32::SharedPtr float_val,
                    geometry_msgs::msg::TransformStamped::SharedPtr tf_val) -> float {
                    return float(float_val->data * tf_val->transform.rotation.z);
                });*/
          
            /// this has type Stream<std::tuple<int, float>>
          
            auto float1 = icey.create_subscription<std_msgs::msg::Float32>("float1");
            auto float2 = icey.create_subscription<std_msgs::msg::Float32>("float2");
            auto float3 = icey.create_subscription<std_msgs::msg::Float32>("float3");
          
            icey::any(float1, float2, float3).publish("float_serialized");
          
            /// And directly publish this signal:
            map_base_link_tf.publish("map_to_base_link_transform");
          
            icey::spin(node);  
          }