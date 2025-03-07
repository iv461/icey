
#include <iostream>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "node_fixture.hpp"
#include "std_srvs/srv/set_bool.hpp"

using ExampleService = std_srvs::srv::SetBool;
using namespace std::chrono_literals;

TEST_F(NodeTest, ContextCreatesEntities) {
  EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("/icey/maydup_camera"), 0);
  node_->icey().create_subscription<sensor_msgs::msg::Image>("/icey/maydup_camera");
  EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("/icey/maydup_camera"), 1);

  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("/icey/maydup_camera/debug"), 0);
  node_->icey().create_publisher<sensor_msgs::msg::Image>("/icey/maydup_camera/debug");
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("/icey/maydup_camera/debug"), 1);

  // EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("/tf"), 0);
  node_->icey().create_transform_subscription("map", "base_link");
  EXPECT_GE(node_->get_node_graph_interface()->count_subscribers("/tf"), 1);

  auto timer = node_->icey().create_timer(90ms);
  // EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("/tf"), 0); /// Is 1 for some
  // reason, looks like a rclcpp bug
  auto num_pubs_before = node_->get_node_graph_interface()->count_publishers("/tf");
  timer.then([](auto) { return geometry_msgs::msg::TransformStamped{}; }).publish_transform();
  EXPECT_GT(node_->get_node_graph_interface()->count_publishers("/tf"), num_pubs_before);

  /* Does not work on Humble since the functions are not available (they are internally implemented
  via rcl, so we cannot just copy some rclcpp code from Jazzy unfortunately.)
  EXPECT_EQ(node_->get_node_graph_interface()->count_clients("set_bool_service_icey_test"), 0);
  node_->icey().call_service<ExampleService>("set_bool_service_icey_test", 1s);
  EXPECT_EQ(node_->get_node_graph_interface()->count_clients("set_bool_service_icey_test"), 1);

  EXPECT_EQ(node_->get_node_graph_interface()->count_services("set_bool_other_service_icey_test"),
  0); node_->icey().create_service<ExampleService>("set_bool_other_service_icey_test");
  EXPECT_EQ(node_->get_node_graph_interface()->count_services("set_bool_other_service_icey_test"),
  1);
  */

  /// Here are some assertions relevant for testing ROS nodes:
  //  auto endpoint_info_vec = pub_node_->get_publishers_info_by_topic("camera/image");
  // EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(),
  // rclcpp::ReliabilityPolicy::Reliable);
}

TEST_F(NodeTest, ContextCreatesParameters) {
  EXPECT_FALSE(node_->has_parameter("icey_my_test_param1"));
  node_->icey().declare_parameter<int64_t>("icey_my_test_param1", 3);
  EXPECT_TRUE(node_->has_parameter("icey_my_test_param1"));

  EXPECT_EQ(node_->get_parameter_or<int64_t>("icey_my_test_param1", -1LL), 3);
}

/// Checks if Streams created in all possible ways have a context
TEST_F(NodeTest, StreamsHaveContext) {
  /// Entities
  auto timer = node_->icey().create_timer(90ms);
  EXPECT_EQ(timer.impl()->context.lock().get(), &node_->icey());

  auto param = node_->icey().declare_parameter<std::string>("icey_test_my_param", "hello");
  EXPECT_EQ(param.impl()->context.lock().get(), &node_->icey());

  auto sub = node_->icey().create_subscription<sensor_msgs::msg::Image>("/icey/maydup_camera");
  EXPECT_EQ(sub.impl()->context.lock().get(), &node_->icey());

  auto pub = node_->icey().create_publisher<sensor_msgs::msg::Image>("/icey/maydup_debug_image");
  EXPECT_EQ(pub.impl()->context.lock().get(), &node_->icey());
  
  auto service = node_->icey().create_service<ExampleService>("icey_test_service");
  EXPECT_EQ(service.impl()->context.lock().get(), &node_->icey());

  auto tf_stream = node_->icey().create_timer(90ms).then(
      [](auto) { return geometry_msgs::msg::TransformStamped{}; });
  auto tf_pub = node_->icey().create_transform_publisher();
  EXPECT_EQ(tf_pub.impl()->context.lock().get(), &node_->icey());

  auto tf_sub = node_->icey().create_transform_subscription("map", "base_link");
  EXPECT_EQ(tf_sub.impl()->context.lock().get(), &node_->icey());

  //// Filters:
  auto thened_stream = sub.then([](auto x) { return x; });
  EXPECT_EQ(thened_stream.impl()->context.lock().get(), &node_->icey());

  auto timeouted_stream = sub.timeout(1s);
  EXPECT_EQ(timeouted_stream.impl()->context.lock().get(), &node_->icey());

  auto excepted_stream = timeouted_stream.except([](auto, auto, auto) { return double{}; });
  EXPECT_EQ(excepted_stream.impl()->context.lock().get(), &node_->icey());

  auto other_tf_sub = node_->icey().create_transform_subscription("map", "odom");
  auto any_value_stream = icey::any(tf_sub, other_tf_sub);
  EXPECT_EQ(any_value_stream.impl()->context.lock().get(), &node_->icey());

  auto tuple_stream = timer.then([](auto) {
    return std::make_tuple(double{}, geometry_msgs::msg::TransformStamped{}, std::string{});
  });
  auto [double_stream, tf_stream2, string_stream] = tuple_stream.unpack();
  EXPECT_EQ(double_stream.impl()->context.lock().get(), &node_->icey());
  EXPECT_EQ(tf_stream2.impl()->context.lock().get(), &node_->icey());
  EXPECT_EQ(string_stream.impl()->context.lock().get(), &node_->icey());

  auto filtered_stream = sub.filter([](auto img) { return img->width > 0; });
  EXPECT_EQ(filtered_stream.impl()->context.lock().get(), &node_->icey());

  auto unwrapped_stream = timeouted_stream.unwrap_or([](auto, auto, auto) {});
  EXPECT_EQ(unwrapped_stream.impl()->context.lock().get(), &node_->icey());

  auto buffered_stream = sub.buffer(100);
  EXPECT_EQ(buffered_stream.impl()->context.lock().get(), &node_->icey());

  auto sub_pc = node_->icey().create_subscription<sensor_msgs::msg::PointCloud2>("/icey/maydup_gownlibu");
  auto approx_time_synched = icey::synchronize_approx_time(10, sub, sub_pc);
  EXPECT_EQ(approx_time_synched.impl()->context.lock().get(), &node_->icey());
}
