
#include "node_fixture.hpp"

#include <iostream>
#include <sensor_msgs/msg/image.hpp>
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


    //EXPECT_EQ(node_->get_node_graph_interface()->count_subscribers("/tf"), 0);
    node_->icey().create_transform_subscription("map", "base_link");
    EXPECT_GE(node_->get_node_graph_interface()->count_subscribers("/tf"), 1);

    auto timer = node_->icey().create_timer(90ms);
    EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("/tf"), 0);
    timer.then([](auto ticks) { return geometry_msgs::msg::TransformStamped{}; }).publish_transform();
    EXPECT_GE(node_->get_node_graph_interface()->count_publishers("/tf"), 1);

    /* Does not work on Humble
    EXPECT_EQ(node_->get_node_graph_interface()->count_clients("set_bool_service_icey_test"), 0);
    node_->icey().call_service<ExampleService>("set_bool_service_icey_test", 1s);
    EXPECT_EQ(node_->get_node_graph_interface()->count_clients("set_bool_service_icey_test"), 1);
    
    
    EXPECT_EQ(node_->get_node_graph_interface()->count_services("set_bool_other_service_icey_test"), 0);
    node_->icey().create_service<ExampleService>("set_bool_other_service_icey_test");
    EXPECT_EQ(node_->get_node_graph_interface()->count_services("set_bool_other_service_icey_test"), 1);
    */
    


    /// Here are some assertions relevant for testing ROS nodes: 
    //  auto endpoint_info_vec = pub_node_->get_publishers_info_by_topic("camera/image");
    //EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
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

    auto sub = node_->icey().create_subscription<sensor_msgs::msg::Image>("/icey/maydup_camera");
    EXPECT_EQ(sub.impl()->context.lock().get(), &node_->icey());

    auto pub = node_->icey().create_publisher<sensor_msgs::msg::Image>("/icey/maydup_debug_image");
    EXPECT_EQ(pub.impl()->context.lock().get(), &node_->icey());

    auto tf_stream = node_->icey().create_timer(90ms).then([](auto) { return geometry_msgs::msg::TransformStamped{}; });
    auto tf_pub = node_->icey().create_transform_publisher(tf_stream);
    EXPECT_EQ(tf_pub.impl()->context.lock().get(), &node_->icey());
    
    auto tf_sub = node_->icey().create_transform_subscription("map", "base_link");
    EXPECT_EQ(tf_sub.impl()->context.lock().get(), &node_->icey());

    //// Transformations:
    auto thened_stream = sub.then([](auto x) { return x; });
    EXPECT_EQ(thened_stream.impl()->context.lock().get(), &node_->icey());

    auto timeouted_stream = sub.timeout(1s); 
    EXPECT_EQ(timeouted_stream.impl()->context.lock().get(), &node_->icey());
    
    auto exceped_stream = timeouted_stream.except([](auto, auto, auto) { return double{}; });
    EXPECT_EQ(exceped_stream.impl()->context.lock().get(), &node_->icey());

    //// Filters:
    auto filtered_stream = sub.filter([](auto img) { return img->width > 0;});
    EXPECT_EQ(filtered_stream.impl()->context.lock().get(), &node_->icey());

    auto other_tf_sub = node_->icey().create_transform_subscription("map", "odom");
    auto any_value_stream = node_->icey().any(tf_sub, other_tf_sub);
    EXPECT_EQ(any_value_stream.impl()->context.lock().get(), &node_->icey());

    auto tuple_stream = timer.then([](auto) { return std::make_tuple(double{}, geometry_msgs::msg::TransformStamped{}, std::string{}); });
    auto [double_stream, tf_stream2, string_stream] = tuple_stream.unpack();
    EXPECT_EQ(double_stream.impl()->context.lock().get(), &node_->icey());
    EXPECT_EQ(tf_stream2.impl()->context.lock().get(), &node_->icey());
    EXPECT_EQ(string_stream.impl()->context.lock().get(), &node_->icey());
}


TEST_F(NodeTest, StreamsUseCount) {
    //auto timer = node_->icey().create_timer(100ms);
    //EXPECT_EQ(timer.impl().use_count(), 2); 
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
