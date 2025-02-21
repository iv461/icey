
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
