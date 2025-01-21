
#include <icey/icey_ros2.hpp>
#include <iostream>
#include <gtest/gtest.h>

class NodeTest : public testing::Test {
 protected:
  
  NodeTest() {
     // You can do set-up work for each test here.
  }

  ~NodeTest() override {
     // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override {
     // Code here will be called immediately after the constructor (right
     // before each test).
  }

  void TearDown() override {
     // Code here will be called immediately after each test (right
     // before the destructor).
  }
};

TEST_F(NodeTest, Smoke) {
    /// Here are some assertions relevant for testing ROS nodes: 
    //  auto endpoint_info_vec = pub_node_->get_publishers_info_by_topic("camera/image");
    //EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
        
    //EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("camera/image"), 0u);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  executor.spin();
  rclcpp::shutdown();
  return RUN_ALL_TESTS();
}
