
#include <icey/icey.hpp>
#include <iostream>
#include <gtest/gtest.h>

using namespace std::chrono_literals;

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

  void spin_all() {
   node_->icey().get_executor()->spin_some(1s);
  }

  std::shared_ptr<icey::Node> node_{std::make_shared<icey::Node>("icey_test_node")};  
};

TEST_F(NodeTest, ContextCreatesEntities) {
    /// Here are some assertions relevant for testing ROS nodes: 
    //  auto endpoint_info_vec = pub_node_->get_publishers_info_by_topic("camera/image");
    //EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
        
    //EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("camera/image"), 0u);
}

TEST_F(NodeTest, TimerTest) {

   size_t timer_ticked{0};
   auto timer = node_->icey().create_timer(100ms);

   EXPECT_EQ(timer_ticked, 0);

   timer
     .then([&](size_t ticks) {
         timer_ticked++;
         if(timer_ticked == 10) {
            timer.impl()->timer->cancel();
         }
     });
   
   EXPECT_EQ(timer_ticked, 0);
   spin_all();
   EXPECT_EQ(timer_ticked, 10);   
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
