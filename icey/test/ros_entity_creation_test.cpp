
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
        EXPECT_EQ(timer_ticked, ticks);
        timer_ticked++;
         if(timer_ticked == 10) {
            timer.impl()->timer->cancel();
            node_->icey().get_executor()->cancel(); /// Cancel the executor because otherwise spin() will block forever. And nothing except spin works (i.e spin_once, spin_all, seems like a bug to me) This idea is from test_timer.cpp in rclcpp.
         }
     });   
   EXPECT_EQ(timer_ticked, 0);
   node_->icey().get_executor()->spin();
   EXPECT_EQ(timer_ticked, 10);   
}

TEST_F(NodeTest, OneOffTimerTest) {
   /// Test one-off timer 
   auto timer2 = node_->icey().create_timer(100ms, true);
   timer_ticked = 0;
   EXPECT_EQ(timer_ticked, 0);
   timer2
     .then([&](size_t ticks) {
        EXPECT_EQ(timer_ticked, ticks);
        timer_ticked++;
     });
   node_->icey().get_executor()->spin();
   EXPECT_EQ(timer_ticked, 1);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
