
#include "node_fixture.hpp"

using namespace std::chrono_literals;
#include "std_msgs/msg/float32.hpp"

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
         }
     });   
   EXPECT_EQ(timer_ticked, 0);
   spin(1100ms);
   EXPECT_EQ(timer_ticked, 10);   
}

TEST_F(NodeTest, OneOffTimerTest) {
   /// Test one-off timer 
   auto timer2 = node_->icey().create_timer(100ms, true);
   size_t timer_ticked = 0;
   EXPECT_EQ(timer_ticked, 0);
   timer2
     .then([&](size_t ticks) {
        EXPECT_EQ(timer_ticked, ticks);
        timer_ticked++;
     });
   spin(300ms);
   EXPECT_EQ(timer_ticked, 1);
}

TEST_F(TwoNodesFixture, PublisherTest) {
   /// Test one-off timer 
   auto timer = sender_->icey().create_timer(100ms);
   auto sub = receiver_->icey().create_subscription<std_msgs::msg::Float32>("/icey_test/sine_signal");

   EXPECT_FALSE(sub.impl()->has_value());

   timer
   .then([&](size_t ticks) -> std::optional<std_msgs::msg::Float32> {
         std_msgs::msg::Float32 float_val;
         float_val.data = ticks;
         if(ticks == 10)
         return {};
      return float_val;
   })
   .publish("/icey_test/sine_signal");
   
   std::size_t received_cnt = 0;
   sub.then([&](auto msg) {
      EXPECT_EQ(received_cnt, std::size_t(msg->data));
      received_cnt++;
   });

   spin(1100ms);
   EXPECT_EQ(received_cnt, 10);
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
