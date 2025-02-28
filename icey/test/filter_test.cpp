#include "node_fixture.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

TEST_F(NodeTest, SyncWithReferenceTest) {

}

TEST_F(NodeTest, SyncApproxTimeTest) {
    
}

TEST_F(NodeTest, SynchronizeWithTransformTest) {
    
}

TEST_F(NodeTest, AnyTest) {
    
}

TEST_F(NodeTest, UnpackTest) {
   auto timer = node_->icey().create_timer(50ms);
   size_t timer_ticked{0};
   EXPECT_EQ(timer_ticked, 0);

   auto [double_stream, tf_stream2, string_stream] = 
        timer
        .then([&](size_t ticks) {
            if(ticks > 3) {
                timer.impl()->timer->cancel();
            }
            ASSERT_FALSE(ticks % 3);
            timer_ticked++;
            return std::make_tuple(double{}, geometry_msgs::msg::TransformStamped{}, std::string{});
        }).unpack();

    size_t double_stream_num_called = 0;
    size_t tf_stream2_num_called = 0;
    size_t string_stream_num_called = 0;

    auto double_stream
        .then([&](auto msg) { 
            double_stream_num_called++;
         });
    
    auto tf_stream2
        .then([&](auto msg) { 
            tf_stream2_num_called++;
        });
    
    auto string_stream
        .then([&](auto msg) { 
            string_stream_num_called++;
        });

    spin(1100ms);

    EXPECT_EQ(double_stream_num_called, 3);
    EXPECT_EQ(tf_stream2_num_called, 3);
    EXPECT_EQ(string_stream_num_called, 3);
}

TEST_F(NodeTest, FilterTest) {
   auto timer = node_->icey().create_timer(50ms);
    
   size_t timer_ticked{0};
   EXPECT_EQ(timer_ticked, 0);

   timer
     .filter([&](size_t ticks) { return !(ticks % 3); })
     .then([&](size_t ticks) {
         if(ticks > 10) {
             timer.impl()->timer->cancel();
             return;
        }
        ASSERT_FALSE(ticks % 3);
        timer_ticked++;
    });

    spin(1100ms);
    /// 0, 3, 6, 9, four times
    EXPECT_EQ(timer_ticked, 4);
}

TEST_F(TwoNodesFixture, BufferTest) {
    
}