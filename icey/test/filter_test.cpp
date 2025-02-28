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
     auto s1 = node_->icey().create_timer(43ms).
        then([](size_t ticks) {
            return "s1_" + std::to_string(ticks);
        });
    auto s2 = node_->icey().create_timer(53ms)
        .then([](size_t ticks) {
            return "s2_" + std::to_string(ticks);
        });
    auto s3 = node_->icey().create_timer(67ms)
        .then([](size_t ticks) {
            return "s3_" + std::to_string(ticks);
        });
    
    std::vector<std::string> received_values;
    icey::any(s1, s2, s3).
        then([&](std::string val) {
            received_values.push_back(val);
        });

    std::vector<std::string> target_values{"s1_1", "s1_2", "s1_3", "s2_1", "s2_2", "s2_3", 
                "s3_1", "s3_2", "s3_3"};

    EXPECT_EQ(received_values, target_values);
}

TEST_F(NodeTest, UnpackTest) {
   auto timer = node_->icey().create_timer(50ms);

   auto [double_stream, tf_stream2, string_stream] = 
        timer
        .then([&](size_t ticks) {
            auto val = std::make_tuple(double(ticks), geometry_msgs::msg::TransformStamped{}, std::string{});
            if(ticks >= 3) {
                timer.impl()->timer->cancel();
                return std::optional<decltype(val)>{};
            }
            return std::optional(val);
        }).unpack();
    
    size_t double_stream_num_called = 0;
    size_t tf_stream2_num_called = 0;
    size_t string_stream_num_called = 0;

    double_stream
        .then([&](double msg) { 
            double_stream_num_called++;
         });
    
    tf_stream2
        .then([&](geometry_msgs::msg::TransformStamped msg) { 
            tf_stream2_num_called++;
        });
    
    string_stream
        .then([&](std::string msg) {             
            string_stream_num_called++;
        });

    spin(250ms);

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