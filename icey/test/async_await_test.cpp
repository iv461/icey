/// Tests whether all Streams work correctly using async/await syntax, i.e. coroutines.
#include "node_fixture.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

struct AsyncAwaitNodeTest : NodeTest {};

TEST_F(AsyncAwaitNodeTest, TimerTest) {

    [this]() -> icey::Stream<int> {
        size_t timer_ticked{0};
        auto timer = node_->icey().create_timer(100ms);
        
        EXPECT_EQ(timer_ticked, 0);
        
        while(true) {
            size_t ticks = co_await timer;
            EXPECT_EQ(timer_ticked, ticks);
            timer_ticked++;
            if(timer_ticked == 10) {
                break;
            }
        }
    
        EXPECT_EQ(timer_ticked, 10);

        co_return 0;
   }();
}

struct AsyncAwaitTwoNodeTest : TwoNodesFixture {};

TEST_F(AsyncAwaitTwoNodeTest, PubSubTest) {

    [this]() -> icey::Stream<int> {
        auto timer = sender_->icey().create_timer(100ms);

        timer
            .then([&](size_t ticks) -> std::optional<std_msgs::msg::Float32> {
                    std_msgs::msg::Float32 float_val;
                    float_val.data = ticks;
                return float_val;
            })
            .publish("/icey_test/sine_signal");

        //auto pub = sender_->icey().create_publisher<std_msgs::msg::Float32>("/icey_test/sine_signal", 1);
        auto sub = receiver_->icey().create_subscription<std_msgs::msg::Float32>("/icey_test/sine_signal");

        std::size_t received_cnt{0};

        while(received_cnt < 10) {
            auto msg = co_await sub;

            EXPECT_EQ(received_cnt, std::size_t(msg->data));
            received_cnt++;
        }
    
        EXPECT_EQ(received_cnt, 10);

        co_return 0;
   }();
}
