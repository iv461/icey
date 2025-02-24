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
        
        while(rclcpp::ok()) {
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
