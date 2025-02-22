#include "node_fixture.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

TEST_F(NodeTest, FilterTest) {
    auto timer = node_->icey().create_timer(50ms);
    
   size_t timer_ticked{0};
   EXPECT_EQ(timer_ticked, 0);

   timer
     .filter([&](size_t ticks) {
        bool cond = (ticks % 3) == 0;
        std::cout << "tikcs" << ticks << " condition is: " << cond << std::endl;
        return cond;
     })
     .then([&](size_t ticks) {
         if(ticks > 10) {
             timer.impl()->timer->cancel();
             return;
         }
        std::cout << "tiks" << ticks << " called  " << std::endl;

        ASSERT_FALSE(ticks % 3);
        timer_ticked++;
    });   
    spin(1100ms);
    EXPECT_EQ(timer_ticked, 4);  
}
