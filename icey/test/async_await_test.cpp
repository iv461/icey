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
        sender_->icey().create_timer(100ms)
            .then([](size_t ticks) {
                    std_msgs::msg::Float32 float_val;
                    float_val.data = ticks;
                return float_val;
            })
            .publish("/icey_test/sine_signal");

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

TEST_F(AsyncAwaitTwoNodeTest, ServiceTest) {

    [this]() -> icey::Stream<int> {

        // The response we are going to receive from the service call:
        using Response = ExampleService::Response::SharedPtr;
        
        auto service_cb = [&](auto request, auto response) { response->success = !request->data; };

    
        auto client1 = receiver_->icey().create_client<ExampleService>("set_bool_service1", 40ms);
        auto client2 = receiver_->icey().create_client<ExampleService>("set_bool_service2", 40ms);
        auto client3 = receiver_->icey().create_client<ExampleService>("set_bool_service3", 40ms);

        auto timer = receiver_->icey().create_timer(50ms);

        co_await timer;

        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;
        icey::Result<Response, std::string> result1 = co_await client1.call(request);

        EXPECT_TRUE(result1.has_error());
        EXPECT_EQ(result1.error(), "SERVICE_UNAVAILABLE ");

        /// Now create the services: 
        sender_->icey().create_service<ExampleService>("set_bool_service1").then(service_cb);
        sender_->icey().create_service<ExampleService>("set_bool_service2").then(service_cb);
        sender_->icey().create_service<ExampleService>("set_bool_service3").then(service_cb);

        result1 = co_await client1.call(request);

        EXPECT_TRUE(result1.has_value());
        EXPECT_FALSE(result1.value()->success);

        /// Then call the second service after we got the response from the first one:
        auto result2 = co_await client2.call(request);
        EXPECT_TRUE(result2.has_value());
        EXPECT_FALSE(result2.value()->success);

        auto result3 = co_await client3.call(request);
        EXPECT_TRUE(result3.has_value());
        EXPECT_FALSE(result3.value()->success);

        co_return 0;
    }();
}