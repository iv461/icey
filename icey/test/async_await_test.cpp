/// Tests whether all Streams work correctly using async/await syntax, i.e. coroutines.
#include "node_fixture.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

struct AsyncAwaitNodeTest : NodeTest {
    /// A check that the coroutine actually ran. Compilers (I have GCC 11.4) currently do not do exception handling in the coroutine code, so if it crashes, 
    /// the unit-test would just pass otherwise without us noticing it didn't acutally run.
    bool async_completed{false};
};

TEST_F(AsyncAwaitNodeTest, ParameterTest) {

    [this]() -> icey::Stream<int> {
        auto string_param = node_->icey().declare_parameter<std::string>("icey_test_my_param", "hello");

        EXPECT_TRUE(node_->has_parameter("icey_test_my_param"));
        EXPECT_EQ(node_->get_parameter("icey_test_my_param").get_value<std::string>(), "hello");
        EXPECT_EQ(string_param.value(), "hello");

        string_param
        .then([](std::string new_val) {
            EXPECT_EQ(new_val, "hello2");
        });
        node_->set_parameter(rclcpp::Parameter("icey_test_my_param", std::string("hello2")));

        async_completed = true;
        co_return 0;
   }();
   ASSERT_TRUE(async_completed);
}

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
        async_completed = true;
        co_return 0;
   }();
   ASSERT_TRUE(async_completed);
}

struct AsyncAwaitTwoNodeTest : TwoNodesFixture {
    /// A check that the coroutine actually ran. Compilers (I have GCC 11.4) currently do not do exception handling in the coroutine code, so if it crashes, 
    /// the unit-test would just pass otherwise without us noticing it didn't acutally run.
    bool async_completed{false};
};

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
        async_completed = true;
        co_return 0;
   }();
   ASSERT_TRUE(async_completed);
}

TEST_F(AsyncAwaitTwoNodeTest, PubSubTest2) {

    [this]() -> icey::Stream<int> {  
        std::size_t received_cnt{0};

        receiver_->icey().create_subscription<std_msgs::msg::Float32>("/icey_test/sine_signal")
            .then([&](auto msg) {
                EXPECT_EQ(received_cnt, std::size_t(msg->data));
                received_cnt++;
            });
            
        auto timer = sender_->icey().create_timer(100ms);

        auto pub = sender_->icey().create_publisher<std_msgs::msg::Float32>("/icey_test/sine_signal", 1);

        auto coro = [timer]() -> icey::Stream<std_msgs::msg::Float32> {
            size_t ticks = co_await timer;
            std_msgs::msg::Float32 float_val;
            float_val.data = ticks;
            co_return float_val;
        };

        for(int i= 0; i < 10; i++) {
            pub.publish(co_await coro());
        }
        
        /// We do not await till the published message was received. 
        /// Since the ACK from DDS is exposed in rclcpp via PublisherBase::wait_for_all_acked, we could implement a co_await publish(msg).
        EXPECT_EQ(received_cnt, 9);
        async_completed = true;
        co_return 0;
   }();
   ASSERT_TRUE(async_completed);
}

TEST_F(AsyncAwaitTwoNodeTest, ServiceTest) {

    [this]() -> icey::Stream<int> {

        // The response we are going to receive from the service call:
        using Response = ExampleService::Response::SharedPtr;
        
        auto client1 = receiver_->icey().create_client<ExampleService>("set_bool_service1", 40ms);
        auto client2 = receiver_->icey().create_client<ExampleService>("set_bool_service2", 40ms);
        auto client3 = receiver_->icey().create_client<ExampleService>("set_bool_service3", 40ms);

        auto timer = receiver_->icey().create_timer(50ms);

        co_await timer;

        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;
        icey::Result<Response, std::string> result1 = co_await client1.call(request);
        
        EXPECT_TRUE(result1.has_error());
        EXPECT_EQ(result1.error(), "SERVICE_UNAVAILABLE");
        
        /// Now create the service servers: 
        auto service_cb = [](auto request) { 
            auto response = std::make_shared<ExampleService::Response>(); 
            response->success = !request->data; 
            return response;
        };
        sender_->icey().create_service<ExampleService>("set_bool_service1", service_cb);
        sender_->icey().create_service<ExampleService>("set_bool_service2", service_cb);
        sender_->icey().create_service<ExampleService>("set_bool_service3", service_cb);

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
        async_completed = true;
        co_return 0;
    }(); 
    ASSERT_TRUE(async_completed);
}