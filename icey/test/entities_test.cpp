#include "node_fixture.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

TEST_F(NodeTest, ParameterTest) {
   
   auto string_param = node_->icey().declare_parameter<std::string>("icey_test_my_param", "hello");

   EXPECT_TRUE(node_->has_parameter("icey_test_my_param"));
   EXPECT_EQ(string_param.value(), "hello");

   auto int64_param = node_->icey().declare_parameter<int64_t>("icey_test_my_param_int64", 7);
   EXPECT_TRUE(node_->has_parameter("icey_test_my_param_int64"));
   EXPECT_EQ(int64_param.value(), 7);

   auto bool_param = node_->icey().declare_parameter<bool>("icey_test_my_param_bool", true);
   EXPECT_TRUE(node_->has_parameter("icey_test_my_param_bool"));
   EXPECT_EQ(bool_param.value(), true);

   auto array_param = node_->icey().declare_parameter<std::vector<double> >("icey_test_my_param_double_array", std::vector<double>{4., 7., 11., -1.1});
   EXPECT_TRUE(node_->has_parameter("icey_test_my_param_double_array"));
   std::vector<double> target_val{4., 7., 11., -1.1};
   EXPECT_EQ(array_param.value(), target_val);

   string_param
     .then([](auto new_val) {
         EXPECT_EQ(new_val, "hello2");
     });   

   node_->set_parameter(rclcpp::Parameter("icey_test_my_param", std::string("hello2")));
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

TEST_F(TwoNodesFixture, PubSubTest) {
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


TEST_F(TwoNodesFixture, ServiceTest) {
   std::string last_error;
   bool got_last = false;
   auto sub = receiver_->icey().create_timer(50ms)
          /// Build a request when the timer ticks
          .then([&](size_t) {
            auto request = std::make_shared<ExampleService::Request>();
            request->data = true;
            return request;
          })
          .call_service<ExampleService>("set_bool_service1", 50ms)
          .then([&](ExampleService::Response::SharedPtr) {
            auto request = std::make_shared<ExampleService::Request>();
            request->data = false;
            return request;
          })
          .call_service<ExampleService>("set_bool_service2", 50ms)
          .then([&](ExampleService::Response::SharedPtr) {
            auto request = std::make_shared<ExampleService::Request>();
            request->data = true;
            return request;
          })
          .call_service<ExampleService>("set_bool_service3", 50ms)
          .then([&](ExampleService::Response::SharedPtr) {
            got_last = true;
            last_error = "";
          })
          /// Here we catch timeout errors as well as unavailability of the service:
          .except([&](const std::string& error_code) {
            /// Possible values for error_code are "SERVICE_UNAVAILABLE", or "INTERRUPTED" (in case
            /// we got interrupted while waiting for the service to become available) or
            /// "rclcpp::FutureReturnCode::INTERRUPTED" or "rclcpp::FutureReturnCode::TIMEOUT"
//            RCLCPP_INFO_STREAM(node->get_logger(), "Service got error: " << error_code);
            last_error = error_code;
         });
   /// Spin a bit to wait for the service timeout
   spin(150ms);

   ASSERT_EQ(last_error, "SERVICE_UNAVAILABLE");

   auto &sender_ctx = sender_->icey();
   auto service_cb = [&](auto request, auto response) { response->success = !request->data; };
   sender_ctx.create_service<ExampleService>("set_bool_service1").then(service_cb);
   sender_ctx.create_service<ExampleService>("set_bool_service2").then(service_cb);
   sender_ctx.create_service<ExampleService>("set_bool_service3").then(service_cb);

   spin(250ms);

   EXPECT_TRUE(got_last);   
   ASSERT_EQ(last_error, "");
}

