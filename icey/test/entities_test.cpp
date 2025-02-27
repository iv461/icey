#include "node_fixture.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

TEST_F(NodeTest, ParameterTest) {
   
   auto string_param = node_->icey().declare_parameter<std::string>("icey_test_my_param", "hello");

   EXPECT_TRUE(node_->has_parameter("icey_test_my_param"));
   EXPECT_EQ(node_->get_parameter("icey_test_my_param").get_value<std::string>(), "hello");
   EXPECT_EQ(string_param.value(), "hello");

   auto int64_param = node_->icey().declare_parameter<int64_t>("icey_test_my_param_int64", 7);
   EXPECT_TRUE(node_->has_parameter("icey_test_my_param_int64"));
   EXPECT_EQ(node_->get_parameter("icey_test_my_param_int64").get_value<int64_t>(), 7);
   EXPECT_EQ(int64_param.value(), 7);

   auto bool_param = node_->icey().declare_parameter<bool>("icey_test_my_param_bool", true);
   EXPECT_TRUE(node_->has_parameter("icey_test_my_param_bool"));
   EXPECT_EQ(node_->get_parameter("icey_test_my_param_bool").get_value<bool>(), true);
   EXPECT_EQ(bool_param.value(), true);

   auto array_param = node_->icey().declare_parameter<std::vector<double> >("icey_test_my_param_double_array", std::vector<double>{4., 7., 11., -1.1});
   EXPECT_TRUE(node_->has_parameter("icey_test_my_param_double_array"));
   std::vector<double> target_val{4., 7., 11., -1.1};
   EXPECT_EQ(node_->get_parameter("icey_test_my_param_double_array").get_value<std::vector<double>>(), target_val);
   EXPECT_EQ(array_param.value(), target_val);

   bool then_reacted = false;
   string_param
     .then([&](std::string new_val) {
         then_reacted = true;
         EXPECT_EQ(new_val, "hello2");
     });

   auto set_res = node_->set_parameter(rclcpp::Parameter("icey_test_my_param", std::string("hello2")));
   spin(100ms); /// Need to spin so that the parameter gets updated
   std::cout << "set_res: successful: " << set_res.successful  << ", reason: " << set_res.reason << std::endl;
   EXPECT_TRUE(then_reacted);
}

/// Here you declare in a single struct all parameters of the node:
struct NodeParameters {
   /// We can have regular fields :
   double amplitude{3};
 
   /// And as well parameters with constraints and a description:
   icey::ParameterStream<double> frequency{10., icey::Interval(0., 25.),
                                        std::string("The frequency of the sine")};
   
   icey::ParameterStream<std::string> mode{"single", icey::Set<std::string>({"single", "double", "pulse"})};
   /// We can also have nested structs with more parameters, they will be named others.max_amp, others.cov:
   struct OtherParams {
     double max_amp = 6.;
     std::vector<double> cov{0.444, 0., 0., 0.01};
   } others;
};

TEST_F(NodeTest, ParameterStructTest) {
    
   NodeParameters params;

   /// Test that no parameters were declared initially
   EXPECT_FALSE(node_->has_parameter("frequency"));
   EXPECT_FALSE(node_->has_parameter("amplitude"));
   EXPECT_FALSE(node_->has_parameter("mode"));
   
   EXPECT_FALSE(node_->has_parameter("others.max_amp"));
   EXPECT_FALSE(node_->has_parameter("others.cov"));

   std::unordered_set<std::string> fields_that_were_updated;
   /// Now declare the parameter struct and a callback that is called when any field updates: 
   node_->icey().declare_parameter_struct(params, [&](const std::string &changed_parameter) {
      fields_that_were_updated.emplace(changed_parameter);
   });
   
   /*
   I don't think it makes sense to specify exactly how the callback get's called initially, 
   it suffices to say it's called so that every parameter is initialized.
   std::unordered_set<std::string> target_updated1{"frequency", "mode"};
   EXPECT_EQ(fields_that_were_updated, target_updated1); /// At first, all the contrained parameters update
   */
   fields_that_were_updated.clear();

   EXPECT_TRUE(node_->has_parameter("amplitude"));
   EXPECT_EQ(node_->get_parameter("amplitude").get_value<double>(), 3.);
   
   EXPECT_TRUE(node_->has_parameter("frequency"));
   EXPECT_EQ(node_->get_parameter("frequency").get_value<double>(), 10.);
   EXPECT_EQ(double(params.frequency), 10.); /// Test implicit conversion access (the EXPECT_EQ macro doesn't do common_type tho)
   
   EXPECT_TRUE(node_->has_parameter("mode"));
   EXPECT_EQ(node_->get_parameter("mode").get_value<std::string>(), "single");
   EXPECT_EQ(std::string(params.mode), std::string("single")); /// Test implicit conversion access (the EXPECT_EQ macro doesn't do common_type tho)

   EXPECT_TRUE(node_->has_parameter("others.max_amp"));
   EXPECT_EQ(node_->get_parameter("others.max_amp").get_value<double>(), 6.);
   EXPECT_EQ(double(params.others.max_amp), 6.); /// Test implicit conversion access (the EXPECT_EQ macro doesn't do common_type tho)

   EXPECT_TRUE(node_->has_parameter("others.max_amp"));
   EXPECT_EQ(node_->get_parameter("others.max_amp").get_value<double>(), 6.);

   std::vector<double> target_cov{0.444, 0., 0., 0.01};
   EXPECT_TRUE(node_->has_parameter("others.cov"));
   EXPECT_EQ(node_->get_parameter("others.cov").get_value<std::vector<double>>(), target_cov);

   EXPECT_TRUE(fields_that_were_updated.empty()); 

   bool then_reacted = false;
   params.frequency
   .then([&](double new_val) {
      then_reacted = true;
      EXPECT_EQ(new_val, 2.5);
     });
     
   /// Test parameter setting:
   node_->set_parameter(rclcpp::Parameter("frequency", 2.5));
   spin(100ms); /// Need to spin so that the parameter gets updated
   EXPECT_TRUE(then_reacted);
   EXPECT_TRUE(fields_that_were_updated.contains("frequency"));
     
   then_reacted = false;
   fields_that_were_updated.clear();

   
   /// Test parameter update rejection: 
   node_->set_parameter(rclcpp::Parameter("frequency", 100.));
   spin(100ms); /// Need to spin so that the parameter gets updated
   EXPECT_FALSE(then_reacted);
   EXPECT_TRUE(fields_that_were_updated.empty());
   /// The parameter should have stayed the same:
   EXPECT_EQ(node_->get_parameter("frequency").get_value<double>(), 2.5);

   /// Test updating nested fields:
   node_->set_parameter(rclcpp::Parameter("others.max_amp", 120.));
   spin(100ms); /// Need to spin so that the parameter gets updated
   EXPECT_TRUE(fields_that_were_updated.contains("others.max_amp"));
   EXPECT_EQ(double(params.others.max_amp), 120.); 
   
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
   bool called_following = false;

   /// Here we catch timeout errors as well as unavailability of the service:
   auto handle_error = [&](const std::string& error_code) {
      /// Possible values for error_code are "SERVICE_UNAVAILABLE", or "INTERRUPTED" (in case
      /// we got interrupted while waiting for the service to become available) or
      /// "rclcpp::FutureReturnCode::INTERRUPTED" or "rclcpp::FutureReturnCode::TIMEOUT"
      last_error = error_code;
   };

   auto sub = receiver_->icey().create_timer(50ms)
          /// Build a request when the timer ticks
          .then([&](size_t) {
            auto request = std::make_shared<ExampleService::Request>();
            request->data = true;
            return request;
          })
          .call_service<ExampleService>("set_bool_service1", 50ms)
          .unwrap_or(handle_error)
          .then([&](ExampleService::Response::SharedPtr) {
            auto request = std::make_shared<ExampleService::Request>();
            request->data = false;
            called_following = true;
            return request;
          })
          .call_service<ExampleService>("set_bool_service2", 50ms)
          .unwrap_or(handle_error)
          .then([&](ExampleService::Response::SharedPtr) {
            auto request = std::make_shared<ExampleService::Request>();
            request->data = true;
            called_following = true;
            return request;
          })
          .call_service<ExampleService>("set_bool_service3", 50ms)
          .unwrap_or(handle_error)
          .then([&](ExampleService::Response::SharedPtr) {
            got_last = true;
            called_following = true;
            last_error = "";
          });

   /// Spin a bit to wait for the service timeout
   spin(150ms);

   ASSERT_FALSE(called_following);
   ASSERT_EQ(last_error, "SERVICE_UNAVAILABLE");

   auto service_cb = [](auto request) { 
      auto response = std::make_shared<ExampleService::Response>(); 
      response->success = !request->data; 
      return response;
   };
   sender_->icey().create_service<ExampleService>("set_bool_service1", service_cb);
   sender_->icey().create_service<ExampleService>("set_bool_service2", service_cb);
   sender_->icey().create_service<ExampleService>("set_bool_service3", service_cb);

   spin(250ms);

   EXPECT_TRUE(got_last);   
   ASSERT_EQ(last_error, "");
}

