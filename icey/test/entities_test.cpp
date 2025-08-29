/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved. 
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#include "node_fixture.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

TEST_F(NodeTest, rclcppTimeToChronoConversion) {
  // Define a fixed point in time (e.g., Unix timestamp: 1700000000 seconds + 123456789 nanoseconds)
  std::chrono::time_point<std::chrono::system_clock> fixed_time =
      std::chrono::time_point<std::chrono::system_clock>(std::chrono::seconds(1700000000)) +
      std::chrono::nanoseconds(123456789);

  // Perform conversions
  rclcpp::Time ros_time = icey::rclcpp_from_chrono(fixed_time);
  icey::Time converted_time = icey::rclcpp_to_chrono(ros_time);

  // Calculate the difference in nanoseconds
  auto diff =
      std::chrono::duration_cast<std::chrono::nanoseconds>(converted_time - fixed_time).count();

  // Check that the difference is exactly zero
  EXPECT_EQ(diff, 0) << "Timestamps converted wrongly";
}

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

  auto array_param = node_->icey().declare_parameter<std::vector<double>>(
      "icey_test_my_param_double_array", std::vector<double>{4., 7., 11., -1.1});
  EXPECT_TRUE(node_->has_parameter("icey_test_my_param_double_array"));
  std::vector<double> target_val{4., 7., 11., -1.1};
  EXPECT_EQ(
      node_->get_parameter("icey_test_my_param_double_array").get_value<std::vector<double>>(),
      target_val);
  EXPECT_EQ(array_param.value(), target_val);

  bool then_reacted = false;
  string_param.then([&](std::string new_val) {
    then_reacted = true;
    EXPECT_EQ(new_val, "hello2");
  });

  auto set_res =
      node_->set_parameter(rclcpp::Parameter("icey_test_my_param", std::string("hello2")));
  spin(100ms);  /// Need to spin so that the parameter gets updated
  EXPECT_TRUE(then_reacted);
}

/// Here you declare in a single struct all parameters of the node:
struct NodeParameters {
  /// We can have regular fields :
  double amplitude{3};

  /// And as well parameters with constraints and a description:
  icey::Parameter<double> frequency{10., icey::Interval(0., 25.),
                                    std::string("The frequency of the sine")};

  icey::Parameter<std::string> mode{"single",
                                    icey::Set<std::string>({"single", "double", "pulse"})};
  /// We can also have nested structs with more parameters, they will be named others.max_amp,
  /// others.cov:
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
  node_->icey().declare_parameter_struct(params, [&](const std::string& changed_parameter) {
    fields_that_were_updated.emplace(changed_parameter);
  });

  /*
  I don't think it makes sense to specify exactly how the callback get's called initially,
  it suffices to say it's called so that every parameter is initialized.
  std::unordered_set<std::string> target_updated1{"frequency", "mode"};
  EXPECT_EQ(fields_that_were_updated, target_updated1); /// At first, all the contained parameters
  update
  */
  fields_that_were_updated.clear();

  EXPECT_TRUE(node_->has_parameter("amplitude"));
  EXPECT_EQ(node_->get_parameter("amplitude").get_value<double>(), 3.);

  EXPECT_TRUE(node_->has_parameter("frequency"));
  EXPECT_EQ(node_->get_parameter("frequency").get_value<double>(), 10.);
  EXPECT_EQ(
      double(params.frequency),
      10.);  /// Test implicit conversion access (the EXPECT_EQ macro doesn't do common_type tho)

  EXPECT_TRUE(node_->has_parameter("mode"));
  EXPECT_EQ(node_->get_parameter("mode").get_value<std::string>(), "single");
  EXPECT_EQ(std::string(params.mode),
            std::string("single"));  /// Test implicit conversion access (the EXPECT_EQ macro
                                     /// doesn't do common_type tho)

  EXPECT_TRUE(node_->has_parameter("others.max_amp"));
  EXPECT_EQ(node_->get_parameter("others.max_amp").get_value<double>(), 6.);
  EXPECT_EQ(
      double(params.others.max_amp),
      6.);  /// Test implicit conversion access (the EXPECT_EQ macro doesn't do common_type tho)

  EXPECT_TRUE(node_->has_parameter("others.max_amp"));
  EXPECT_EQ(node_->get_parameter("others.max_amp").get_value<double>(), 6.);

  std::vector<double> target_cov{0.444, 0., 0., 0.01};
  EXPECT_TRUE(node_->has_parameter("others.cov"));
  EXPECT_EQ(node_->get_parameter("others.cov").get_value<std::vector<double>>(), target_cov);

  EXPECT_TRUE(fields_that_were_updated.empty());

  /*TODO(Ivo) I don't think it makes sense to support this
  bool then_reacted = false;
  params.frequency
  .then([&](double new_val) {
     then_reacted = true;
     EXPECT_EQ(new_val, 2.5);
    });*/

  /// Test parameter setting:
  node_->set_parameter(rclcpp::Parameter("frequency", 2.5));
  spin(100ms);  /// Need to spin so that the parameter gets updated
  EXPECT_TRUE(fields_that_were_updated.contains("frequency"));

  fields_that_were_updated.clear();

  /// Test parameter update rejection:
  node_->set_parameter(rclcpp::Parameter("frequency", 100.));
  spin(100ms);  /// Need to spin so that the parameter gets updated

  EXPECT_TRUE(fields_that_were_updated.empty());
  /// The parameter should have stayed the same:
  EXPECT_EQ(node_->get_parameter("frequency").get_value<double>(), 2.5);

  /// Test updating nested fields:
  node_->set_parameter(rclcpp::Parameter("others.max_amp", 120.));
  spin(100ms);  /// Need to spin so that the parameter gets updated
  EXPECT_TRUE(fields_that_were_updated.contains("others.max_amp"));
  EXPECT_EQ(double(params.others.max_amp), 120.);
}

TEST_F(TwoNodesFixture, PubSubTest) {
  /// Test one-off timer
  auto timer = sender_->icey().create_timer(100ms);
  auto sub =
      receiver_->icey().create_subscription<std_msgs::msg::Float32>("/icey_test/sine_signal");

  EXPECT_FALSE(sub.impl()->has_value());

  timer
      .then([&](size_t ticks) -> std::optional<std_msgs::msg::Float32> {
        std_msgs::msg::Float32 float_val;
        float_val.data = ticks;
        if (ticks == 10) return {};
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

TEST_F(TwoNodesFixture, TransformSubscriptionTest) {
  auto sub =
      receiver_->icey().create_transform_subscription("icey_test_frame1", "icey_test_frame3");

  EXPECT_FALSE(sub.impl()->has_value());

  icey::Time base_time{1700000000s};

  sender_->icey()
      .create_timer(100ms)
      .then([&](size_t ticks) -> std::optional<geometry_msgs::msg::TransformStamped> {
        geometry_msgs::msg::TransformStamped tf1;
        tf1.header.stamp = icey::rclcpp_from_chrono(base_time);  // sender_->get_clock()->now();
        tf1.header.frame_id = "icey_test_frame1";
        tf1.child_frame_id = "icey_test_frame2";
        tf1.transform.translation.x = 0. + 0.1 * ticks;
        if (ticks >= 10) return {};
        return tf1;
      })
      .publish_transform();

  sender_->icey()
      .create_timer(100ms)
      .then([&](size_t ticks) -> std::optional<geometry_msgs::msg::TransformStamped> {
        geometry_msgs::msg::TransformStamped tf1;
        tf1.header.stamp = icey::rclcpp_from_chrono(base_time);  // sender_->get_clock()->now();
        tf1.header.frame_id = "icey_test_frame2";
        tf1.child_frame_id = "icey_test_frame3";
        tf1.transform.rotation.z = std::sin(0.1 * ticks);
        tf1.transform.rotation.w = std::cos(0.1 * ticks);
        /// Since both transforms must be received for the first TF from 1 to 3, we got to publish
        /// one of the two once more
        if (ticks >= 11) return {};
        return tf1;
      })
      .publish_transform();

  std::size_t received_cnt = 0;
  sub.then([&](const geometry_msgs::msg::TransformStamped&) { received_cnt++; });

  spin(1500ms);
  EXPECT_EQ(received_cnt, 20);
}

TEST_F(NodeTest, TimerTest) {
  size_t timer_ticked{0};
  auto timer = node_->icey().create_timer(100ms);

  EXPECT_EQ(timer_ticked, 0);
  timer.then([&](size_t ticks) {
    EXPECT_EQ(timer_ticked, ticks);
    timer_ticked++;
    if (timer_ticked == 10) {
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
  timer2.then([&](size_t ticks) {
    EXPECT_EQ(timer_ticked, ticks);
    timer_ticked++;
  });
  spin(300ms);
  EXPECT_EQ(timer_ticked, 1);
}

/*TEST_F(TwoNodesFixture, ServiceClientTest) {
  {
    auto service_client =
      receiver_->icey().create_client<ExampleService>("icey_test_service");
    
    EXPECT_TRUE(receiver_->get_service_names_and_types().contains("icey_test_service"));
    /// Test that copying the client works and that also making an request through a client copy works after this  client copy was destroyed
    {
      auto client2 = service_client;
      
    }
    spin(1500ms);
  }
  /// Assert the client was destroyed
  EXPECT_TRUE(receiver_->get_service_names_and_types().empty());  
}*/