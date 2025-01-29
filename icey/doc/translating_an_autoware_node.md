# Translating teh autoware_gyro_odometer Node to use ICEY 

In this tutorial, we explain how a node using the regular ROS 2 API can be translated to use ICEY. We use as an example the node `autoware_gyro_odometer`, a node in the Autoware autonomous driving stack responsible for producing a odometry-message `geometry_msgs::Twist` based on IMU data. 

We choose this node because: 

- Is is neither trivial nor has an excessive amount of algorithm complexity
- It has an interesting communication pattern of synchronizing two topics of different frequency, buffering and detecting timeout

TODO info about universe commit version etc. 

# Identifying the communication pattern 
First, we look at the subscriptions: 
We subscribe to two topics, TODO
and look at their callbacks: 

[Code Listing 1]
```cpp
void GyroOdometerNode::callback_vehicle_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr vehicle_twist_msg_ptr)
{
  diagnostics_->clear();
  diagnostics_->add_key_value(
    "topic_time_stamp",
    static_cast<rclcpp::Time>(vehicle_twist_msg_ptr->header.stamp).nanoseconds());

  vehicle_twist_arrived_ = true;
  latest_vehicle_twist_ros_time_ = vehicle_twist_msg_ptr->header.stamp;
  vehicle_twist_queue_.push_back(*vehicle_twist_msg_ptr);
  concat_gyro_and_odometer();

  diagnostics_->publish(vehicle_twist_msg_ptr->header.stamp);
}
```

and the IMu callback looks very similar: TODO 
[Code Listing 2]

```cpp

void GyroOdometerNode::callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  diagnostics_->clear();
  diagnostics_->add_key_value(
    "topic_time_stamp", static_cast<rclcpp::Time>(imu_msg_ptr->header.stamp).nanoseconds());

  imu_arrived_ = true;
  latest_imu_ros_time_ = imu_msg_ptr->header.stamp;
  gyro_queue_.push_back(*imu_msg_ptr);
  concat_gyro_and_odometer();

  diagnostics_->publish(imu_msg_ptr->header.stamp);
}
```
as we see, both callbacks call and the end `concat_gyro_and_odometer()`, the function fusing both messages. This reveals that actually these topics need to be fused. The queues that are manually managed here indicate that a synchronization is needed. 

We see also the manually managed flags `vehicle_twist_arrived_` and `vehicle_twist_arrived_`, we will remove them since by using ICEY they become unnecessary. 

# Notifying on waiting 

We see that it is published much info on the diagnostics topic to ease debugging in case not all messages arrived. 

TODO 


# Checking for timeout 

Next, we look at this part of the code, where we check for timeouts: 

[Function `concat_gyro_and_odometer`]
```cpp
const double vehicle_twist_dt =
    std::abs((this->now() - latest_vehicle_twist_ros_time_).seconds());
  const double imu_dt = std::abs((this->now() - latest_imu_ros_time_).seconds());
  //diagnostics_->add_key_value("vehicle_twist_time_stamp_dt", vehicle_twist_dt);
  //diagnostics_->add_key_value("imu_time_stamp_dt", imu_dt);
  if (vehicle_twist_dt > message_timeout_sec_) {
    const std::string message = fmt::format(
      "Vehicle twist msg is timeout. vehicle_twist_dt: {}[sec], tolerance {}[sec]",
      vehicle_twist_dt, message_timeout_sec_);
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message);
    //diagnostics_->update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);

    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }
```
We see that the timestamp present in the message header (that is passed indirectly via `latest_vehicle_twist_ros_time_`, see Listing 1) is compared to the current time.

In case it is too old, it is not passed through but an error message is printed. We can achieve the same in ICEY using the `timeout`-filter that works the same: 

```cpp
auto message_timeout_sec_param = icey().declare_parameter<double>("message_timeout_sec");

auto imu_sub_with_timeout = imu_sub
    .timeout(message_timeout_sec_param) 
    .then([](const auto &msg) {
        /// Consume the message in case it is not too old
    });

aut imu_sub_with_timeout.except([](auto message_timestamp, auto current_time, auto max_age) {
    auto vehicle_twist_dt = (current_time - message_timestamp).seconds();
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(),        *this->get_clock(), 1000, fmt::format(
            "Vehicle twist msg is timeout. vehicle_twist_dt: {}[sec], tolerance {}[sec]",
            vehicle_twist_dt, max_age));
    });
```

TODO for the transforming we have the frame_id based on data pattern, for this we need to support the `tf2_ros::MessageFilter<M>`