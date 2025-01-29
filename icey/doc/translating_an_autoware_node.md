# Translating teh autoware_gyro_odometer Node to use ICEY 

In this tutorial, we explain how a node using the regular ROS 2 API can be translated to use ICEY. We use as an example the node `autoware_gyro_odometer`, a node in the Autoware autonomous driving stack responsible for producing a odometry-message `geometry_msgs::Twist` based on IMU data. 

We choose this node because: 

- Is is neither trivial nor has an excessive amount of algorithm complexity
- It has an interesting communication pattern of synchronizing two topics of different frequency and required buffering


# Identifying the communication pattern 
First, we look at the subscriptions: 
We subscribe to two topics, TODO
and look at their callbacks: 

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
