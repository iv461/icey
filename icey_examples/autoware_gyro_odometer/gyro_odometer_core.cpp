// Copyright 2015-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gyro_odometer_core.hpp"

#include <rclcpp/rclcpp.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>


namespace autoware::gyro_odometer
{

std::array<double, 9> transform_covariance(const std::array<double, 9> & cov)
{
  using COV_IDX = autoware::universe_utils::xyz_covariance_index::XYZ_COV_IDX;

  double max_cov = std::max({cov[COV_IDX::X_X], cov[COV_IDX::Y_Y], cov[COV_IDX::Z_Z]});

  std::array<double, 9> cov_transformed = {};
  cov_transformed.fill(0.);
  cov_transformed[COV_IDX::X_X] = max_cov;
  cov_transformed[COV_IDX::Y_Y] = max_cov;
  cov_transformed[COV_IDX::Z_Z] = max_cov;
  return cov_transformed;
}

sensor_msgs::msg::Imu transform_imu_msg(const sensor_msgs::msg::Imu &imu_msg, 
    const geometry_msgs::msg::TransformStamped &transform,
    const std::string &output_frame) {

    auto gyro = imu_msg;
    geometry_msgs::msg::Vector3Stamped angular_velocity;
    angular_velocity.header = gyro.header;
    angular_velocity.vector = gyro.angular_velocity;

    geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
    transformed_angular_velocity.header = transform.header;
    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);

    gyro.header.frame_id = output_frame;
    gyro.angular_velocity = transformed_angular_velocity.vector;
    gyro.angular_velocity_covariance = transform_covariance(gyro.angular_velocity_covariance);
    return gyro;
  }

geometry_msgs::msg::TwistStamped surpress_imu_bias(geometry_msgs::msg::TwistStamped twist_msg) {
     if (std::fabs(twist_msg.twist.angular.z) < 0.01 &&
        std::fabs(twist_msg.twist.linear.x) < 0.01) {
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;
    }
    return twist_msg;
}

geometry_msgs::msg::TwistStamped to_twist_wo_covariance(const geometry_msgs::msg::TwistWithCovarianceStamped &twist_with_cov) {
      geometry_msgs::msg::TwistStamped twist_msg;
      twist_msg.header = twist_with_cov.header;
      twist_msg.twist = twist_with_cov.twist.twist;
      return twist_msg;
}

using namespace std::chrono_literals;

GyroOdometerNode::GyroOdometerNode(const rclcpp::NodeOptions & node_options)
: icey::Node("gyro_odometer", node_options) {   

    auto message_timeout_sec_param = icey().declare_parameter<double>("message_timeout_sec");
    auto output_frame_param = icey().declare_parameter<std::string>("output_frame"); 

    /// We do not need QoS(100), i.e. keeping up to 100 messages in the RMW since we will use the synchronizer's queue
  /// Make subscribers, add timeout:
  auto twist_with_cov_sub = icey().create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "vehicle/twist_with_covariance").timeout(1s); // TODO use message_timeout_sec_param
  auto imu_sub = icey().create_subscription<sensor_msgs::msg::Imu>("imu").timeout(1s); // TODO use message_timeout_sec_param

  /// Handle timeout:
  twist_with_cov_sub.except([](auto current_time, auto msg_time, auto max_age){});
  imu_sub.except([](auto current_time, auto msg_time, auto max_age){});

  // Now for each IMU message, wait on the transform to become available (uses the tf2_ros::MessageFilter internally), with 5s tf timeout
  /// TODO after param init
  auto gyro_with_imu_to_base_tf = icey().synchronize_with_transform(imu_sub, output_frame_param.value(), rclcpp::Duration(5s));
  
  gyro_with_imu_to_base_tf.except([output_frame_param](const auto &tf_ex) {
    //diagnostics_->add_key_value("is_succeed_transform_imu", is_succeed_transform_imu);
    std::stringstream message;
    /*
      message << "Please publish TF " << output_frame_param.value() << " to "
              << gyro_queue_.front().header.frame_id;
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
      diagnostics_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    */

  });

    /// Now synchronize: Previously, this was done manually with imu_arrived_ and twist_arrived_flags and two callbacks. 
    /// All of this becomes unnecessary with ICEY.
    /// TODO pass queue_size
  //auto [twist_raw, twist] =
   icey().synchronize(twist_with_cov_sub, gyro_with_imu_to_base_tf)
    .then([output_frame_param](const auto &twist_msg, const auto &imu_msg) {
        //const auto [imu_msg, imu2base_tf] = gyro_with_tf;
        ///TODO get the tf and transform
        geometry_msgs::msg::TransformStamped imu2base_tf;
        auto transformed = transform_imu_msg(*imu_msg, imu2base_tf, output_frame_param.value());
    });
    /*
    .unpack()
    .except([]() {
        /// TODO publish over diagnostics debug info about the synchronizer: What has arrived, 
        /// and on which topics the synchronizer is still waiting
        ///diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    });
  
  .publish<geometry_msgs::msg::TwistStamped>("twist_raw", rclcpp::QoS{10});
  .publish<geometry_msgs::msg::TwistWithCovarianceStamped>("twist_with_covariance_raw", rclcpp::QoS{10});
  
  .publish<geometry_msgs::msg::TwistStamped>("twist", rclcpp::QoS{10});
  .publish<geometry_msgs::msg::TwistWithCovarianceStamped>("twist_with_covariance", rclcpp::QoS{10});
    */

    /*
  //diagnostics_ =
    std::make_unique<autoware::universe_utils::DiagnosticsInterface>(this, "gyro_odometer_status");
    */

  // TODO(YamatoAndo) createTimer
}


void GyroOdometerNode::concat_gyro_and_odometer()
{
  // check arrive first topic
  //diagnostics_->add_key_value("is_arrived_first_vehicle_twist", vehicle_twist_arrived_);
  //diagnostics_->add_key_value("is_arrived_first_imu", imu_arrived_);
 
  
  // check queue size
  //diagnostics_->add_key_value("vehicle_twist_queue_size", vehicle_twist_queue_.size());
  //diagnostics_->add_key_value("imu_queue_size", gyro_queue_.size());


 

  using COV_IDX_XYZ = autoware::universe_utils::xyz_covariance_index::XYZ_COV_IDX;
  using COV_IDX_XYZRPY = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  // calc mean, covariance
  double vx_mean = 0;
  geometry_msgs::msg::Vector3 gyro_mean{};
  double vx_covariance_original = 0;
  geometry_msgs::msg::Vector3 gyro_covariance_original{};
  for (const auto & vehicle_twist : vehicle_twist_queue_) {
    vx_mean += vehicle_twist.twist.twist.linear.x;
    vx_covariance_original += vehicle_twist.twist.covariance[0 * 6 + 0];
  }
  vx_mean /= static_cast<double>(vehicle_twist_queue_.size());
  vx_covariance_original /= static_cast<double>(vehicle_twist_queue_.size());

  for (const auto & gyro : gyro_queue_) {
    gyro_mean.x += gyro.angular_velocity.x;
    gyro_mean.y += gyro.angular_velocity.y;
    gyro_mean.z += gyro.angular_velocity.z;
    gyro_covariance_original.x += gyro.angular_velocity_covariance[COV_IDX_XYZ::X_X];
    gyro_covariance_original.y += gyro.angular_velocity_covariance[COV_IDX_XYZ::Y_Y];
    gyro_covariance_original.z += gyro.angular_velocity_covariance[COV_IDX_XYZ::Z_Z];
  }
  gyro_mean.x /= static_cast<double>(gyro_queue_.size());
  gyro_mean.y /= static_cast<double>(gyro_queue_.size());
  gyro_mean.z /= static_cast<double>(gyro_queue_.size());
  gyro_covariance_original.x /= static_cast<double>(gyro_queue_.size());
  gyro_covariance_original.y /= static_cast<double>(gyro_queue_.size());
  gyro_covariance_original.z /= static_cast<double>(gyro_queue_.size());

  // concat
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov;
  const auto latest_vehicle_twist_stamp = rclcpp::Time(vehicle_twist_queue_.back().header.stamp);
  const auto latest_imu_stamp = rclcpp::Time(gyro_queue_.back().header.stamp);
  if (latest_vehicle_twist_stamp < latest_imu_stamp) {
    twist_with_cov.header.stamp = latest_imu_stamp;
  } else {
    twist_with_cov.header.stamp = latest_vehicle_twist_stamp;
  }
  twist_with_cov.header.frame_id = gyro_queue_.front().header.frame_id;
  twist_with_cov.twist.twist.linear.x = vx_mean;
  twist_with_cov.twist.twist.angular = gyro_mean;

  // From a statistical point of view, here we reduce the covariances according to the number of
  // observed data
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::X_X] =
    vx_covariance_original / static_cast<double>(vehicle_twist_queue_.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Y_Y] = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Z_Z] = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::ROLL_ROLL] =
    gyro_covariance_original.x / static_cast<double>(gyro_queue_.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::PITCH_PITCH] =
    gyro_covariance_original.y / static_cast<double>(gyro_queue_.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::YAW_YAW] =
    gyro_covariance_original.z / static_cast<double>(gyro_queue_.size());

  publish_data(twist_with_cov);

  vehicle_twist_queue_.clear();
  gyro_queue_.clear();
}


}  // namespace autoware::gyro_odometer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gyro_odometer::GyroOdometerNode)