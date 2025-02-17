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

#ifndef GYRO_ODOMETER_CORE_HPP_
#define GYRO_ODOMETER_CORE_HPP_


#include "autoware/localization_util/diagnostics_module.hpp"
#include "autoware/universe_utils/ros/msg_covariance.hpp"
#include "autoware/universe_utils/ros/transform_listener.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/transform_datatypes.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <deque>
#include <memory>
#include <string>

#include <icey/icey.hpp>

namespace autoware::gyro_odometer
{//

class GyroOdometerNode : public icey::Node
{
private:
  using COV_IDX = autoware::universe_utils::xyz_covariance_index::XYZ_COV_IDX;

public:
  explicit GyroOdometerNode(const rclcpp::NodeOptions & node_options);

private:
  void concat_gyro_and_odometer();
  void publish_data(const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw);

  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> vehicle_twist_queue_;
  std::deque<sensor_msgs::msg::Imu> gyro_queue_;

  std::unique_ptr<autoware::localization_util::DiagnosticsModule> diagnostics_;
};

}  // namespace autoware::gyro_odometer

#endif  // GYRO_ODOMETER_CORE_HPP_