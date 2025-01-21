//// Support for the image_transport subscribers and publishers.
#pragma once

#ifndef ICEY_ROS2_WAS_INCLUDED
#error \
    "You must first include the <icey/icey_ros2.hpp> header before you can include the <icey/icey_image_transport.hpp> header."
#endif

#include <functional>
#include <tuple>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace icey {

/// See https://docs.ros.org/en/ros2_packages/jazzy/api/image_transport for API docs.
/// and https://github.com/ros-perception/image_common/blob/humble/image_transport
/// Note that we do not create a image_transport::ImageTransport object to avoid circular
/// dependency: https://github.com/ros-perception/image_common/issues/311

// An observable representing a camera image subscriber.
struct ImageTransportSubscriber : public Observable<sensor_msgs::msg::Image::ConstSharedPtr,
                                                    image_transport::TransportLoadException> {
  ImageTransportSubscriber(const std::string &base_topic_name, const std::string &transport,
                           const ROSAdapter::QoS qos, const rclcpp::SubscriptionOptions &options) {
    this->name = base_topic_name;
    auto this_obs = this->observable_;
    const auto cb = [this_obs](sensor_msgs::msg::Image::ConstSharedPtr image) {
      this_obs->resolve(image);
    };
    /// TODO do not capture this, save sub somewhere else
    this->attach_ = [this, this_obs, base_topic_name, transport, qos, cb,
                     options](ROSAdapter::NodeHandle &node) {
      try {
        subscriber_ = image_transport::create_subscription(&node, base_topic_name, cb, transport,
                                                           qos.get_rmw_qos_profile(), options);
      } catch (const image_transport::TransportLoadException &exception) {
        this_obs->reject(exception);
      }
    };
  }
  /// The image_transport subs/pubs use PIMPL, so we can hold them by value.
  image_transport::Subscriber subscriber_;
};

struct ImageTransportPublisher : public Observable<sensor_msgs::msg::Image::SharedPtr> {
  ImageTransportPublisher(const std::string &base_topic_name, const ROSAdapter::QoS qos,
                          const rclcpp::PublisherOptions &options = rclcpp::PublisherOptions()) {
    this->name = base_topic_name;
    auto this_obs = this->observable_;
    this->attach_ = [this_obs, base_topic_name, qos, options](ROSAdapter::NodeHandle &node) {
      image_transport::Publisher publisher = image_transport::create_publisher(
          &node, base_topic_name, qos.get_rmw_qos_profile());  /// TODO Humble did accept options
      this_obs->_register_handler([this_obs, publisher]() {
        const auto &image_msg = this_obs->value();  /// There can be no error
        publisher.publish(image_msg);
      });
    };
  }
};

// An observable representing a camera subscriber.
struct CameraSubscriber
    : public Observable<std::tuple<sensor_msgs::msg::Image::ConstSharedPtr,
                                   sensor_msgs::msg::CameraInfo::ConstSharedPtr>,
                        image_transport::TransportLoadException> {
  CameraSubscriber(const std::string &base_topic_name, const std::string &transport,
                   const ROSAdapter::QoS qos) {
    this->name = base_topic_name;
    auto this_obs = this->observable_;
    const auto cb = [this_obs](sensor_msgs::msg::Image::ConstSharedPtr image,
                               sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      this_obs->resolve(std::make_tuple(image, camera_info));
    };
    /// TODO do not capture this, save sub somewhere else
    this->attach_ = [this, this_obs, base_topic_name, transport, qos,
                     cb](ROSAdapter::NodeHandle &node) {
      try {
        subscriber_ = image_transport::create_camera_subscription(
            &node, base_topic_name, cb, transport, qos.get_rmw_qos_profile());
      } catch (const image_transport::TransportLoadException &exception) {
        this_obs->reject(exception);
      }
    };
  }

  /// The image_transport sub/pub use PIMPL, so we can hold them by value.
  image_transport::CameraSubscriber subscriber_;
};

struct CameraPublisher
    : public Observable<
          std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>> {
  CameraPublisher(const std::string &base_topic_name, const ROSAdapter::QoS qos) {
    this->name = base_topic_name;

    auto this_obs = this->observable_;
    this->attach_ = [this_obs, base_topic_name, qos](ROSAdapter::NodeHandle &node) {
      image_transport::CameraPublisher publisher = image_transport::create_camera_publisher(
          &node, base_topic_name, qos.get_rmw_qos_profile());
      this_obs->_register_handler([this_obs, publisher]() {
        const auto &new_value = this_obs->value();  /// There can be no error
        const auto &[image_msg, camera_info_msg] = new_value;
        /// TODO don't we need to copy here, why does this compile ??
        publisher.publish(image_msg, camera_info_msg);
      });
    };
  }
};

auto create_image_transport_subscription(
    const std::string &base_topic_name, const std::string &transport, const ROS2Adapter::QoS &qos,
    const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions()) {
  return g_state.get_context().create_observable<ImageTransportSubscriber>(base_topic_name,
                                                                           transport, qos, options);
};

auto create_image_transport_publisher(
    const std::string &base_topic_name, const std::string &transport, const ROS2Adapter::QoS &qos,
    const rclcpp::PublisherOptions &options = rclcpp::PublisherOptions()) {
  return g_state.get_context().create_observable<ImageTransportPublisher>(base_topic_name, qos,
                                                                          options);
};

auto create_camera_subscription(const std::string &base_topic_name, const std::string &transport,
                                const ROS2Adapter::QoS &qos = rclcpp::SensorDataQoS()) {
  return g_state.get_context().create_observable<CameraSubscriber>(base_topic_name, transport, qos);
};

auto create_camera_publisher(const std::string &base_topic_name,
                             const ROS2Adapter::QoS &qos = rclcpp::SensorDataQoS()) {
  return g_state.get_context().create_observable<CameraPublisher>(base_topic_name, qos);
};

}  // namespace icey