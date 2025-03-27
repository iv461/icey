/// Support for the image_transport subscriptions and publishers.
/// See https://docs.ros.org/en/ros2_packages/jazzy/api/image_transport and
/// https://github.com/ros-perception/image_common/blob/humble/image_transport for API docs.
/// Note that we do not create a image_transport::ImageTransport object to avoid circular
/// dependency: https://github.com/ros-perception/image_common/issues/311
#pragma once

#include <functional>
#include <icey/icey.hpp>
#include <tuple>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace icey {

/// Image_transport does not yet support lifecycle_nodes:
void assert_is_not_lifecycle_node(Context &context) {
  if (context.node_base().is_lifecycle_node())
    throw std::runtime_error(
        "You tried to use image_transport with a lifecycle node, unfortunately ROS Humble does not "
        "currently support image_transport with lifecycle nodes.");
}

struct ImageTransportSubscriptionImpl {
  /// The image_transport subs/pubs use PIMPL, so we can hold them by value.
  image_transport::Subscriber subscription;
};
// An stream representing a camera image subscription.
struct ImageTransportSubscription
    : public Stream<sensor_msgs::msg::Image::ConstSharedPtr,
                    image_transport::TransportLoadException, ImageTransportSubscriptionImpl> {
  using Base = Stream<sensor_msgs::msg::Image::ConstSharedPtr,
                      image_transport::TransportLoadException, ImageTransportSubscriptionImpl>;
  ImageTransportSubscription(Context &context, const std::string &base_topic_name,
                           const std::string &transport, const rclcpp::QoS qos,
                           const rclcpp::SubscriptionOptions &options = {})
      : Base(context) {
    assert_is_not_lifecycle_node(
        context);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                /// lifecycle nodes, so we can only assert this at runtime
    const auto cb = [impl = this->impl()](sensor_msgs::msg::Image::ConstSharedPtr image) {
      impl->put_value(image);
    };
    try {
      this->impl()->subscription =
          image_transport::create_subscription(&context.node_base().as_node(), base_topic_name, cb,
                                               transport, qos.get_rmw_qos_profile(), options);
    } catch (const image_transport::TransportLoadException &exception) {
      this->impl()->put_error(exception);
    }
  }
};

struct ImageTransportPublisher : public Stream<sensor_msgs::msg::Image::SharedPtr> {
  using Base = Stream<sensor_msgs::msg::Image::SharedPtr>;
  ImageTransportPublisher(Context &context, const std::string &base_topic_name,
                          const rclcpp::QoS qos,
                          const rclcpp::PublisherOptions & /*options*/ = {})
      : Base(context) {
    assert_is_not_lifecycle_node(
        context);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                /// lifecycle nodes, so we can only assert this at runtime
    image_transport::Publisher publisher = image_transport::create_publisher(
        &context.node_base().as_node(), base_topic_name, qos.get_rmw_qos_profile());
    this->impl()->register_handler([publisher](const auto &new_state) {
      publisher.publish(new_state.value());  /// There can be no error
    });
  }
};

// An stream representing a camera subscription.
struct CameraSubscriptionImpl {
  /// The image_transport sub/pub use PIMPL, so we can hold them by value.
  image_transport::CameraSubscriber subscription;
};

struct CameraSubscription
    : public Stream<std::tuple<sensor_msgs::msg::Image::ConstSharedPtr,
                               sensor_msgs::msg::CameraInfo::ConstSharedPtr>,
                    image_transport::TransportLoadException, CameraSubscriptionImpl> {
  using Base = Stream<std::tuple<sensor_msgs::msg::Image::ConstSharedPtr,
                                 sensor_msgs::msg::CameraInfo::ConstSharedPtr>,
                      image_transport::TransportLoadException, CameraSubscriptionImpl>;
  CameraSubscription(Context &context, const std::string &base_topic_name,
                   const std::string &transport, const rclcpp::QoS qos)
      : Base(context) {
    const auto cb = [impl = this->impl()](
                        sensor_msgs::msg::Image::ConstSharedPtr image,
                        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      impl->put_value(std::make_tuple(image, camera_info));
    };
    assert_is_not_lifecycle_node(
        context);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                /// lifecycle nodes, so we can only assert this at runtime
    try {
      this->impl()->subscription = image_transport::create_camera_subscription(
          &context.node_base().as_node(), base_topic_name, cb, transport, qos.get_rmw_qos_profile());
    } catch (const image_transport::TransportLoadException &exception) {
      this->impl()->put_error(exception);
    }
  }
};

struct CameraPublisher
    : public Stream<
          std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>> {
  using Base = Stream<
      std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>>;
  CameraPublisher(Context &context, const std::string &base_topic_name, const rclcpp::QoS qos)
      : Base(context) {
    assert_is_not_lifecycle_node(
        context);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                /// lifecycle nodes, so we can only assert this at runtime
    image_transport::CameraPublisher publisher = image_transport::create_camera_publisher(
        &context.node_base().as_node(), base_topic_name, qos.get_rmw_qos_profile());
    this->impl()->register_handler([publisher](const auto &new_state) {
      const auto [image_msg, camera_info_msg] = new_state.value();  /// There can be no error;
      publisher.publish(image_msg, camera_info_msg);
    });
  }
};

}  // namespace icey