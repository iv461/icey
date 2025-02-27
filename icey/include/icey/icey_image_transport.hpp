/// Support for the image_transport subscribers and publishers.
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
void assert_is_not_lifecycle_node(const NodeBookkeeping &book) {
  if (book.maybe_regular_node == nullptr)
    throw std::runtime_error(
        "You tried to use image_transport with a lifecycle node, unfortunately ROS Humble does not "
        "currently support image_transport with lifecycle nodes.");
}

struct ImageTransportSubscriberImpl {
  /// The image_transport subs/pubs use PIMPL, so we can hold them by value.
  image_transport::Subscriber subscriber;
};
// An stream representing a camera image subscriber.
struct ImageTransportSubscriber
    : public Stream<sensor_msgs::msg::Image::ConstSharedPtr,
                    image_transport::TransportLoadException, ImageTransportSubscriberImpl> {
  ImageTransportSubscriber(NodeBookkeeping &node, const std::string &base_topic_name,
                           const std::string &transport, const rclcpp::QoS qos,
                           const rclcpp::SubscriptionOptions &options) {
    assert_is_not_lifecycle_node(
        node);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                /// lifecycle nodes, so we can only assert this at runtime
    this->impl()->name = base_topic_name;
    const auto cb = [impl = this->impl()](sensor_msgs::msg::Image::ConstSharedPtr image) {
      impl->put_value(image);
    };
    try {
      this->impl()->subscriber =
          image_transport::create_subscription(node.maybe_regular_node, base_topic_name, cb,
                                               transport, qos.get_rmw_qos_profile(), options);
    } catch (const image_transport::TransportLoadException &exception) {
      this->impl()->put_error(exception);
    }
  }
};

struct ImageTransportPublisher : public Stream<sensor_msgs::msg::Image::SharedPtr> {
  ImageTransportPublisher(
      NodeBookkeeping &node, const std::string &base_topic_name, const rclcpp::QoS qos,
      const rclcpp::PublisherOptions & /*options*/ = rclcpp::PublisherOptions()) {
    this->impl()->name = base_topic_name;
    assert_is_not_lifecycle_node(
        node);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                /// lifecycle nodes, so we can only assert this at runtime
    image_transport::Publisher publisher = image_transport::create_publisher(
        node.maybe_regular_node, base_topic_name, qos.get_rmw_qos_profile());
    this->impl()->register_handler([publisher](const auto &new_state) {
      publisher.publish(new_state.value());  /// There can be no error
    });
  }
};

// An stream representing a camera subscriber.
struct CameraSubscriberImpl {
  /// The image_transport sub/pub use PIMPL, so we can hold them by value.
  image_transport::CameraSubscriber subscriber;
};

struct CameraSubscriber
    : public Stream<std::tuple<sensor_msgs::msg::Image::ConstSharedPtr,
                               sensor_msgs::msg::CameraInfo::ConstSharedPtr>,
                    image_transport::TransportLoadException, CameraSubscriberImpl> {
  CameraSubscriber(NodeBookkeeping &node, const std::string &base_topic_name,
                   const std::string &transport, const rclcpp::QoS qos) {
    this->impl()->name = base_topic_name;
    const auto cb = [impl = this->impl()](
                        sensor_msgs::msg::Image::ConstSharedPtr image,
                        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      impl->put_value(std::make_tuple(image, camera_info));
    };
    assert_is_not_lifecycle_node(
        node);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                /// lifecycle nodes, so we can only assert this at runtime
    try {
      this->impl()->subscriber = image_transport::create_camera_subscription(
          node.maybe_regular_node, base_topic_name, cb, transport, qos.get_rmw_qos_profile());
    } catch (const image_transport::TransportLoadException &exception) {
      this->impl()->put_error(exception);
    }
  }
};

struct CameraPublisher
    : public Stream<
          std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>> {
  CameraPublisher(NodeBookkeeping &node, const std::string &base_topic_name,
                  const rclcpp::QoS qos) {
    this->impl()->name = base_topic_name;
    assert_is_not_lifecycle_node(
        node);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                /// lifecycle nodes, so we can only assert this at runtime
    image_transport::CameraPublisher publisher = image_transport::create_camera_publisher(
        node.maybe_regular_node, base_topic_name, qos.get_rmw_qos_profile());
    this->impl()->register_handler([publisher](const auto &new_state) {
      const auto [image_msg, camera_info_msg] = new_state.value();  /// There can be no error;
      publisher.publish(image_msg, camera_info_msg);
    });
  }
};

/// Create an ImageTransportSubscriber using the given Context.
ImageTransportSubscriber create_image_transport_subscription(
    Context &ctx, const std::string &base_topic_name, const std::string &transport,
    const rclcpp::QoS &qos,
    const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions()) {
  return ctx.create_ros_stream<ImageTransportSubscriber>(base_topic_name, transport, qos, options);
};

/// Create an ImageTransportPublisher using the given Context.
ImageTransportPublisher create_image_transport_publisher(
    Context &ctx, const std::string &base_topic_name, const rclcpp::QoS &qos,
    const rclcpp::PublisherOptions &options = rclcpp::PublisherOptions()) {
  return ctx.create_ros_stream<ImageTransportPublisher>(base_topic_name, qos, options);
};
/// Create an CameraSubscriber using the given Context.
CameraSubscriber create_camera_subscription(Context &ctx, const std::string &base_topic_name,
                                            const std::string &transport,
                                            const rclcpp::QoS &qos = rclcpp::SensorDataQoS()) {
  return ctx.create_ros_stream<CameraSubscriber>(base_topic_name, transport, qos);
};
/// Create an CameraPublisher using the given Context.
CameraPublisher create_camera_publisher(Context &ctx, const std::string &base_topic_name,
                                        const rclcpp::QoS &qos = rclcpp::SensorDataQoS()) {
  return ctx.create_ros_stream<CameraPublisher>(base_topic_name, qos);
};

}  // namespace icey