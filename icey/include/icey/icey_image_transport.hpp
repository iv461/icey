/// Support for the image_transport subscribers and publishers.
/// See https://docs.ros.org/en/ros2_packages/jazzy/api/image_transport and
/// https://github.com/ros-perception/image_common/blob/humble/image_transport for API docs.
/// Note that we do not create a image_transport::ImageTransport object to avoid circular
/// dependency: https://github.com/ros-perception/image_common/issues/311
#pragma once

#ifndef ICEY_ROS2_WAS_INCLUDED
#error \
    "You must first include the <icey/icey.hpp> header before you can include the <icey/icey_image_transport.hpp> header."
#endif

#include <functional>
#include <tuple>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace icey {

/// Image_transport does not yet support lifecycle_nodes:
void assert_is_not_lifecycle_node(const NodeBookkeeping &book) {
  if (book.node_.maybe_regular_node == nullptr)
    throw std::runtime_error(
        "You tried to use image_transport with a lifecycle node, unfortunately ROS Humble does not "
        "currently support image_transport with lifecycle nodes.");
}

struct ImageTransportSubscriberImpl : public NodeAttachable {
  /// The image_transport subs/pubs use PIMPL, so we can hold them by value.
  image_transport::Subscriber subscriber;
};
// An observable representing a camera image subscriber.
struct ImageTransportSubscriber
    : public Stream<sensor_msgs::msg::Image::ConstSharedPtr,
                    image_transport::TransportLoadException, ImageTransportSubscriberImpl> {
  ImageTransportSubscriber(const std::string &base_topic_name, const std::string &transport,
                           const rclcpp::QoS qos, const rclcpp::SubscriptionOptions &options) {
    this->impl()->name = base_topic_name;
    const auto cb = [impl = this->impl()](sensor_msgs::msg::Image::ConstSharedPtr image) {
      impl->resolve(image);
    };
    this->impl()->attach_ = [impl = this->impl(), base_topic_name, transport, qos, cb,
                             options](NodeBookkeeping &node) {
      assert_is_not_lifecycle_node(
          node);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                  /// lifecycle nodes, so we can only assert this at runtime
      try {
        impl->subscriber =
            image_transport::create_subscription(node.node_.maybe_regular_node, base_topic_name, cb,
                                                 transport, qos.get_rmw_qos_profile(), options);
      } catch (const image_transport::TransportLoadException &exception) {
        impl->reject(exception);
      }
    };
  }
};

struct ImageTransportPublisher : public Stream<sensor_msgs::msg::Image::SharedPtr> {
  ImageTransportPublisher(const std::string &base_topic_name, const rclcpp::QoS qos,
                          const rclcpp::PublisherOptions &options = rclcpp::PublisherOptions()) {
    this->impl()->name = base_topic_name;
    this->impl()->attach_ = [impl = this->impl(), base_topic_name, qos,
                             options](NodeBookkeeping &node) {
      assert_is_not_lifecycle_node(
          node);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                  /// lifecycle nodes, so we can only assert this at runtime
      image_transport::Publisher publisher = image_transport::create_publisher(
          node.node_.maybe_regular_node, base_topic_name, qos.get_rmw_qos_profile());
      impl->register_handler([publisher](const auto &new_state) {
        const auto &image_msg = new_state.value();  /// There can be no error
        publisher.publish(image_msg);
      });
    };
  }
};

// An observable representing a camera subscriber.
struct CameraSubscriberImpl : public NodeAttachable {
  /// The image_transport sub/pub use PIMPL, so we can hold them by value.
  image_transport::CameraSubscriber subscriber;
};

struct CameraSubscriber
    : public Stream<std::tuple<sensor_msgs::msg::Image::ConstSharedPtr,
                               sensor_msgs::msg::CameraInfo::ConstSharedPtr>,
                    image_transport::TransportLoadException, CameraSubscriberImpl> {
  CameraSubscriber(const std::string &base_topic_name, const std::string &transport,
                   const rclcpp::QoS qos) {
    this->impl()->name = base_topic_name;
    const auto cb = [impl = this->impl()](
                        sensor_msgs::msg::Image::ConstSharedPtr image,
                        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      impl->resolve(std::make_tuple(image, camera_info));
    };
    this->impl()->attach_ = [impl = this->impl(), base_topic_name, transport, qos,
                             cb](NodeBookkeeping &node) {
      assert_is_not_lifecycle_node(
          node);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                  /// lifecycle nodes, so we can only assert this at runtime
      try {
        impl->subscriber = image_transport::create_camera_subscription(
            node.node_.maybe_regular_node, base_topic_name, cb, transport,
            qos.get_rmw_qos_profile());
      } catch (const image_transport::TransportLoadException &exception) {
        impl->reject(exception);
      }
    };
  }
};

struct CameraPublisher
    : public Stream<
          std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>> {
  CameraPublisher(const std::string &base_topic_name, const rclcpp::QoS qos) {
    this->impl()->name = base_topic_name;
    this->impl()->attach_ = [impl = this->impl(), base_topic_name, qos](NodeBookkeeping &node) {
      assert_is_not_lifecycle_node(
          node);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                  /// lifecycle nodes, so we can only assert this at runtime
      image_transport::CameraPublisher publisher = image_transport::create_camera_publisher(
          node.node_.maybe_regular_node, base_topic_name, qos.get_rmw_qos_profile());
      impl->register_handler([publisher](const auto &new_state) {
        const auto [image_msg, camera_info_msg] = new_state.value();  /// There can be no error;
        /// TODO Maybe clarify message ownership, this publish-function is inconsistent with regular
        /// publishers because it accepts share_ptr<M>
        publisher.publish(image_msg, camera_info_msg);
      });
    };
  }
};

auto create_image_transport_subscription(
    const std::string &base_topic_name, const std::string &transport, const rclcpp::QoS &qos,
    const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions()) {
  return create_observable<ImageTransportSubscriber>(base_topic_name, transport, qos, options);
};

auto create_image_transport_publisher(
    const std::string &base_topic_name, const rclcpp::QoS &qos,
    const rclcpp::PublisherOptions &options = rclcpp::PublisherOptions()) {
  return create_observable<ImageTransportPublisher>(base_topic_name, qos, options);
};

auto create_camera_subscription(const std::string &base_topic_name, const std::string &transport,
                                const rclcpp::QoS &qos = rclcpp::SensorDataQoS()) {
  return create_observable<CameraSubscriber>(base_topic_name, transport, qos);
};

auto create_camera_publisher(const std::string &base_topic_name,
                             const rclcpp::QoS &qos = rclcpp::SensorDataQoS()) {
  return create_observable<CameraPublisher>(base_topic_name, qos);
};

}  // namespace icey