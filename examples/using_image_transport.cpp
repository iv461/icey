
/// Example showing how to use the image_transport Subscribers/Publishers for subscribing
/// simultaneously at the image as well as the CameraInfo message. The CameraInfo message holds
/// information about the pinhole camera model that is required for computer vision algorithms.
/// But one of the most
/// important use cases for image_transport is to compress a camera image before publishing it. This
/// is useful to be able to record images into a rosbag and prevent it from growing to tens of
/// gigabytes after just a few minutes.

// clang-format off
#include <icey/icey_ros2.hpp>             
#include <icey/icey_image_transport.hpp>  /// Include the message_transport header after (!) the ICEY header
// clang-format on

int main(int argc, char **argv) {
  /// Create an image_transport::CameraSubsciber. The second argument is the transport, i.e. the
  /// compression algorithm to use. Common ones are "raw" "theora" etc.
  // Note to class-based API users: call
  /// "icey().create_observable<icey::CameraSubscriber>("camera_center", "raw")" (this API is
  /// somewhat rough, it might change in the future).
  auto camera_center_sub =
      icey::create_camera_subscription("camera_center", "ffmpeg", rclcpp::SensorDataQoS());

  camera_center_sub
      ->then([](sensor_msgs::msg::Image::ConstSharedPtr image,
                sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Received image and info: ");
      })
      /// Optionally, you can handle the exception that the transport plugin could not be loaded
      /// (otherwise it will just throw the exception unhandled)
      ->except([](const image_transport::TransportLoadException &exception) {
        RCLCPP_ERROR_STREAM(icey::node->get_logger(),
                            "Could not load plugin, got exception: " << exception.what());
      });

  /// Create a regular image_transport::Subscriber to receive compressed images
  auto camera_left_sub =
      icey::create_image_transport_subscription("camera_left", "raw", rclcpp::SensorDataQoS());

  /// And now publish again compressed
  camera_left_sub
      ->then([](sensor_msgs::msg::Image::ConstSharedPtr image) {
        /// Process your image, e.g. paint it here:
        // As an example, we simply copy the image, output must be a shared_ptr
        return std::make_shared<sensor_msgs::msg::Image>(*image);
      })
      ->publish<icey::ImageTransportPublisher>("camera_left", rclcpp::SensorDataQoS());

  icey::spawn(argc, argv, "camera_subscriber_example");  /// Create and start node
}