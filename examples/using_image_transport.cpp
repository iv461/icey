
/// Example showing how to use the image_transport CameraSubscriber/Publisher for subscribing 
/// simultaneously at the image as well as the CameraInfo message holding information about the pinhole camera model 
/// that is required for computer vision algorithms. 
/// But one of the most important use cases for image_transport is compressing a camera image 
/// before publishing it to be able to record it into a rosbag to prevent that it becomes dozens of gigabytes 
/// large after a couple of minutes.

#include <icey/icey_ros2.hpp>
#include <icey/icey_image_transport.hpp> /// Include the message_transport header after (!) the ICEY header

int main(int argc, char **argv) {

    /// The second argument is the transport, i.e. the compression algorithm to use. Common ones are "raw", 
    /// Compressed, ffmpeg etc.
    /// Note to class-based API users: call "icey().create_observable<CameraSubscriber>("camera_center", "raw")" (this API is somewhat rough, it might change in the future).
    auto camera_center_sub = icey::create_camera_subscription("camera_center", "raw", rclcpp::SensorDataQoS());

    camera_center_sub->then([](sensor_msgs::msg::Image::ConstSharedPtr image, 
                                    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info){
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Received image and info: ");

    });

    icey::spawn(argc, argv, "camera_subscriber_example"); /// Create and start node
}