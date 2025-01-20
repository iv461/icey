
/// Example showing how to use the image_transport CameraSubscriber/Publisher for subscribing 
/// simultaneously at the image as well as the CameraInfo message holding information about the pinhole camera model 
/// that is required for computer vision algorithms. 

#include <icey/icey_ros2.hpp>
#include <icey/icey_image_transport.hpp> /// Include the message_transport header after (!) the ICEY header

int main(int argc, char **argv) {

    icey::icey_debug_print = true;
    /// The second argument is the transport, i.e. the compression algorithm to use. Common ones are "raw", 
    /// Compressed, ffmpeg etc.
    auto camera_center_sub = icey::create_camera_subscription("camera_center", "raw");
    camera_center_sub->then([](sensor_msgs::msgs::Image::SharedPtr image, 
                                    sensor_msgs::msgs::CameraInfo::SharedPtr camera_info){
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Received image and info: ");

    });

    icey::spawn(argc, argv, "camera_subscriber_example"); /// Create and start node
}