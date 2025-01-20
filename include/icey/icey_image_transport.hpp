//// Support for the image_transport subscribers and publishers. 
#pragma once 

#include <functional>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/camera_subscriber.hpp"

namespace icey {

/// See https://docs.ros.org/en/ros2_packages/jazzy/api/image_transport for API docs.
/// and https://github.com/ros-perception/image_common/blob/humble/image_transport
/*struct ImageTransportSupport {
    using QoS = rclcpp::QoS;
    using DefaultQoS = rclcpp::SystemDefaultsQoS;
    explicit ImageTransportSupport(rclcpp::Node::SharedPtr node) : node_(node), 
        image_transport_(node_) {
    }
    
    void add_publication(std::string topic_name, const QoS &qos = DefaultQoS()) {
        publishers_.emplace(topic_name, image_transport_.advertise(topic_name, qos));
    }

    void add_subscriber(std::string &topic_name, const QoS &qos = DefaultQoS()) {
        publishers_.emplace(topic_name, image_transport_.advertise(topic_name, qos));
    }

    std::map<std::string, image_transport::Publisher> publishers_;
    std::map<std::string, image_transport::Subscriber> subscribers_;
    std::map<std::string, image_transport::CameraPublisher> camera_publishers_;
    std::map<std::string, image_transport::CameraSubscriber> camera_subscribers_;
    
    rclcpp::Node::SharedPtr node_;
    image_transport::ImageTransport image_transport_;
};*/


// An observable representing a camera subscriber. 
struct CameraSubscriber : 
    public Observable < 
        std::tuple < sensor_msgs::msgs::Image::SharedPtr, 
                     sensor_msgs::msgs::CameraInfo::SharedPtr > > {

    CameraSubscriber(const std::string &base_topic_name,
                      const std::string &transport,
                      const ROSAdapter::QoS qos = ROS2Adapter::DefaultQoS()) {
        this->attach_priority_ = 4;
        this->name = topic_name;
        
        attach_ = [base_topic_name, transport, qos](ROSAdapter::NodeHandle &node_handle) {
        if (icey_debug_print) 
            std::cout << "[CameraSubscriber] attach_to_node()" << std::endl;
        /// TODO pass QoS with the qos.getInternalNoTWrapPedRMWAPIThing 
            auto this_obs = this->observable_;
        subscriber_ = image_transport::create_camera_subscription(&node_handle, 
                base_topic_name, [](sensor_msgs::msgs::Image::SharedPtr image, 
                                    sensor_msgs::msgs::CameraInfo::SharedPtr camera_info) {
                this_obs->resolve(std::make_tuple(image, camera_info));
            },
            qos.get_rmw_qos_profile());
        };
    }

  void attach_to_node(ROSAdapter::NodeHandle &node_handle) { attach_(node_handle); }
  std::function<void(ROSAdapter::NodeHandle &)> attach_;
  image_transport::CameraSubscriber subscriber_; /// The image_transport sub/pub use PIMPL, so we can hold them by value.
};

struct CameraPublisher: 
    public Observable < 
        std::tuple < sensor_msgs::msgs::Image::SharedPtr, 
                     sensor_msgs::msgs::CameraInfo::SharedPtr > > {

    CameraPublisher(const std::string &base_topic_name,
                      const ROSAdapter::QoS qos = ROS2Adapter::DefaultQoS()) {
        this->attach_priority_ = 1;
        this->name = topic_name;
        
        attach_ = [base_topic_name, qos](ROSAdapter::NodeHandle &node_handle) {
            if (icey_debug_print) 
                std::cout << "[CameraSubscriber] attach_to_node()" << std::endl;
            
            image_transport::CameraPublisher publisher = image_transport::create_camera_publisher(base_topic_name,
                qos.get_rmw_qos_profile()); /
            auto this_obs = this->observable_;
            this_obs->_register_handler([this_obs, publisher]() { 
                const auto &new_value = this_obs->value(); /// There can be no error
                const auto &[image_msg, camera_info_msg] = new_value;
                publisher->publish(image_msg, camera_info_msg);
            });
        };
   }
  void attach_to_node(ROSAdapter::NodeHandle &node_handle) { attach_(node_handle); }
  std::function<void(ROSAdapter::NodeHandle &)> attach_;
};

auto create_camera_subscription(
    const std::string& base_topic_name, const std::string &transport, const ROS2Adapter::QoS& qos = ROS2Adapter::DefaultQoS()) {
  return g_state.get_context().create_camera_subscription(topic_name, transport, qos);
};


}