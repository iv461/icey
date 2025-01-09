#pragma once 

#include <iostream> 
#include <fmt/core.h>
#include <fmt/ostream.h>

#include <functional>
#include <tuple>
#include <map>
#include <optional>
#include <unordered_map>
#include <any> 

/// TF2 support 
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"

#include "rclcpp/rclcpp.hpp"

namespace icey {

/// A ROS adapter, abstracting ROS 1 and ROS 2, so that everything works with both 

class ROS2Adapter {
public:

    template<typename Msg>
    using MsgPtr = std::shared_ptr<Msg>;

    /// TODO should be compatible with the templates, but check
    using Clock = std::chrono::system_clock;
    using Time = std::chrono::time_point<Clock>;
    using Duration = Clock::duration;

    using Timer = rclcpp::TimerBase::SharedPtr;

    template<typename Msg>
    using Subscriber = typename rclcpp::Subscription<Msg>::SharedPtr;
    template<typename Msg>
    using Publisher = typename rclcpp::Publisher<Msg>::SharedPtr;

    using NodePtr = std::shared_ptr<rclcpp::Node>;
    using _Node = rclcpp::Node; /// Underlying Node type

    using QoS = rclcpp::QoS;
    using DefaultQos = rclcpp::SystemDefaultsQoS;
    using ServiceQoS = rclcpp::ServicesQoS;

    /// A modified listener that can notify when a single transform changes. 
    /// It is implemented similarly to the tf2_ros::TransformListener, but without a separate spinning thread.
    /// TODO this simple implementation currently checks every time a new message is receved on /tf for every trnasform we are looking for. 
    /// If /tf is high-frequency topic, this can become a performance problem. i.e., the more transforms, the more this becomes a search for the needle in the heapstack.
    /// But without knowing the path in the TF-tree that connects our transforms that we are looking for, we cannot do bettter. And atm
    /// I cannot easily hack the tf2 library since it is a mature and reliably working code mess.
    /// TODO look also if tf2_ros::MessageFilter is not an overall better solution. It adheres to the message_filter interface.
    struct TFListener {
        using TransformMsg = geometry_msgs::msg::TransformStamped;
        
        TFListener(NodePtr node, tf2::BufferCore & buffer) : node_(node), buffer_(buffer) {
            init(node);
        }
        
        /// Add notification for a single transform.
        template<typename CallbackT>
        void add_subscription(std::string target_frame, std::string source_frame, CallbackT && callback) {
            subscribed_transforms_.emplace_back(std::make_pair(target_frame, source_frame), 
                std::nullopt, std::move(callback));
        }

    private:
        using TransformsMsg = tf2_msgs::msg::TFMessage::ConstSharedPtr;
        using NotifyCB = std::function<void(const TransformMsg &)>;

        void on_tf_message(TransformsMsg msg, bool is_static) {
            store_in_buffer(*msg, is_static);
            notify_if_any_relevant_transform_was_received();
        }
        
        void notify_if_any_relevant_transform_was_received() {
            for(const auto &[transform_id, last_received_transform, notify_cb] : subscribed_transforms_) {
                auto maybe_new_transform = get_maybe_new_transform(transform_id.first, transform_id.second, last_received_transform);
                if(maybe_new_transform) {
                    notify_cb(*maybe_new_transform);
                }
            }
        }

        /// This simply looks up the transform in the buffer at the latest stamp and checks if it changed with respect to the previously 
        /// received one. If the transform has changed, we know we have to notify
        std::optional<geometry_msgs::msg::TransformStamped> 
            get_maybe_new_transform(std::string target_frame, std::string source_frame, 
            std::optional<geometry_msgs::msg::TransformStamped> last_received_tf) {
                std::optional<geometry_msgs::msg::TransformStamped> tf_msg;
                try {
                    tf_msg = buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                } catch (tf2::TransformException & e) {
                    /// Simply ignore. Because we are requesting the latest transform in the buffer, the only exception we can get is that there is no transform available yet.
                    /// TODO duble-check if there is reallly nothing to do here.
                    // std::cout << "Lookup failed: " << e.what() << std::endl;
                }
                /// Now check if it is the same as the last one, in this case we return nothing since the transform did not change. (Instead, we received on /tf some other, unrelated transforms.)
                if(last_received_tf && tf_msg && *tf_msg == *last_received_tf)
                    return {};
                return tf_msg;
        }

        /// Store the received transforms in the buffer. 
        void store_in_buffer(const tf2_msgs::msg::TFMessage &msg_in, bool is_static) {
            std::string authority = "Authority undetectable";
            for (size_t i = 0u; i < msg_in.transforms.size(); i++) {
                try {
                    buffer_.setTransform(msg_in.transforms[i], authority, is_static);
                } catch (const tf2::TransformException & ex) {
                    // /\todo Use error reporting
                    std::string temp = ex.what();
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "Failure to set received transform from %s to %s with error: %s\n",
                        msg_in.transforms[i].child_frame_id.c_str(),
                        msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
                }
            }
        }

        void init(NodePtr node) {
            const rclcpp::QoS qos = tf2_ros::DynamicListenerQoS();
            const rclcpp::QoS &static_qos = tf2_ros::StaticListenerQoS();
            message_subscription_tf_ = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", qos, [this](TransformsMsg msg) { on_tf_message(msg, false); });
            message_subscription_tf_static_ = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", static_qos, [this](TransformsMsg msg) { on_tf_message(msg, true); });
        }

        using TFId = std::pair<std::string, std::string>;
        /// A tf subctiption, the frames, last received transform, and the notify CB
        using SubTF = std::tuple < TFId, std::optional<TransformMsg>, NotifyCB >;
        
        NodePtr node_; /// Stored for logging
        tf2::BufferCore & buffer_;

        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_;

        std::vector< SubTF > subscribed_transforms_;
    };
    
    /// A node interface, wrapping to some common functions
    class Node : public rclcpp::Node {
    public:
        using Base = rclcpp::Node;
        
        Node(const std::string &name) : Base(name) {}

        template<typename Msg, typename F>
        void add_subscription(const std::string &topic, F && cb, const rclcpp::QoS & qos = DefaultQos()) {
            if(my_subscribers_.count(topic) != 0) {
                /// TODO throw topic already exists
            }
            if(my_publishers_.count(topic) != 0) {
                /// TODO throw cannot subscribe on a topic that is being published at the same time
            }
            my_subscribers_[topic] = create_subscription<Msg>(topic, qos, cb);
        }

        template<typename Msg>
        auto add_publication(const std::string &topic, const QoS &qos= DefaultQos(), 
            std::optional<double> max_frequency = std::nullopt) {
            if(my_publishers_.count(topic) != 0) {
                /// TODO throw topic already exists
            }
            if(my_subscribers_.count(topic) != 0) {
                /// TODO throw cannot publish on a topic that is being subscribed at the same time
            }
            auto publisher = create_publisher<Msg>(topic, qos);
            my_publishers_[topic] = publisher;
            auto const publish = [this, publisher, topic, max_frequency](const Msg &msg) {
                publisher->publish(msg);   
            };
            return publish;
        }

        template<typename CallbackT>
        auto add_timer(const Duration &time_interval, bool use_wall_time, CallbackT &&cb) {
            rclcpp::TimerBase::SharedPtr timer = 
                use_wall_time ? create_wall_timer(time_interval, std::move(cb)) : 
                                create_wall_timer(time_interval, std::move(cb)); /// TODO no normal timer in humble
            
            my_timers_.push_back(timer);        
            return timer;
        }

        template<typename ServiceT, typename CallbackT>
        void add_service(const std::string &service_name, CallbackT && callback, const rclcpp::QoS & qos = rclcpp::ServicesQoS()) {
            auto service = this->create_service<ServiceT>(service_name, std::move(callback)); /// TODO In humble, we cannot pass QoS, only in Jazzy (the API wasn't ready yet, it needs rmw_*-stuff)
            my_services_.emplace(service_name, service);
        }


        /// TODO cb groups !!
        template<typename Service>
        auto add_client(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
            auto client = this->create_client<Service>(service_name, qos);
            my_services_clients_.emplace(service_name, client);
            return client;
        }

        
        /// Subscribe to a transform on tf between two frames
        template<typename CallbackT>
        void add_tf_subscription(std::string target_frame, std::string source_frame, CallbackT && callback) {
            add_tf_listener_if_needed();
            tf2_listener_->add_subscription(target_frame, source_frame, std::move(callback));
        }

        
        std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;

    private:
        
        void add_tf_listener_if_needed() { 
            if(tf2_listener_) /// We need only one subscription on /tf, but can have multiple transforms on which we listen
                return;
            init_tf_buffer();
            tf2_listener_ = std::make_unique<TFListener>(shared_from_this(), *tf2_buffer_);
        }

        void init_tf_buffer() {
            /// This code is a bit tricky. It's about asynchronous programming essentially. The official example is rather incomplete ([official](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)) .
            /// I'm following instead this example https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/src/imu_transformer.cpp#L16
            /// See also the follwing discussions: 
            /// https://answers.ros.org/question/372608/?sort=votes
            /// https://github.com/ros-navigation/navigation2/issues/1182 
            /// https://github.com/ros2/geometry2/issues/446
            tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                    this->get_node_base_interface(),
                    this->get_node_timers_interface());
            tf2_buffer_->setCreateTimerInterface(timer_interface);
        }

        /// Do not force the user to do the bookkeeping itself: Do it instead automatically 
        
        std::vector<rclcpp::TimerBase::SharedPtr> my_timers_;
        std::map<std::string, std::any> my_subscribers_;
        std::map<std::string, std::any> my_publishers_;
        std::map<std::string, std::any> my_services_;
        std::map<std::string, std::any> my_services_clients_;

        std::vector<rclcpp::CallbackGroup::SharedPtr> my_callback_groups_;

        /// TF stuff
        std::unique_ptr<TFListener> tf2_listener_;
    };

    using NodeHandle = Node;
};

using ROSAdapter = ROS2Adapter;

}

#include <icey/icey.hpp> 
