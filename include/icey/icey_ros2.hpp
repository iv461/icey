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


#include "rclcpp/rclcpp.hpp"

namespace icey {

/// A ROS adapter, abstracting ROS 1 and ROS 2, so that everything works with both 


/// Simplify the arcane QoS complexity in ROS2 : We have only two: reliable and unreliable. The incident with choosing DDS causes everything to belong mostly to the second category anyway.
enum class SimpleQoS {
    RELIABLE,
    UNRELIABLE
};

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

    using _Node = rclcpp::Node; /// Underlying Node type


    /// A service client interface, that does not produce memory leaks unless the 
    /// user does not forget to call random clean-up functions after calling the service. 
    class Client : public rclcpp::Client {
        using Base = rclcpp::Client;
        /// Base class type aliases:
        using Request = Base::Request;
        using Response = Base::Response;
        using SharedRequest = Base::SharedRequest;
        using SharedResponse = Base::SharedResponse;
        /// rclcpp::FutureReturnCode is either be SUCCESS, INTERRUPTED or TIMEOUT.
        /// (reference: https://docs.ros.org/en/jazzy/p/rclcpp/generated/enum_namespacerclcpp_1a7b4ff5f1e516740d7e11ea97fe6f5532.html#_CPPv4N6rclcpp16FutureReturnCodeE)
        using SyncResponse = std::pair<std::optional<SharedResponse>, rclcpp::FutureReturnCode>

        /// A blocking (synchronous) call to the service. It maybe returns the response 
        /// It calls the async_send_request() and adds the boilerplate code from the examples/documentation.
        /// The second argument is the timeout, by default no timeout is given
        /// TODO now figure out where to get the executor lol ... 
        SyncResponse send_request(SharedRequest request, Duration timeout = Duration(-1)) {
            auto future = client->async_send_request(my_request);
            /// OR rclcpp::spin_until_future_complete(node, result_future) for global executor
            auto future_result = executor->spin_until_future_complete(future, timeout);

            SyncResponse result;
            result.second = future_result;
            if (future_result == rclcpp::FutureReturnCode::TIMEOUT) {
                client->remove_pending_request(future);
            } 
            if(future_result == rclcpp::FutureReturnCode::SUCCESS) {
                result.first = future.get();
            }
            return result;
        }
    };

    
    struct TFSubscription {
        TFSubscription(std::shared_ptr<rclcpp::Node> node) {
            /// This code is a bit tricky. It's about asynchronous programming essentially. The official example is rather incomplete ([official](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)) .
            /// I'm following instead this example https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/src/imu_transformer.cpp#L16
            /// See also the follwing discussions: 
            /// https://answers.ros.org/question/372608/?sort=votes
            /// https://github.com/ros-navigation/navigation2/issues/1182 
            /// https://github.com/ros2/geometry2/issues/446
            tf2_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
            auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                    node->get_node_base_interface(),
                    node->get_node_timers_interface());
            tf2_buffer->setCreateTimerInterface(timer_interface);
            tf2_listener = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer);
        }
         
        std::optional<geometry_msgs::msg::TransformStamped>
            get_transform(std::string from_frame, std::string to_frame) {
                geometry_msgs::msg::TransformStamped t;
                // Look up for the transformation between target_frame and turtle2 frames
                // and send velocity commands for turtle2 to reach target_frame
            try {
                t = tf_buffer->lookupTransform(
                toFrameRel, fromFrameRel,
                tf2::TimePointZero);
                return t;
            } catch (const tf2::TransformException & ex) {
                    /*RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());*/
                return {};
            }
        }
        
        std::unique_ptr<tf2_ros::Buffer> tf2_buffer;
        std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    };
    
    /// A node interface, wrapping to some common functions
    class Node : public rclcpp::Node {
    public:
        using Base = rclcpp::Node;
        
        Node(const std::string &name) : Base(name) {}

        template<typename Msg, typename F>
        void add_subscription(std::string topic, F cb) {
            if(my_subscribers_.count(topic) != 0) {
                /// TODO throw topic already exists
            }
            if(my_publishers_.count(topic) != 0) {
                /// TODO throw cannot subscribe on a topic that is being published at the same time
            }
            my_subscribers_[topic] = create_subscription<Msg>(topic, 1, cb);
        }

        template<typename Msg>
        auto add_publication(std::string topic, std::optional<double> max_frequency) {
            if(my_publishers_.count(topic) != 0) {
                /// TODO throw topic already exists
            }
            if(my_subscribers_.count(topic) != 0) {
                /// TODO throw cannot publish on a topic that is being subscribed at the same time
            }
            auto publisher = create_publisher<Msg>(topic, 1);
            my_publishers_[topic] = publisher;
            auto const publish = [this, publisher, topic, max_frequency](const Msg &msg) {
                auto curr_time = this->get_clock()->now();
                if(!max_frequency || !last_published_time_.count(topic) || 
                       (curr_time - last_published_time_.at(topic)).seconds() > (1. / max_frequency.value()) ) {

                    publisher->publish(msg);
                    last_published_time_[topic] = curr_time;
                }
            };
            return publish;
        }

        template<typename F>
        void add_timer(Duration time_interval, F cb) {
            my_timers_.emplace_back(create_wall_timer(time_interval, cb));
        }

        template<typename F>
        void add_service(std::string name, F cb) {
            my_services_.emplace(name, create_service(name, cb));
        }

        template<typename Service>
        auto add_client(std::string name) {
            auto client = create_client(name);
            my_services_clients_.emplace(name, client);
            using Request std::shared_ptr<typename Service::Request>;

            const auto call_service = [this](Request request) {

            };
        }

        void add_tf_subscription() {

        }

        /// TODO add service 
        /// TODO add action
    private:
        std::map<std::string, rclcpp::Time> last_published_time_;
        std::vector<Timer> my_timers_;
        std::map<std::string, std::any> my_subscribers_;
        std::map<std::string, std::any> my_publishers_;
        std::map<std::string, std::any> my_services_;
        std::map<std::string, std::any> my_services_clients_;
    };

    using NodeHandle = Node;
};

using ROSAdapter = ROS2Adapter;

}

#include <icey/icey.hpp> 
