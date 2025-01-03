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
    using Subscriber = rclcpp::Subscription<Msg>::SharedPtr;
    using Publisher = rclcpp::Publisher<Msg>::SharedPtr;

    using _Node = rclcpp::Node; /// Underlying Node type
    /// A node interface, wrapping to some common

    class Node : public rclcpp::Node {
    public:
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

        template<typename Msg, typename F>
        auto add_publication(std::string topic, F cb) {
            if(my_publishers_.count(topic) != 0) {
                /// TODO throw topic already exists
            }
            if(my_subscribers_.count(topic) != 0) {
                /// TODO throw cannot publish on a topic that is being subscribed at the same time
            }
            auto publisher = create_publisher<Msg>(topic, 1);
            my_publishers_[topic] = publisher;
            auto const publish = [publisher](const Msg &msg) {
                publisher.publish(msg);
            };
            return publish;
        }

        template<typename F>
        void add_timer(std::string name, Duration time_interval, F cb) {
            my_timers_[name] = create_wall_timer(time_interval, timer_callback);
        }

        /// TODO add service 
        /// TODO add action
    private:
        std::map<std::string, Timer> my_timers_;
        std::map<std::string, std::any> my_subscribers_;
        std::map<std::string, std::any> my_publishers_;
    };
};

using ROSAdapter = ROS2Adapter;

}

#include <icey.hpp> 
