#pragma once 

#include <iostream> 
#include <fmt/core.h>
#include <fmt/ostream.h>

#include <functional>
#include <tuple>
#include <map>
#include <unordered_map>

/// A ROS adapter, abstracting ROS 1 and ROS 2, so that everything works with both 
class ROSAdapter {
template<typename MsgT>
  using MsgPtr = typename detail::MsgPtr<MsgT>;
};

#include "rclcpp/rclcpp.hpp"

/// Simplify the arcane QoS complexity in ROS2 : We have only two: reliable and unreliable. The incident with choosing DDS causes everything to belong mostly to the second category anyway.
enum class SimpleQoS {
    RELIABLE,
    UNRELIABLE
};



class ROS2Adapter {
    using Timer = rclcpp::TimerBase::SharedPtr;
    template<typename Msg>
    using Subscriber = rclcpp::Publisher<Msg>::SharedPtr;

    /// TODO use some type-erasing library 
    /*class SubscriberBase {  

    };
    class TypeErasingSub : public SubscriberBase {

    };*/

    class Node : public rclcpp::Node {
        /// TODO should be compat, but check
        using Clock = std::chrono::system_clock;
        using Time = std::chrono::time_point<Clock>;
        using Duration = Clock::duration;
        
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
        void add_publication(std::string topic, F cb) {
            if(my_publishers_.count(topic) != 0) {
                /// TODO throw topic already exists
            }
            if(my_subscribers_.count(topic) != 0) {
                /// TODO throw cannot publish on a topic that is being subscribed at the same time
            }
            my_publishers_[topic] = create_publisher<Msg>(topic, 1);
        }

        void publish() {
            
        }

        template<typename F>
        void add_timer(std::string name, Duration time_interval, F cb) {
            my_timers_[name] = create_wall_timer(time_interval, timer_callback);
        }

        std::map<std::string, Timer> my_timers_;
        std::map<std::string, std::any> my_subscribers_;
        std::map<std::string, std::any> my_publishers_;
    };
};

/// Global state
class GState {
    std::shared_ptr<rclcpp::Node> node_;
};

GState g_state;

void spawn(int argc, char **argv, 
    std::optional<std::string> node_name = std::nullopt) {

    rclcpp::init(argc, argv);
    auto concrete_name = node_name.has_value() ? node_name.value() : std::string("jighe385");
    
    g_state.node_ = rclcpp::Node::make_shared(concrete_name);
    rclcpp::spin(g_state.node_);
  rclcpp::shutdown();
}

class Observable {
    using Cb = std::function<void()>;

    void on_change() {
        for(auto cb: notify_list_) {
            cb();
        }
    }

    std::vector<Cb> notify_list_;
};

enum class FrequencyStrategy {
    KEEP_LAST,
    INTERPOLATE
};

template<typename StateValue>
class SubstribedState : public Observable {
public:
    SubstribedState(const std::string &name, const StateValue &value): name_(name), value_(value) {
        
    }

    std::string name_; 
    StateValue value_;
};

template<typename F, typename... Arguments>
auto compute_based_on(F f, Arguments...) {

}

int main() {

}