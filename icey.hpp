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

namespace icey {

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

        std::map<std::string, Timer> my_timers_;
        std::map<std::string, std::any> my_subscribers_;
        std::map<std::string, std::any> my_publishers_;
    };
};

/// Global state
class GState {
    std::shared_ptr<rclcpp::Node> node_;
};

/// A read-only observable
template<typename StateValue>
class Observable {
public:
    using Cb = std::function<void(const StateValue&)>;

    /// Register to be notified when smth. changes
    void on_change(Cb cb) {
        notify_list_.push_back(cb);
    }

protected:
    void _set_value(const StateValue &new_value) {
        value_ = new_value;
        notify();
    }

    /// Notify all subcribers about the new value
    void notify() {
        for(auto cb: notify_list_) {
            cb(value_.value());
        }
    }

    std::optional<StateValue> value_;
    std::vector<Cb> notify_list_;
};

/// A readable and writable observable
template<typename StateValue>
class WritableObservable : public Observable<StateValue> {
public: 
    void set_value(const StateValue &new_value) {
        _set_value(new_value);
    }
};

enum class FrequencyStrategy {
    KEEP_LAST,
    INTERPOLATE
};

template<typename StateValue>
class SubstribedState : public Observable<StateValue> {
public:
    using Base = Observable<StateValue>;

    SubscribedState(const std::string &name): name_(name) {
        /// TODO move out of the constructor, this should be a factory for consistency with create_timer 
        /// and because this can fail and we do not want a failing constructor
        g_state.node_.add_subscription<StateValue>(name, [this](const auto &new_value) {
            set_value(new_value);
        });
    }

    auto name() const {return name_;}
private:
    std::string name_; 
};

template<typename StateValue>
class PublishedState : public WritableObservable<StateValue> {
public:
    using Base = WritableObservable<StateValue>;

    PublishedState(const std::string &name): name_(name) {
        g_state.node_.add_publication<StateValue>(name, [this](const auto &new_value) {
            set_value(new_value);
        });
    }

    auto name() const {return name_;}
private:
    std::string name_; 
};

template<typename F, typename... Arguments>
auto compute_based_on(F f, Arguments...) {

}

GState g_state;

/// Blocking spawn of a node
void spawn(int argc, char **argv, 
    std::optional<std::string> node_name = std::nullopt) {
    rclcpp::init(argc, argv);
    auto concrete_name = node_name.has_value() ? node_name.value() : std::string("jighe385");

    g_state.node_ = rclcpp::Node::make_shared(concrete_name);
    rclcpp::spin(g_state.node_);
  rclcpp::shutdown();
}

/// Non-blocking spawn of nodes.
auto spawn_async(int argc, char **argv, 
    std::optional<std::string> node_name = std::nullopt) {
    rclcpp::init(argc, argv);
    auto concrete_name = node_name.has_value() ? node_name.value() : std::string("jighe385");

    g_state.node_ = rclcpp::Node::make_shared(concrete_name);
    rclcpp::spin(g_state.node_);
    rclcpp::shutdown();
}
}

int main() {
    
}