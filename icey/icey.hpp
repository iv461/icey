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

/// Global state
class GState {
    std::shared_ptr<ROSAdapter::Node> node_;
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

    /// TODO [Feature] spawn anonymous node
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
