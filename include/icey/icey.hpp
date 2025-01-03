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


/// A read-only observable
template<typename _StateValue>
class Observable {
public:
    using StateValue = _StateValue;
    using Cb = std::function<void(const StateValue&)>;

    static auto create() {
        return std::make_shared< Observable<StateValue> >();
    }

    /// Register to be notified when smth. changes
    void on_change(Cb cb) {
        notify_list_.push_back(cb);
    }

    auto has_value() const {return value_.has_value(); }

//protected:
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

    static auto create(const std::string &name) {
        return std::make_shared<We>();
    }

    auto name() const {return name_;}

    std::function<void(const ROSAdapter::NodeHandle &)> attach_to_node_;
private:
    SubscribedState(const std::string &name): name_(name) {
        attach_to_node_ = [this, name](const auto &node_handle) {
            node_handle->add_subscription<StateValue>(name, [this](const auto &new_value) {
                set_value(new_value);
            });
        }
    }
    std::string name_; 
};

template<typename StateValue>
class PublishedState : public WritableObservable<StateValue> {
public:
    using Base = WritableObservable<StateValue>;
    using We = PublishedState<StateValue>

    static auto create(const std::string &name) {
        return std::make_shared<We>();
    }
    
    auto name() const {return name_;}
    
    std::function<void(const ROSAdapter::NodeHandle &)> attach_to_node_;
private:

    PublishedState(const std::string &name): name_(name) {
        attach_to_node_ = [this, name](const auto &node_handle) {
            auto publish = node_handle->add_publication<StateValue>(name);
            on_change([publish](const auto &new_value) {
                publish(new_value);
            });
        };
    }
    std::string name_; 
};

/// Global state, used to enable a simple, purely functional API
struct GState {
    std::vector<std::any> staged_observables;
    std::shared_ptr<ROSAdapter::Node> node_;
    ~GState() {
        if(!staged_observables.empty() && !node_) {
            std::cout << "WARNING: You created some signals, but no node was created, did you forget to call icey::spawn() ?" << std::endl;
        }
    }
};

template<typename StateValue>
auto create_signal(const std::string &name) {
    auto signal = SubscribedState<StateValue>::create();
    g_state.staged_observables.push_back(signal);
    return signal;
}

template<typename StateValue>
auto create_state(const std::string &name) {
    auto state = PublishedState<StateValue>::create();
    g_state.staged_observables.push_back(state);
    return state;
}

/// Args must be a Observable, i.e. not constants are supported.
template<typename F, typename... Arguments>
auto compute_based_on(F f, Arguments && ... args) {
    using ReturnType = decltype(f());
    auto new_observable = Observable<ReturnType>::create();
     ([&]{ 
        args.on_change([new_observable](const auto &new_value) {
            auto all_argunments_arrived = (args.has_value() && ...);
            if(all_argunments_arrived) {
                auto result = f(args.value_.get_value()...);
                new_observable.set_value(result);
            }
        });
     }(), ...);
    return new_observable;
}

GState g_state;

/// Blocking spawn of a node
void spawn(int argc, char **argv, 
    std::optional<std::string> node_name = std::nullopt) {
    rclcpp::init(argc, argv);

    /// TODO [Feature] spawn anonymous node
    auto concrete_name = node_name.has_value() ? node_name.value() : std::string("jighe385");

    g_state.node_ = rclcpp::Node::make_shared(concrete_name);

    for(const auto &observable : g_state.staged_observables) {
        observable->attach_to_node_(g_state.node_);
    }

    rclcpp::spin(g_state.node_);
    rclcpp::shutdown();
}

/// Non-blocking spawn of nodes. TODO [Feature] implement this using MultiThreadedExecutor
/*auto spawn_async(int argc, char **argv, 
    std::optional<std::string> node_name = std::nullopt) {
    rclcpp::init(argc, argv);
    auto concrete_name = node_name.has_value() ? node_name.value() : std::string("jighe385");

    g_state.node_ = rclcpp::Node::make_shared(concrete_name);
    rclcpp::spin(g_state.node_);
    rclcpp::shutdown();
}*/

}
