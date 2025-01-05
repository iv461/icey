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

#include <boost/noncopyable.hpp>
namespace icey {

// A thing with a name.
class Nameable {
public:

};

/// A read-only observable
template<typename _StateValue>
class Observable : private boost::noncopyable {
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

//protected: /// TODO make inaccessible to force correct usage of the API !
    void _set(const StateValue &new_value) {
        std::cout << "[OBservable] set was called " << std::endl;
        value_ = new_value;
        notify();
    }

    /// Notify all subcribers about the new value
    void notify() {
        std::cout << "[OBservable] notifying .." << std::endl;
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
    void set(const StateValue &new_value) {
        this->_set(new_value);
    }
};

enum class FrequencyStrategy {
    KEEP_LAST,
    INTERPOLATE
};


using NodeAttachable = std::function<void(const std::shared_ptr<ROSAdapter::NodeHandle> &node_handle)>;

template<typename StateValue>
class SubscribedState : public Observable<StateValue> {
public:
    using Base = Observable<StateValue>;
    using We = SubscribedState<StateValue>;

    static auto create(const std::string &name) {
        return std::make_shared<We>(name);
    }

    auto name() const {return name_;}

    NodeAttachable make_node_attachable() {
        return [this](const std::shared_ptr<ROSAdapter::NodeHandle> &node_handle) {
            node_handle->add_subscription<StateValue>(name_, [this](const StateValue &new_value) {
                this->_set(new_value);
            });
        };
    }

    SubscribedState(const std::string &name): name_(name) {}
    std::string name_; 
};


template<typename StateValue>
class PublishedState : public WritableObservable<StateValue>, public NodeAttachable {
public:
    using Base = WritableObservable<StateValue>;
    using We = PublishedState<StateValue>;

    static auto create(const std::string &name, std::optional<double> max_frequency) {
        return std::make_shared<We>(name, max_frequency);
    }
    
    auto name() const {return name_;}
    
    NodeAttachable make_node_attachable() {
        return [this](const std::shared_ptr<ROSAdapter::NodeHandle> &node_handle) {
            auto publish = node_handle->add_publication<StateValue>(name_, max_frequency_);
            this->on_change([publish](const auto &new_value) {
                std::cout << "[PublishedState] value changed, publishing .." << std::endl;
                publish(new_value);
            });
        };
    }
/// TODO make private
    PublishedState(const std::string &name, std::optional<double> max_frequency): name_(name), 
        max_frequency_(max_frequency) {}
    std::string name_; 
    std::optional<double> max_frequency_;
};

/// Global node object adapter, used to conveniently wrap getting the underlying node
struct GlobNode {
    std::shared_ptr<ROSAdapter::NodeHandle> node_;
};

struct NodeWithAttachedObservables {
    /// TODO attach here all the observables
};

/// Global state, used to enable a simple, purely functional API
struct GState {
    std::vector<NodeAttachable> staged_node_attachables;
    std::shared_ptr<ROSAdapter::NodeHandle> node_;
    ~GState() {
        if(!staged_node_attachables.empty() && !node_) {
            std::cout << "WARNING: You created some signals/states/timers, but no node was created, did you forget to call icey::spawn() ?" << std::endl;
        }
    }
};

GState g_state;

template<typename StateValue>
auto create_signal(const std::string &name) {
    auto signal = SubscribedState<StateValue>::create(name);
    g_state.staged_node_attachables.emplace_back(signal->make_node_attachable());
    return signal;
}

template<typename StateValue>
auto create_state(const std::string &name, std::optional<double> max_frequency = std::nullopt) {
    auto state = PublishedState<StateValue>::create(name, max_frequency);
    g_state.staged_node_attachables.emplace_back(state->make_node_attachable());
    return state;
}

template<typename F> 
void create_timer(const ROSAdapter::Duration &interval, F cb) {
    const auto timer_attachable = [interval, cb = std::move(cb)](const auto &node_handle) 
        { node_handle->add_timer(interval, cb); };
    g_state.staged_node_attachables.push_back(timer_attachable);
}

/// Args must be an Observable, i.e. no constants are supported.
template<typename F, typename... Arguments>
auto compute_based_on(F f, Arguments && ... args) {
    using ReturnType = decltype(f(args->value_.value()...));
    auto new_observable = Observable<ReturnType>::create(); /// A result is rhs, i.e. read-only
     ([&]{ 
        const auto f_continued = [new_observable, f](auto&&... args) {
            auto all_argunments_arrived = (args && ... && true);
            if(all_argunments_arrived) {
                auto result = f(args->value_.value()...);
                new_observable->_set(result);
            }
        };
        args->on_change(std::bind(f_continued, std::forward<Arguments>(args)...));
     }(), ...);
    return new_observable;
}

/// Blocking spawn of a node
void spawn(int argc, char **argv, 
    std::optional<std::string> node_name = std::nullopt) {

    if(g_state.staged_node_attachables.empty()) {
        std::cout << "WARNING: Nothing to spawn, try first to create some signals/states" << std::endl;
        return;
    }

    rclcpp::init(argc, argv);

    /// TODO [Feature] spawn anonymous node
    auto concrete_name = node_name.has_value() ? node_name.value() : std::string("jighe385");

    g_state.node_ = std::make_shared<ROSAdapter::Node>(concrete_name);

    for(const auto &attachable : g_state.staged_node_attachables) {
        attachable(g_state.node_); /// Attach
    }

    rclcpp::spin(g_state.node_);

    /// TODO
    //staged_node_attachables.clear();
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
