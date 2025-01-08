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

template<typename T>
struct is_tuple : std::false_type {};

template<typename... Args>
struct is_tuple<std::tuple<Args...>> : std::true_type {};


template<typename T>
constexpr bool is_tuple_v = is_tuple<T>::value;

// Step 2: Function to unpack and call
template<typename Func, typename Tuple>
auto call_if_tuple(Func&& func, Tuple&& tuple) {
    if constexpr (is_tuple_v<std::decay_t<Tuple>>) {
        // Tuple detected, unpack and call the function
        return std::apply(std::forward<Func>(func), std::forward<Tuple>(tuple));
    } else {
        // Not a tuple, just call the function directly
        return func(std::forward<Tuple>(tuple));
    }
}

struct NodeAttachable {
    virtual void attach_to_node(ROSAdapter::NodeHandle &) {}
    /// Priority at which to attach this, needed to implement an order of initialization:
    // 0: parameters
    // 1: publishers
    // 2: services
    // 3: subscribers   
    // 4: clients 
    // 5: timers
    virtual size_t attach_priority() const { return 0; } 
};

/// Just a helper for timers etc. 
/// TODO solve more elegantly
struct NodeAttachableFunctor : public NodeAttachable {
    using F = std::function<void(ROSAdapter::NodeHandle &)>;

    NodeAttachableFunctor(F && f, size_t attach_priority): f_(std::move(f)), attach_priority_(attach_priority) {}

    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {   
        f_(node_handle);
    }
    size_t attach_priority() const override { return attach_priority_; } 
    F f_;
    size_t attach_priority_{0};
};

/// TODO do we need this ? Only used for graph. A read-only observable, with no value. Atatchnbel to ROS-node 
/// TODO everything that captures this in a lambda should be noncopyable. Currently there are only the subscribers. 
/// But how do we achive transparently copying around only references ?
struct ObservableBase : public NodeAttachable, private boost::noncopyable {
    size_t index{0}; /// The index of this observable in list of vertices in the data-flow graph. We have to store it here because of cyclic dependency
};

/// An observable holding a value. Similar to a promise in JS.
template<typename _StateValue>
class Observable : public ObservableBase {
public:
    using StateValue = _StateValue;
    using Cb = std::function<void(const StateValue&)>;

    /// Register to be notified when smth. changes
    void on_change(Cb && cb) {
        notify_list_.emplace_back(cb); /// TODO rename to children ?
    }

    /// Notify all subcribers about the new value
    void notify() {
        std::cout << "[OBservable] notifying .." << std::endl;
        for(auto cb: notify_list_) {
            cb(value_.value());
        }
    }

    static auto create() {
        return std::make_shared< Observable<StateValue> >();
    }

    auto has_value() const {return value_.has_value(); }

//protected: /// TODO make inaccessible to force correct usage of the API !
    void _set(const StateValue &new_value) {
        std::cout << "[OBservable] set was called " << std::endl;
        value_ = new_value;
        notify();
    }
    std::vector<Cb> notify_list_;
    std::optional<StateValue> value_;
};

enum class FrequencyStrategy {
    KEEP_LAST,
    INTERPOLATE
};

template<typename StateValue>
class SubscribedState : public Observable<StateValue> {
public:
    using Base = Observable<StateValue>;
    using Self = SubscribedState<StateValue>;

    static auto create(const std::string &name, const ROSAdapter::QoS &qos) {
        return std::make_shared<Self>(name, qos);
    }
    
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) override {
        node_handle.add_subscription<StateValue>(name_, [this](std::shared_ptr<StateValue> new_value) {
            this->_set(*new_value);
        }, qos_);
    }

    size_t attach_priority() const override { return 3; }

    SubscribedState(const std::string &name, const ROSAdapter::QoS &qos): name_(name), qos_(qos) {}

    std::string name_; 
    ROSAdapter::QoS qos_;
};


/// A signal for subscribing to /tf and obtaining a transform.
/// TODO consider deriving from SubscribedObs
struct TransformSignal : public Observable<geometry_msgs::msg::TransformStamped> {
public:
    using Base = Observable<geometry_msgs::msg::TransformStamped>;
    using StateValue = Base::StateValue;
    
    static auto create(const std::string &frame1, const std::string &frame2) {
        return std::make_shared<TransformSignal>(frame1, frame2);
    }

    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        node_handle.add_tf_subscription(frame1_, frame2_, [this](const StateValue &new_value) {
            this->_set(new_value);
        });
    }

    size_t attach_priority() const override { return 3; } /// TODO dup

    /// TODO make ctor private
    TransformSignal(const std::string &frame1, const std::string &frame2) : 
        frame1_(frame1), frame2_(frame2) {}

    std::string frame1_;
    std::string frame2_;
  
};

/// Timer signal, saves the number of ticks as the value
struct TimerSignal: public Observable<size_t> {
    static auto create(const ROSAdapter::Duration &interval, bool use_wall_time) {
        return std::make_shared<TimerSignal>(interval, use_wall_time);
    }
    size_t attach_priority() const override { return 5; } /// TODO dup

    
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {        
        node_handle.add_timer(interval_, use_wall_time_, [this] () {
            size_t new_value = value_ ? (*value_ + 1) : 0;
            this->_set(new_value);
        });
    }

    TimerSignal(const ROSAdapter::Duration &interval, bool use_wall_time) : interval_(interval), use_wall_time_(use_wall_time) {}

    ROSAdapter::Duration interval_;
    bool use_wall_time_{false};
};

/// A publishabe state, read-only
template<typename StateValue>
class PublishableState : public Observable<StateValue> {
public:
    using Base = Observable<StateValue>;
    using Self = PublishableState<StateValue>;

    static auto create() {
        return std::make_shared<Self>();
    }

    void publish(const std::string &name, const ROSAdapter::QoS qos=  ROS2Adapter::DefaultQos(), std::optional<double> max_frequency = std::nullopt) {
        //static_assert(rclcpp::is_ros_compatible_type<StateValue>::value, "The function has to return a publishable ROS message (no primitive types are possible)");
        name_ = name;
        qos_ = qos;
        max_frequency_ = max_frequency;
    }

    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        if(name_ == "") {
            return; /// Do not do anything if we should not publish.
        }

        auto publish = node_handle.add_publication<StateValue>(name_, qos_, max_frequency_);
        this->on_change([publish](const auto &new_value) {
            std::cout << "[PublishableState] value changed, publishing .." << std::endl;
            publish(new_value);
        });
    
    }

    size_t attach_priority() const override { return 1; } 

    std::string name_;
    ROSAdapter::QoS qos_{ROS2Adapter::DefaultQos()};
    std::optional<double> max_frequency_;
};

/// A graph, owning the observables TODO decide on base-class, since the graph only needs to own the data, we could even use std::any
struct Graph {
    using NodeData = std::shared_ptr<ObservableBase>;
    struct Node {
        explicit Node(const NodeData &_data): data(_data) {}
        NodeData data;
        std::vector<size_t> in_edges;
        std::vector<size_t> out_edges;
    };

    /// Create a new node from data
    Node& add_vertex(const NodeData &node_data) {
        node_data->index = vertices.size(); /// TODO very ugly, but Obs needs to know the index
        vertices.emplace_back(node_data);
        return vertices.back();
    }
    Node& add_vertex_with_parents(const NodeData &node_data, const std::vector<size_t> &parents) {
        auto &new_node = add_vertex(node_data);
        new_node.in_edges = parents;
        return new_node;
    }
    std::vector<Node> vertices;
};

/// A context, essentially the data-flow graph. Basis for the class-based API of ICEY.
struct Context {
    /// These are prepended with ICEY because with derive from rclcpp::Node and have to avoid name collisions
    Graph icey_dfg_graph_;
    std::vector<std::shared_ptr<NodeAttachable>> icey_node_attachables_; /// TODO Other node attachables that actually also should be in the graph
    bool icey_was_initialized_{false}; /// Indicates whether icey_initialize() was called. Used to ensure the graph is static, i.e. no items are added after initially initilizing.

    void assert_icey_was_not_initialized() {
        if(icey_was_initialized_) 
            throw std::invalid_argument("You are not allowed to add signals after ICEY was initialized. The graph must be static");
    }

    template<typename StateValue>
    auto create_signal(const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
        assert_icey_was_not_initialized();
        auto signal = SubscribedState<StateValue>::create(name, qos);
        /// Attach to graph and return vertex
        icey_dfg_graph_.add_vertex(signal);
        return signal;
    }

     auto create_transform_signal(const std::string &frame1, const std::string &frame2) {
        assert_icey_was_not_initialized();
        auto tf_signal = TransformSignal::create(frame1, frame2);
        /// Attach to graph and return vertex
        icey_dfg_graph_.add_vertex(tf_signal);
        return tf_signal;
    }

    /// A writable signal, i.e. publisher
    template<typename StateValue>
    auto create_state(const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos(), std::optional<double> max_frequency = std::nullopt) {
        assert_icey_was_not_initialized();
        auto state = PublishableState<StateValue>::create();
        state->publish(name, qos, max_frequency);
        icey_dfg_graph_.add_vertex(state);
        return state;
    }

    auto create_timer(const ROSAdapter::Duration &interval, bool use_wall_time = false) {
        assert_icey_was_not_initialized();
        auto observable = TimerSignal::create(interval, use_wall_time);
        icey_dfg_graph_.add_vertex(observable);
        return observable;
    }

    /// Provide a service 
    template<typename ServiceT, typename CallbackT> 
    void create_service(const std::string &service_name, CallbackT && callback, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
        assert_icey_was_not_initialized();
        const size_t attach_priority = 2;
        const auto service_attachable = std::make_shared<NodeAttachableFunctor>([service_name, callback = std::move(callback), qos](auto &node_handle) 
            { node_handle.template add_service<ServiceT>(service_name, std::move(callback), qos); }, attach_priority);
        icey_node_attachables_.push_back(service_attachable);
    }

    /// Add a service client
    template<typename Service> 
    void create_client(const std::string & service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
        assert_icey_was_not_initialized();
        const size_t attach_priority = 4;
        const auto client_attachable = std::make_shared<NodeAttachableFunctor>([service_name, qos](auto &node_handle) 
            { node_handle.template add_client<Service>(service_name, qos); }, attach_priority);
        icey_node_attachables_.push_back(client_attachable);
    }

    /// Here are the filters. First, the most basic filter: fuse. It fuses the inputs and updates the output if any of the inputs change.
    /// Parents must be of type Observable
    template<typename... Parents>
    auto fuse(Parents && ... parents) { 
        /// Remote shared_ptr TODO write proper type trait for this
        using ReturnType = std::tuple<typename std::remove_reference_t<decltype(*parents)>::StateValue...>;
        auto resulting_observable = PublishableState<ReturnType>::create();  /// A result is rhs, i.e. read-only

        /// Add to global graph
        const std::vector<size_t> node_parents{parents->index...};
        icey_dfg_graph_.add_vertex_with_parents(resulting_observable, node_parents); /// And add to the graph

        ([&]{ 
            const auto f_continued = [resulting_observable](auto&&... parents) {
                auto all_argunments_arrived = (parents->value_ && ... && true);
                if(all_argunments_arrived) {
                    auto result = std::make_tuple(parents->value_.value()...);
                    resulting_observable->_set(result);
                }
            };
            /// This should be called "parent", "parent->on_change()". This is essentially a for-loop over all parents, but C++ cannot figure out how to create a readable syntax, instead we got these fold expressions
            parents->on_change(std::bind(f_continued, std::forward<Parents>(parents)...));
        }(), ...);
        return resulting_observable; /// Return the underlying pointer. We can do this, since internally everything is stores reference-counted.
    }

    template<typename Parent, typename F>
    auto then(Parent &parent, F && f) {
        using ReturnType = decltype(f(parent->value_.value()));
        if constexpr (std::is_void_v<ReturnType>) {
            parent->on_change([f=std::move(f)](const auto &new_value) {
                f(new_value);
            });
        } else {
            auto resulting_observable = PublishableState<ReturnType>::create();
            icey_dfg_graph_.add_vertex_with_parents(resulting_observable, {parent->index}); /// And add to the graph

            parent->on_change([resulting_observable, f=std::move(f)](const auto &new_value) {
                auto result = f(new_value);
                resulting_observable->_set(result);  
            });
            return resulting_observable;
        }
    }
};

/// The ROS node, additionally owning the data-flow graph (DFG) that contains the observables 
class ROSNodeWithDFG : public ROSAdapter::Node, public Context {
public:
    using NodeBase = ROSAdapter::Node;
    using NodeBase::NodeBase;
    using Self = ROSNodeWithDFG;


    /// This attaches all the ICEY signals to the node, meaning it creates the subcribes etc. It initializes everything in a pre-defined order.
    void icey_initialize() {
        if(icey_dfg_graph_.vertices.empty()) {
            std::cout << "WARNING: Nothing to spawn, try first to create some signals/states" << std::endl;
            return;
        }
        /// TODO bin-sort by prio !
        
        /// First, attach to the ROS node all vertices in the DFG 
        for(auto &vertex : icey_dfg_graph_.vertices) {
            vertex.data->attach_to_node(*this); /// Attach
        }
        /// Then, add all other misc 
        for(auto &attachable : icey_node_attachables_) {
            attachable->attach_to_node(*this); /// Attach
        }
        icey_was_initialized_ = true;
    }
};

/// Blocking spawn of an existing node. must call rclcpp::init() before this !
/// Does decide which executor to call and creates the callback groups depending on the depencies in the DFG
void spawn(int argc, char **argv, std::shared_ptr<ROSAdapter::Node> node) {
    rclcpp::spin(node);
    rclcpp::shutdown();
}

/// API aliases 
using Node = ROSNodeWithDFG;

#include <icey/functional_api.hpp>

}
