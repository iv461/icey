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
struct is_pair : std::false_type {};

template<typename... Args>
struct is_pair<std::pair<Args...>> : std::true_type {};

template<typename T>
constexpr bool is_tuple_v = is_tuple<T>::value;

template<typename T>
constexpr bool is_pair_v = is_pair<T>::value;

template<typename Func, typename Tuple>
auto call_if_tuple(Func&& func, Tuple&& tuple) {
    if constexpr (is_tuple_v<std::decay_t<Tuple>> || is_pair_v<std::decay_t<Tuple>>) {
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
    virtual size_t attach_priority() const { return attach_priority_; } 

    size_t attach_priority_{0};
};

/// TODO do we need this ? Only used for graph. A read-only observable, with no value. Atatchnbel to ROS-node 
/// TODO everything that captures this in a lambda should be noncopyable. Currently there are only the subscribers. 
/// But how do we achive transparently copying around only references ?
struct ObservableBase : public NodeAttachable, private boost::noncopyable {
    size_t index{0}; /// The index of this observable in list of vertices in the data-flow graph. We have to store it here because of cyclic dependency
};

/// An observable holding a value. Similar to a promise in JS.
template<typename _StateValue, typename _ErrorValue = std::string>
class Observable : public ObservableBase {
public:
    using StateValue = _StateValue;
    using ErrorValue = _ErrorValue;
    using Self = Observable<_StateValue, _ErrorValue>;

    using OnResolve = std::function<void(const StateValue&)>;
    using OnReject = std::function<void(const ErrorValue&)>;

    /// Register to be notified when smth. changes
    void on_change(OnResolve && cb) {
        notify_list_.emplace_back(std::move(cb)); /// TODO rename to children ?
    }

    /// Notify all subcribers about the new value
    void notify() {
        std::cout << "[OBservable] notifying .." << std::endl;
        for(auto cb: notify_list_) {
            cb(value_.value());
        }
    }


    auto has_value() const {return value_.has_value(); }

//protected: /// TODO make inaccessible to force correct usage of the API !
    void _set(const StateValue &new_value) {
        std::cout << "[OBservable] set was called " << std::endl;
        value_ = new_value;
        notify();
    }

    void _reject(const ErrorValue &error) {
        std::cout << "[OBservable] reject " << std::endl;
        for(auto cb: reject_cbs_) cb(error);
    }

    std::vector<OnResolve> notify_list_;
    std::vector<OnReject> reject_cbs_;
    std::optional<StateValue> value_;
};

template<typename Value>
class ParameterObs : public Observable<Value> {
public:
    using MaybeValue = std::optional<Value>;
    ParameterObs(const std::string &parameter_name, const MaybeValue &default_value
        ) : parameter_name_(parameter_name), default_value_(default_value) {
        //attach_priority_ = 0;
    }

    void attach_to_node(ROSAdapter::NodeHandle & node_handle) override {
        /*
        node_handle.declare_parameter<StateValue>(parameter_name_, [this](std::shared_ptr<StateValue> new_value) {
            this->_set(*new_value);
        }, qos_);
        */
    }

    std::string parameter_name_;
    MaybeValue default_value_;
};

template<typename StateValue>
class SubscribedState : public Observable<StateValue> {
public:
    SubscribedState(const std::string &name, const ROSAdapter::QoS &qos): name_(name), qos_(qos) {
        //attach_priority_ = 3;
    }

    void attach_to_node(ROSAdapter::NodeHandle & node_handle) override {
        node_handle.add_subscription<StateValue>(name_, [this](std::shared_ptr<StateValue> new_value) {
            this->_set(*new_value);
        }, qos_);
    }

    std::string name_;
    ROSAdapter::QoS qos_;
};


/// A signal for subscribing to /tf and obtaining a transform.
struct TransformSignal : public Observable<geometry_msgs::msg::TransformStamped> {
public:
    TransformSignal(const std::string &target_frame, const std::string &source_frame) : 
         target_frame_(target_frame), source_frame_(source_frame) { 
            //attach_priority_ = 3;
            }

    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        node_handle.add_tf_subscription(target_frame_, source_frame_, [this](const geometry_msgs::msg::TransformStamped &new_value) {
            this->_set(new_value);
        });
    }
    
    std::string target_frame_;
    std::string source_frame_;
};

/// Timer signal, saves the number of ticks as the value
struct TimerSignal: public Observable<size_t> {
    TimerSignal(const ROSAdapter::Duration &interval, bool use_wall_time) : 
        interval_(interval), use_wall_time_(use_wall_time) { 
            //attach_priority_ = 5;
            }
    
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {        
        node_handle.add_timer(interval_, use_wall_time_, [this] () {
            this->_set(value_ ? (*value_ + 1) : 0);
        });
    }

    ROSAdapter::Duration interval_;
    bool use_wall_time_{false};
};

/// A publishabe state, read-only
template<typename StateValue>
class PublishableState : public Observable<StateValue> {
public:
    static_assert(rclcpp::is_ros_compatible_type<StateValue>::value, "A publisher must use a publishable ROS message (no primitive types are possible)");

    PublishableState(std::shared_ptr<Observable<StateValue>> parent, const std::string &name, const ROSAdapter::QoS qos=ROS2Adapter::DefaultQos()) : 
        parent_(parent), name_(name), qos_(qos) { 
            //attach_priority_ = 1; 
        }


    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        auto publish = node_handle.add_publication<StateValue>(name_, qos_);
        /// TODO do we need to copy the value to self here ?
        parent_->on_change([publish](const auto &new_value) {
            publish(new_value);
        });
    }

    std::shared_ptr<Observable<StateValue>> parent_;
    std::string name_;
    ROSAdapter::QoS qos_{ROS2Adapter::DefaultQos()};
};

/// A service observable, storing 
template<typename _ServiceT>
struct ServiceObs : public Observable<std::pair<std::shared_ptr<typename _ServiceT::Request>, 
    std::shared_ptr<typename _ServiceT::Response>>> {
    using Request = std::shared_ptr<typename _ServiceT::Request>;
    using Response = std::shared_ptr<typename _ServiceT::Response>;

    ServiceObs(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) : 
        service_name_(service_name), qos_(qos) { 
            //attach_priority_ = 2; 
            }

    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
         node_handle.add_service<_ServiceT>(service_name_, [this](Request request, Response response) {
            this->_set(std::make_pair(request, response));
         }, qos_);
    }
    std::string service_name_;
    ROSAdapter::QoS qos_;
};

template<typename _ServiceT>
struct ClientObs : public Observable<typename _ServiceT::Response> {
    using Request = typename _ServiceT::Request;
    using Response = typename _ServiceT::Request;
    using Parent = std::shared_ptr<Observable<  std::shared_ptr<Request> >>;

    ClientObs(Parent parent, const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) : 
        parent_(parent), service_name_(service_name), qos_(qos) { 
            //attach_priority_ = 4; 
            }

    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        auto client = node_handle.add_client<_ServiceT>(service_name_, qos_);
        parent_->on_change([this, client](std::shared_ptr<Request> request) {
            client->async_send_request(request, [this](auto response_futur) {
                this->_set(response_futur.get());
            });
        });
    }

    Parent parent_;
    std::string service_name_;
    ROSAdapter::QoS qos_;
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

    bool icey_was_initialized_{false}; /// Indicates whether icey_initialize() was called. Used to ensure the graph is static, i.e. no items are added after initially initilizing.

    void assert_icey_was_not_initialized() {
        if(icey_was_initialized_) 
            throw std::invalid_argument("You are not allowed to add signals after ICEY was initialized. The graph must be static");
    }

    template<typename ParameterT>
    auto declare_parameter(const std::string &name, const ParameterT &default_value, 
        const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor(), 
            bool ignore_override = false) {
        assert_icey_was_not_initialized();
        /// TODO 
        auto param_obs = std::make_shared<ParameterObs<ParameterT>>(name, default_value);
        icey_dfg_graph_.add_vertex(param_obs);
        return param_obs;
    }

    template<typename StateValue>
    auto create_subscription(const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
        assert_icey_was_not_initialized();
        auto signal = std::make_shared<SubscribedState<StateValue>>(name, qos);
        /// Attach to graph and return vertex
        icey_dfg_graph_.add_vertex(signal);
        return signal;
    }

     auto create_transform_subscription(const std::string &target_frame, const std::string &source_frame) {
        assert_icey_was_not_initialized();
        auto tf_signal = std::make_shared<TransformSignal>(target_frame, source_frame);
        /// Attach to graph and return vertex
        icey_dfg_graph_.add_vertex(tf_signal);
        return tf_signal;
    }

    /// A writable signal, i.e. publisher
    template<typename StateValue>
    auto create_publisher(std::shared_ptr<Observable<StateValue>> parent, const std::string &topic_name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
        assert_icey_was_not_initialized();
        auto state = std::make_shared<PublishableState<StateValue>>(parent, topic_name, qos);
        icey_dfg_graph_.add_vertex(state);
        return state;
    }

    auto create_timer(const ROSAdapter::Duration &interval, bool use_wall_time = false) {
        assert_icey_was_not_initialized();
        auto observable = std::make_shared<TimerSignal>(interval, use_wall_time);
        icey_dfg_graph_.add_vertex(observable);
        return observable;
    }

    /// Provide a service 
    template<typename ServiceT> 
    auto create_service(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
        assert_icey_was_not_initialized();
        const auto service_attachable = std::make_shared<ServiceObs<ServiceT>>(service_name, qos);
        icey_dfg_graph_.add_vertex(service_attachable);
        return service_attachable;
    }

    /// Add a service client
    template<typename Service> 
    auto create_client(typename ClientObs<Service>::Parent parent, 
         const std::string & service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
        assert_icey_was_not_initialized();
        const auto client_attachable = std::make_shared<ClientObs<Service>>(parent, service_name, qos);
        icey_dfg_graph_.add_vertex(client_attachable);
        return client_attachable;
    }

    /// Here are the filters. First, the most basic filter: fuse. It fuses the inputs and updates the output if any of the inputs change.
    /// Parents must be of type Observable
    template<typename... Parents>
    auto fuse(Parents && ... parents) { 
        /// Remote shared_ptr TODO write proper type trait for this
        using ReturnType = std::tuple<typename std::remove_reference_t<decltype(*parents)>::StateValue...>;
        auto resulting_observable = std::make_shared<PublishableState<ReturnType>>;  /// A result is rhs, i.e. read-only
        icey_dfg_graph_.add_vertex_with_parents(resulting_observable, {parents->index...}); /// And add to the graph
        ([&]{ 
            const auto f_continued = [resulting_observable](auto&&... parents) {
                auto all_argunments_arrived = (parents->value_ && ... && true);
                if(all_argunments_arrived) 
                    resulting_observable->_set(std::make_tuple(parents->value_.value()...));                
            };
            /// This should be called "parent", "parent->on_change()". This is essentially a for-loop over all parents, but C++ cannot figure out how to create a readable syntax, instead we got these fold expressions
            parents->on_change(std::bind(f_continued, std::forward<Parents>(parents)...));
        }(), ...);
        return resulting_observable; /// Return the underlying pointer. We can do this, since internally everything is stores reference-counted.
    }

    template<typename Parent, typename F>
    auto then(Parent &parent, F && f) {
        using ReturnType = decltype(call_if_tuple(f, parent->value_.value()));
        if constexpr (std::is_void_v<ReturnType>) {
            parent->on_change([f=std::move(f)](const auto &new_value) {
                call_if_tuple(f, new_value);
            });
        } else {
            auto resulting_observable = std::make_shared<Observable<ReturnType>>();
            icey_dfg_graph_.add_vertex_with_parents(resulting_observable, {parent->index}); /// And add to the graph

            parent->on_change([resulting_observable, f=std::move(f)](const auto &new_value) {
                resulting_observable->_set(call_if_tuple(f, new_value));
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
    
    Context &icey() { return *this; } /// Upcast to disambiguate calls 

    /// TODO 
    void analyze_graph() {
        /// do_topological_sort_on_dfg_graph()
        // assert_dfg_graph_is_acyclic() 
    }    
    /// This attaches all the ICEY signals to the node, meaning it creates the subcribes etc. It initializes everything in a pre-defined order.
    void icey_initialize() {
        if(icey_dfg_graph_.vertices.empty()) {
            std::cout << "WARNING: Nothing to spawn, try first to create some signals/states" << std::endl;
            return;
        }
        /// First, sort the attachables by priority: first parameters, then publishers, services, subsribers etc.
        /// We could do bin-sort here which runs in linear time, but the standard library does not have an implementation for it.
        std::sort(icey_dfg_graph_.vertices.begin(), icey_dfg_graph_.vertices.end(), 
            [](const auto &attachable1, const auto &attachable2) { return attachable1.data->attach_priority() 
                < attachable1.data->attach_priority();});

        /// Now attach everything to the ROS-Node, this creates the parameters, publishers etc.
        for(auto &vertex : icey_dfg_graph_.vertices) {
            vertex.data->attach_to_node(*this); /// Attach
        }
        
        icey_was_initialized_ = true;
    }
};

/// Blocking spawn of an existing node. must call rclcpp::init() before this !
/// Does decide which executor to call and creates the callback groups depending on the depencies in the DFG
void spawn(int argc, char **argv, std::shared_ptr<ROSAdapter::Node> node, bool use_mt=true) {
    if(use_mt) {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    } else {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
}

/// A non-blocking spawn of a node in its own executor. Returns a future that waits in it's destructor until the executor is done.
auto spawn_async(int argc, char **argv, std::shared_ptr<ROSAdapter::Node> node) {
    /// TODO 
    return std::async(std::launch::async, [node]() {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);    
        executor.spin();
    });
    
}

template<typename ... Futures>
void spin_nodes(Futures && ...f) {
    ([&]{ f.wait(); }(), ...);
    rclcpp::shutdown();
}

/// API aliases 
using Node = ROSNodeWithDFG;

}

#include <icey/functional_api.hpp>
