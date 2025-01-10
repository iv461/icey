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
#include <boost/hana.hpp>

namespace icey {

bool icey_debug_print = false;

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


template<class T>
struct is_optional : std::false_type {};

template<class T>
struct is_optional<std::optional<T>> : std::true_type {};

template<class T>
constexpr bool is_optional_v = is_optional<T>::value;


template<class T>
struct remove_optional { using type = T;};

template<class T>
struct remove_optional<std::optional<T>> { using type = T; };

template<typename Head, typename...Tail>
constexpr bool all_same(const std::tuple<Head,Tail...>&){
    return (std::is_same_v<Head,Tail> && ...);
}

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

class NodeAttachable {
public:
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
class ObservableBase : public NodeAttachable, private boost::noncopyable {
public:
    std::optional<size_t> index; /// The index of this observable in list of vertices in the data-flow graph. We have to store it here because of cyclic dependency. None if this observable was not attached to the graph yet.
};

class Context;
/// An observable holding a value. Similar to a promise in JS.
template<typename _StateValue, typename _ErrorValue = std::string>
class Observable : public ObservableBase {
    friend class Context; /// The context needs to set the value, but to guard against misuse, the value must be inaccessible
public:
    using StateValue = _StateValue;
    using ErrorValue = _ErrorValue;
    using Self = Observable<_StateValue, _ErrorValue>;

    using OnResolve = std::function<void(const StateValue&)>;
    using OnReject = std::function<void(const ErrorValue&)>;

    auto has_value() const {return value_.has_value(); }


protected:
    /// Register to be notified when smth. changes
    void on_change(OnResolve && cb) {
        notify_list_.emplace_back(std::move(cb)); /// TODO rename to children ?
    }

    /// Notify all subcribers about the new value
    void notify() {
        for(auto cb: notify_list_) {
            cb(value_.value());
        }
    }


    void _set(const StateValue &new_value) {
        value_ = new_value;
        notify();
    }

    void _reject(const ErrorValue &error) {
        for(auto cb: reject_cbs_) cb(error);
    }

    std::vector<OnResolve> notify_list_;
    std::vector<OnReject> reject_cbs_;
    std::optional<StateValue> value_;
};

template<typename Value>
class ParameterObservable : public Observable<Value> {
public:
    using MaybeValue = std::optional<Value>;
    /// TODO FIX ARG DUP, capture hana sequence maybe in cb maybe
    ParameterObservable(const std::string &parameter_name, const MaybeValue &default_value,
        const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor 
                = rcl_interfaces::msg::ParameterDescriptor(), 
                    bool ignore_override = false
        ) : parameter_name_(parameter_name), default_value_(default_value), 
            parameter_descriptor_(parameter_descriptor), ignore_override_(ignore_override) {
        this->attach_priority_ = 0;
    }

    /// Parameters are initialized always at the beginning, so we can provide getters for the value. Note that has_value() must be checked beforehand since if no default value was provided, this function will throw std::bad_optional_access()
    const Value &get() const {
        /// TODO implement cleaner, improve error message
        /// TODO do we want to allow this if the user provided a default value ? I think not, since it adds inconcistency to the otherwise inaccessible Promises
        if(!this->value_.has_value()) { /// First, check if this observable was accessed before spawning. This is needed to provide a nice  error message since confusing syncronous code will likely happen for users.
            throw std::runtime_error("[parameter '" + parameter_name_ + "']You cannot access the parameters before spawning the node, you can only access them inside callbacks (which are triggered after calling icey::spawn())");
        }
        return this->value_.value();
    }

protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) override {
        if(icey_debug_print)
            std::cout << "[ParameterObservable] attach_to_node()" << std::endl;
        /// Declare on change handler
        node_handle.declare_parameter<Value>(parameter_name_, default_value_, 
            [this](const rclcpp::Parameter &new_param) {
                this->_set(new_param.get_value<Value>());
        }, parameter_descriptor_, ignore_override_);
        /// Set default value
        if(default_value_) {
            Value initial_value;
            node_handle.get_parameter_or(parameter_name_, initial_value, *default_value_);
            this->_set(initial_value);
        }
    }
    std::string parameter_name_;
    MaybeValue default_value_;
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor_;
    bool ignore_override_;
};

template<typename StateValue>
class SubscriptionObservable : public Observable<StateValue> {
public:
    SubscriptionObservable(const std::string &name, const ROSAdapter::QoS &qos): name_(name), qos_(qos) {
        this->attach_priority_ = 3;
    }

protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) override {
        if(icey_debug_print)
            std::cout << "[SubscriptionObservable] attach_to_node()" << std::endl;
        node_handle.add_subscription<StateValue>(name_, [this](std::shared_ptr<StateValue> new_value) {
            this->_set(*new_value);
        }, qos_);
    }

    std::string name_;
    ROSAdapter::QoS qos_;
};


/// A subscription for single transforms 
struct TransformSubscriptionObservable : public Observable<geometry_msgs::msg::TransformStamped> {
public:
    TransformSubscriptionObservable(const std::string &target_frame, const std::string &source_frame) : 
         target_frame_(target_frame), source_frame_(source_frame) { 
            this->attach_priority_ = 3;
            }

protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        if(icey_debug_print)
            std::cout << "[TransformSubscriptionObservable] attach_to_node()" << std::endl;
        node_handle.add_tf_subscription(target_frame_, source_frame_, [this](const geometry_msgs::msg::TransformStamped &new_value) {
            this->_set(new_value);
        });
    }
    
    std::string target_frame_;
    std::string source_frame_;
};

/// Timer signal, saves the number of ticks as the value and also passes the timerobject as well to the callback
struct TimerObservable: public Observable<size_t> {
    TimerObservable(const ROSAdapter::Duration &interval, bool use_wall_time, bool is_one_off_timer) : 
        interval_(interval), use_wall_time_(use_wall_time), is_one_off_timer_(is_one_off_timer) { 
            this->attach_priority_ = 5;
            }
    
protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {      
        if(icey_debug_print)
            std::cout << "[TimerObservable] attach_to_node()" << std::endl;  
        ros_timer_ = node_handle.add_timer(interval_, use_wall_time_, [this] () {
            this->_set(value_ ? (*value_ + 1) : 0);
            if(is_one_off_timer_)
                ros_timer_->cancel();
        });
    }
    rclcpp::TimerBase::SharedPtr ros_timer_;
    ROSAdapter::Duration interval_;
    bool use_wall_time_{false};
    bool is_one_off_timer_{false};
};

/// A publishabe state, read-only
template<typename StateValue>
class PublisherObservable : public Observable<StateValue> {
public:
    static_assert(rclcpp::is_ros_compatible_type<StateValue>::value, "A publisher must use a publishable ROS message (no primitive types are possible)");

    PublisherObservable(const std::string &topic_name, const ROSAdapter::QoS qos=ROS2Adapter::DefaultQos()) : topic_name_(topic_name), qos_(qos) {
        this->attach_priority_ = 1; 
    }

protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        if(icey_debug_print)
            std::cout << "[PublisherObservable] attach_to_node()" << std::endl;
        auto publish = node_handle.add_publication<StateValue>(topic_name_, qos_);
        this->on_change([publish](const auto &new_value) { publish(new_value); });
    }

    std::string topic_name_;
    ROSAdapter::QoS qos_{ROS2Adapter::DefaultQos()};
};

// A transform broadcaster observable 
struct TransformPublisherObservable : public Observable<geometry_msgs::msg::TransformStamped> {
protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        if(icey_debug_print)
            std::cout << "[TransformPublisherObservable] attach_to_node()" << std::endl;
        auto publish = node_handle.add_tf_broadcaster_if_needed();
        this->on_change([publish](const auto &new_value) { publish(new_value); });
    }
};

/// A service observable, storing 
template<typename _ServiceT>
struct ServiceObservable : public Observable<std::pair<std::shared_ptr<typename _ServiceT::Request>, 
    std::shared_ptr<typename _ServiceT::Response>>> {
    using Request = std::shared_ptr<typename _ServiceT::Request>;
    using Response = std::shared_ptr<typename _ServiceT::Response>;

    ServiceObservable(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) : 
        service_name_(service_name), qos_(qos) {  this->attach_priority_ = 2; 
    }

protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        if(icey_debug_print)
            std::cout << "[ServiceObservable] attach_to_node()" << std::endl;
         node_handle.add_service<_ServiceT>(service_name_, [this](Request request, Response response) {
            this->_set(std::make_pair(request, response));
         }, qos_);
    }
    std::string service_name_;
    ROSAdapter::QoS qos_;
};

template<typename _ServiceT>
struct ClientObservable : public Observable<typename _ServiceT::Response> {
    using Request = typename _ServiceT::Request;
    using Response = typename _ServiceT::Request;
    using Parent = std::shared_ptr<Observable<  std::shared_ptr<Request> >>;

    ClientObservable(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) : 
        service_name_(service_name), qos_(qos) { this->attach_priority_ = 4; }
    
protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        if(icey_debug_print)
            std::cout << "[ClientObservable] attach_to_node()" << std::endl;
        auto client = node_handle.add_client<_ServiceT>(service_name_, qos_);
        this->on_change([this, client](std::shared_ptr<Request> request) {
            client->async_send_request(request, [this](auto response_futur) {
                this->_set(response_futur.get());
            });
        });
    }

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
    template<typename ParameterT>
    auto declare_parameter(const std::string &name, const std::optional<ParameterT> &maybe_default_value= std::nullopt, 
        const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor(), 
            bool ignore_override = false) {
        assert_icey_was_not_initialized();
        auto param_obs = std::make_shared<ParameterObservable<ParameterT>>(name, maybe_default_value, parameter_descriptor, ignore_override);
        icey_dfg_graph_.add_vertex(param_obs);
        return param_obs;
    }

    template<typename MessageT>
    auto create_subscription(const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
        assert_icey_was_not_initialized();
        auto signal = std::make_shared<SubscriptionObservable<MessageT>>(name, qos);
        /// Attach to graph and return vertex
        icey_dfg_graph_.add_vertex(signal);
        return signal;
    }

     auto create_transform_subscription(const std::string &target_frame, const std::string &source_frame) {
        assert_icey_was_not_initialized();
        auto tf_signal = std::make_shared<TransformSubscriptionObservable>(target_frame, source_frame);
        /// Attach to graph and return vertex
        icey_dfg_graph_.add_vertex(tf_signal);
        return tf_signal;
    }

    /// A writable signal, i.e. publisher
    template<typename MessageT>
    auto create_publisher(std::shared_ptr<Observable<MessageT>> parent, const std::string &topic_name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
        assert_icey_was_not_initialized();
        auto publisher_observable = std::make_shared<PublisherObservable<MessageT>>(topic_name, qos);
        icey_dfg_graph_.add_vertex_with_parents(publisher_observable, {parent->index.value()});
        icey_connect(parent, publisher_observable);
        return publisher_observable;
    }
    
    auto create_transform_publisher(std::shared_ptr<Observable<geometry_msgs::msg::TransformStamped>> parent) {
        assert_icey_was_not_initialized();
        auto publisher_observable = std::make_shared<TransformPublisherObservable>();
        icey_dfg_graph_.add_vertex_with_parents(publisher_observable, {parent->index.value()});
        icey_connect(parent, publisher_observable); /// This would be done by publish
        return publisher_observable;
    }
    
    auto create_timer(const ROSAdapter::Duration &interval, bool use_wall_time = false, bool is_one_off_timer = false) {
        assert_icey_was_not_initialized();
        auto observable = std::make_shared<TimerObservable>(interval, use_wall_time, is_one_off_timer);
        icey_dfg_graph_.add_vertex(observable);
        return observable;
    }

    /// Provide a service 
    template<typename ServiceT> 
    auto create_service(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
        assert_icey_was_not_initialized();
        const auto service_attachable = std::make_shared<ServiceObservable<ServiceT>>(service_name, qos);
        icey_dfg_graph_.add_vertex(service_attachable);
        return service_attachable;
    }

    /// Add a service client
    template<typename ServiceT> 
    auto create_client(typename ClientObservable<ServiceT>::Parent parent, 
         const std::string & service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
        assert_icey_was_not_initialized();
        const auto client_attachable = std::make_shared<ClientObservable<ServiceT>>(service_name, qos);
        icey_dfg_graph_.add_vertex_with_parents(client_attachable, {parent->index.value()});
        icey_connect(parent, client_attachable); /// Connect parent with child so that when the parent changes, the new value is propagated to the child
        return client_attachable;
    }

    /// Here are the filters. First, the most basic filter: fuse. It fuses the inputs and updates the output if any of the inputs change.
    /// Parents must be of type Observable
    template<typename... Parents>
    auto fuse(Parents && ... parents) { 
        /// Remote shared_ptr TODO write proper type trait for this
        using ReturnType = std::tuple<typename std::remove_reference_t<decltype(*parents)>::StateValue...>;
        auto resulting_observable = std::make_shared< Observable<ReturnType> >();  /// A result is rhs, i.e. read-only
        icey_dfg_graph_.add_vertex_with_parents(resulting_observable, {parents->index.value()...}); /// And add to the graph
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

    /// Serialize, pipe arbitrary number of parents of the same type into one. Needed for the control-flow where the same publisher can be called from multiple callbacks
    template<typename... Parents>
    auto serialize(Parents && ... parents) { 
        /// DODO all types are the same
        /// Remote shared_ptr TODO write proper remove_shared_ptr type trait for this
        using FirstType = decltype(*std::get<0>(std::forward_as_tuple(parents...)));
        using ReturnType = typename std::remove_reference_t<FirstType>::StateValue;
        auto resulting_observable = std::make_shared< Observable<ReturnType> >();  /// A result is rhs, i.e. read-only
        icey_dfg_graph_.add_vertex_with_parents(resulting_observable, {parents->index.value()...}); /// And add to the graph
        ([&]{ 
            const auto cb = [resulting_observable](const auto &new_val) {
                resulting_observable->_set(new_val);
            };
            /// This should be called "parent", "parent->on_change()". This is essentially a for-loop over all parents, but C++ cannot figure out how to create a readable syntax, instead we got these fold expressions
            parents->on_change(cb);
        }(), ...);
        return resulting_observable; /// Return the underlying pointer. We can do this, since internally everything is stores reference-counted.
    }

    /// When reference updates, we pass all others parents, 
    template<class Reference, class... Parents>
    auto sync_with_reference(Reference && reference, Parents && ... parents) { 
        /// Remote shared_ptr TODO write proper type trait for this
        using ReturnType = std::tuple<Reference, typename std::remove_reference_t<decltype(*parents)>::StateValue...>;
        auto resulting_observable = std::make_shared< Observable<ReturnType> >();  /// A result is rhs, i.e. read-only
        icey_dfg_graph_.add_vertex_with_parents(resulting_observable, {reference, parents->index.value()...}); /// And add to the graph
        const auto f_continued = [resulting_observable, reference](auto&&... parents) {
            auto all_argunments_arrived = (parents->value_ && ... && true);
            if(all_argunments_arrived) 
                resulting_observable->_set(std::make_tuple(reference, parents->value_.value()...));
        };
        /// This should be called "parent", "parent->on_change()". This is essentially a for-loop over all parents, but C++ cannot figure out how to create a readable syntax, instead we got these fold expressions
        reference->on_change(std::bind(f_continued, std::forward<Parents>(parents)...));
        return resulting_observable; /// Return the underlying pointer. We can do this, since internally everything is stores reference-counted.
    }


    /// Creates a new Observable that changes it's value to y every time the value x of the parent observable changes, where y = f(x).
    /// TODO maybe use another observable that does not store the value, but instead simply passes it through ? For efficiency of higher-order filters
    template<typename Parent, typename F>
    auto then(Parent &parent, F && f) {
        using ReturnType = decltype(call_if_tuple(f, parent->value_.value()));
        if constexpr (std::is_void_v<ReturnType>) {
            parent->on_change([f=std::move(f)](const auto &new_value) {
                call_if_tuple(f, new_value);
            });
        } else {
            using ObsValue = typename remove_optional<ReturnType>::type;
            auto resulting_observable = std::make_shared<Observable<ObsValue>>();
            icey_dfg_graph_.add_vertex_with_parents(resulting_observable, {parent->index.value()}); /// And add to the graph

            parent->on_change([resulting_observable, f=std::move(f)](const auto &new_value) {
                auto ret = call_if_tuple(f, new_value);
                if constexpr (is_optional_v<ReturnType>) {
                    if(ret) {
                        resulting_observable->_set(*ret);
                    }
                } else {
                    resulting_observable->_set(ret);
                }
            });
            return resulting_observable;
        }
    }

    /// Now we can construct higher-order filters.

    /// For a tuple-observable, get it's N'th element
    template<int index, class Parent>
    auto get_nth(Parent & parent) { 
        return then(parent, [](const auto &... args) { /// Need to take variadic because then() automatically unpacks tuples
            return std::get<index>(std::forward_as_tuple(args...)); /// So we need to pack this again in a tuple
        });
    }

    bool empty() const { return icey_dfg_graph_.vertices.empty(); }
    void clear() { icey_dfg_graph_.vertices.clear(); }

    /// Register callback to be called after all parameters have been attached

    /// For the class-based API, to enable override
    virtual void after_parameter_initialization() {
        if(after_parameter_initialization_cb_)
            after_parameter_initialization_cb_();
    }
    /// Called in the destructor
    virtual void on_node_destruction_cb() {
        if(on_node_destruction_cb_)
            on_node_destruction_cb_();
    }

    /// For the functional API
    void register_after_parameter_initialization_cb(std::function<void()> cb) {
        after_parameter_initialization_cb_ = cb;
    }
    
    void register_on_node_destruction_cb(std::function<void()> cb) {
        on_node_destruction_cb_ = cb;
    }
protected:
    /// These are prepended with ICEY because with derive from rclcpp::Node and have to avoid name collisions
    Graph icey_dfg_graph_;
    bool icey_was_initialized_{false}; /// Indicates whether icey_initialize() was called. Used to ensure the graph is static, i.e. no items are added after initially initilizing.

    std::function<void()> after_parameter_initialization_cb_;
    std::function<void()> on_node_destruction_cb_;
    
    void assert_icey_was_not_initialized() {
        if(icey_was_initialized_) 
            throw std::invalid_argument("You are not allowed to add signals after ICEY was initialized. The graph must be static");
    }

    template<class Value1, class Value2>
    void icey_connect(Value1 parent, Value2 child) {
        parent->on_change([child](const auto &new_value) { child->_set(new_value); });
    }
};

/// The ROS node, additionally owning the data-flow graph (DFG) that contains the observables 
class ROSNodeWithDFG : public ROSAdapter::Node, public Context {
public:
    using NodeBase = ROSAdapter::Node;
    using NodeBase::NodeBase;
    
    Context &icey() { return *this; } /// Upcast to disambiguate calls 

    /// This attaches all the ICEY signals to the node, meaning it creates the subcribes etc. It initializes everything in a pre-defined order.
    void icey_initialize() {
        if(icey_dfg_graph_.vertices.empty()) {
            std::cout << "WARNING: Nothing to spawn, try first to create some signals/states" << std::endl;
            return;
        }
        attach_everything_to_node();
        icey_was_initialized_ = true;
    }

protected:
    void attach_everything_to_node() {
        if(icey_debug_print)
            std::cout << "[icey::Context] attach_everything_to_node() start" << std::endl;
        /// First, sort the attachables by priority: first parameters, then publishers, services, subsribers etc.
        /// We could do bin-sort here which runs in linear time, but the standard library does not have an implementation for it.
        std::sort(icey_dfg_graph_.vertices.begin(), icey_dfg_graph_.vertices.end(), 
            [](const auto &attachable1, const auto &attachable2) { return attachable1.data->attach_priority() 
                < attachable2.data->attach_priority();});
        size_t i_vertex = 0;
        /// Now attach everything to the ROS-Node, this creates the parameters, publishers etc.
        /// Now, allow for attaching additional nodes after we got the parameters. After attaching, parameters immediatelly have their values. 
        /// For this, first attach only the parameters (prio 0)
        if(icey_debug_print)
            std::cout << "[icey::Context] Attaching parameters ..." << std::endl;
        for(; i_vertex < icey_dfg_graph_.vertices.size(); i_vertex++) {
            auto &vertex = icey_dfg_graph_.vertices.at(i_vertex);
            if(vertex.data->attach_priority() == 1) /// Stop after attachning all parameters
                break;
            vertex.data->attach_to_node(*this); /// Attach
        }
        if(icey_debug_print)
            std::cout << "[icey::Context] Attaching parameters finished." << std::endl;
        after_parameter_initialization(); /// Call user callback. Here, more vertices may be added 

        if(icey_debug_print)
            std::cout << "[icey::Context] Analyzing data flow graph ... " << std::endl;
        analyze_dfg(); /// Now analyze the graph before attaching everything to detect cycles as soon as possible, especially before the node is started and everything crashes

        if(icey_debug_print)
            std::cout << "[icey::Context] Maybe new vertices... " << std::endl;
        /// If additional vertices have been added to the DFG, attach it as well
        for(; i_vertex < icey_dfg_graph_.vertices.size(); i_vertex++) {
            auto &vertex = icey_dfg_graph_.vertices.at(i_vertex);
            vertex.data->attach_to_node(*this); /// Attach
        }
        if(icey_debug_print)
            std::cout << "[icey::Context] attach_everything_to_node() finished. " << std::endl;
    }

    /// TODO 
    void analyze_dfg() {
        /// do_topological_sort_on_dfg_graph()
        // assert_dfg_graph_is_acyclic() 
        /// analyze_dependencies() -> computes the number of threads needed
    }
};

/// Blocking spawn of an existing node. 
void spawn(std::shared_ptr<ROSAdapter::Node> node, bool use_mt=true) {
    if(use_mt) {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    } else {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
}

void spin_nodes(const std::vector<std::shared_ptr<ROSNodeWithDFG>> &nodes) {
    auto num_threads = 2; /// TODO how many do we need ?
    /// This is how nodes should be composed according to ROS maintainer wjwwood: https://robotics.stackexchange.com/a/89767
    /// He references https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), num_threads);
    for(const auto &node : nodes)  
        executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}

/// API aliases 
using Node = ROSNodeWithDFG;

}

#include <icey/functional_api.hpp>
