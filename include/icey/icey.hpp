#pragma once 

#include <iostream> 
#include <fmt/core.h>
#include <fmt/ostream.h>

#include <functional>
#include <tuple>
#include <map>
#include <optional>
#include <unordered_map>
/// TODO figure out where boost get's pulled in, we do not explicitly depend on it, but it's a dependecy of one 
#include <boost/noncopyable.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

#include <icey/bag_of_metaprogramming_tricks.hpp>

namespace icey {

bool icey_debug_print = false;
using Time = ROS2Adapter::Time;
class Context;

/// A node in the DFG-graph, corresponds to a node-attachable
class DFGNode {
public:
    /// We bould the node starting from the root, to allow a context-free graph creation, we refer to the parents
    std::weak_ptr<Context> context;
    std::optional<size_t> index; /// The index of this observable in list of vertices in the data-flow graph. We have to store it here because of cyclic dependency. None if this observable was not attached to the graph yet.
};

/// Everything that can be "attached" to a node, publishers, subscribers, clients, TF subscriber etc.
class NodeAttachable : private boost::noncopyable {
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

struct AttachableNode  : public DFGNode, public NodeAttachable {

};
/// An observable. Similar to a promise in JS.
/// TODO consider CRTP, would also be beneficial for PIMPL 
template<typename _Value, typename _ErrorValue = std::string>
class Observable : public AttachableNode, public std::enable_shared_from_this<Observable<_Value, _ErrorValue>> {
    friend class Context; /// To prevent misuse, only the Context is allowed to register on_change callbacks
public:
    using Value = _Value;
    using ErrorValue = _ErrorValue;
    using Self = Observable<_Value, _ErrorValue>;

    using OnResolve = std::function<void(const Value&)>;
    using OnReject = std::function<void(const ErrorValue&)>;

    /// Creates a new Observable that changes it's value to y every time the value x of the parent observable changes, where y = f(x).
    template<typename F>
    auto then(F && f) { return this->context.lock()->then(this->shared_from_this(), f); }

    /// Create a ROS publisher for this observable.
    template<typename MessageT>
    void publish(const std::string &topic_name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
        return this->context.lock()->create_publisher(this->shared_from_this(), topic_name, qos);
    }
protected:

    /// Register to be notified when smth. changes
    void on_change(OnResolve && cb) {
        notify_list_.emplace_back(std::move(cb)); /// TODO rename to children ?
    }
    /// TODO protect

    /// set and notify all observers about the new value
    virtual void _set(const Value &new_value) {
        for(auto cb: notify_list_) cb(new_value);
        
    }

    void _reject(const ErrorValue &error) {
        for(auto cb: reject_cbs_) cb(error);
    }

    std::vector<OnResolve> notify_list_;
    std::vector<OnReject> reject_cbs_;
};

/// An observable storing the last received value
template<typename _Value>
struct BufferedObservable : public Observable<_Value> {
    using Base = Observable<_Value>;
    using Value = _Value;
    using MaybeValue = std::optional<Value>;
    virtual bool has_value() const {return value_.has_value(); }
    virtual const Value &value() const { value_.value(); }
protected:
    /// Buffers the value and calls the base method
    void _set(const Value &new_value) override {
        value_ = new_value;
        Base::_set(new_value);
    }
    /// The last received value.
    MaybeValue value_;
};

/// An interpolatable observable is one that buffers the incoming values using a circular buffer and allows to query the message at a given point, optionally using interpolation.
/// It is an essential for using lookupTransform with TF.
/// It is used by synchronizers to synchronize a topic exactly at a given time point. 
template<typename _Value>
struct InterpolateableObservable : public BufferedObservable<_Value> {
    using Value = _Value;
    using Base = BufferedObservable<_Value>;    
    using MaybeValue = typename Base::MaybeValue;
    /// Get the closest measurement to a given time point. Returns nothing if the buffer is empty or an extrapolation would be required.
    virtual MaybeValue get_at_time(const Time &time_point) const = 0;
};

/// An observable that holds the last received value. Fires initially an event if a default_value is set
template<typename _Value>
class ParameterObservable : public BufferedObservable<_Value> {
public:
    using Value = _Value;
    using Base = BufferedObservable<_Value>;
    using MaybeValue = typename Base::MaybeValue;

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
    const Value &value() const override {
        /// TODO do we want to allow this if the user provided a default value ? I think not, since it adds conceptual inconcistency to the otherwise inaccessible Observables
        if(!this->value_.has_value()) { /// First, check if this observable was accessed before spawning. This is needed to provide a nice  error message since confusing syncronous code will likely happen for users.
            throw std::runtime_error("[parameter '" + parameter_name_ + "'] You cannot access the parameters before spawning the node, you can only access them inside callbacks (which are triggered after calling icey::spawn())");
        }
        return this->value_.value();
    }

protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) override {
        if(icey_debug_print)
            std::cout << "[ParameterObservable] attaching parameter '" + parameter_name_ + "' to node ..." << std::endl;
        /// Declare on change handler
        node_handle.declare_parameter<Value>(parameter_name_, default_value_, 
            [this](const rclcpp::Parameter &new_param) {
                Value new_value = new_param.get_value<Value>();
                this->value_ = new_value; /// Store value
                this->_set(new_value); /// notify
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

template<typename _Value>
class SubscriptionObservable : public Observable<_Value> {
public:
    using Value = _Value;
    SubscriptionObservable(const std::string &name, const ROSAdapter::QoS &qos, 
        const rclcpp::SubscriptionOptions &options): name_(name), qos_(qos), options_(options) {
        this->attach_priority_ = 3;
    }

protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) override {
        if(icey_debug_print)
            std::cout << "[SubscriptionObservable] attach_to_node()" << std::endl;
        node_handle.add_subscription<Value>(name_, [this](std::shared_ptr<Value> new_value) {
            this->_set(*new_value);
        }, qos_, options_);
    }

    std::string name_;
    ROSAdapter::QoS qos_;
    const rclcpp::SubscriptionOptions options_;
};

/// A subscription for single transforms. It implements InterpolateableObservable but by using lookupTransform, not an own buffer 
struct TransformSubscriptionObservable : public InterpolateableObservable<geometry_msgs::msg::TransformStamped> {
public:
    TransformSubscriptionObservable(const std::string &target_frame, const std::string &source_frame) : 
         target_frame_(target_frame), source_frame_(source_frame) { 
            this->attach_priority_ = 3;
            }

    /// TODO override: Is TF buffer empty ? -> Needed to distinguitsh TF errors
    virtual bool has_value() const { return 
        this->value_.has_value(); 
    }
protected:
    MaybeValue get_at_time(const Time &time_point) const override {
        std::optional<geometry_msgs::msg::TransformStamped> tf_msg;
        try {
            // Note that this call does not wait, the transform must already be arrived. This works because get_at_time()
            // is called by the synchronizer as 
            tf_msg = tf2_listener_->buffer_.lookupTransform(target_frame_, source_frame_, time_point);
        } catch (tf2::TransformException & e) {

        }
        return tf_msg;
    }

    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        if(icey_debug_print)
            std::cout << "[TransformSubscriptionObservable] attach_to_node()" << std::endl;
        tf2_listener_ = node_handle.add_tf_subscription(target_frame_, source_frame_, 
            [this](const geometry_msgs::msg::TransformStamped &new_value) {
            this->_set(new_value);
        });
    }
    
    std::shared_ptr<ROSAdapter::TFListener> tf2_listener_;
    std::string target_frame_;
    std::string source_frame_;
};

/// Timer signal, saves the number of ticks as the value and also passes the timerobject as well to the callback
struct TimerObservable: public Observable<size_t> {
    using Value = size_t;
    TimerObservable(const ROSAdapter::Duration &interval, bool use_wall_time, bool is_one_off_timer) : 
        interval_(interval), use_wall_time_(use_wall_time), is_one_off_timer_(is_one_off_timer) { 
            this->attach_priority_ = 5;
            }
    
protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {      
        if(icey_debug_print)
            std::cout << "[TimerObservable] attach_to_node()" << std::endl;  
        ros_timer_ = node_handle.add_timer(interval_, use_wall_time_, [this] () {
            this->_set(ticks_counter_++);
            if(is_one_off_timer_)
                ros_timer_->cancel();
        });
    }
    rclcpp::TimerBase::SharedPtr ros_timer_;
    ROSAdapter::Duration interval_;
    bool use_wall_time_{false};
    bool is_one_off_timer_{false};
    Value ticks_counter_{0};
};

/// A publishabe state, read-only
template<typename _Value>
class PublisherObservable : public Observable<_Value> {
public:
    using Value = _Value;
    static_assert(rclcpp::is_ros_compatible_type<Value>::value, "A publisher must use a publishable ROS message (no primitive types are possible)");

    PublisherObservable(const std::string &topic_name, const ROSAdapter::QoS qos=ROS2Adapter::DefaultQos()) : topic_name_(topic_name), qos_(qos) {
        this->attach_priority_ = 1; 
    }

protected:
    void attach_to_node(ROSAdapter::NodeHandle & node_handle) {
        if(icey_debug_print)
            std::cout << "[PublisherObservable] attach_to_node()" << std::endl;
        auto publish = node_handle.add_publication<Value>(topic_name_, qos_);
        this->on_change([publish](const auto &new_value) { publish(new_value); });
    }

    std::string topic_name_;
    ROSAdapter::QoS qos_{ROS2Adapter::DefaultQos()};
};

/// A publishabe state, can be written. Needed where we need to publish by reacting on external events 
// that are not in control of ROS. This is needed for hardware drivers for example
template<typename _Value>
class WritablePublisherObservable : public PublisherObservable<_Value> {
public:
    using Value = _Value;
    void publish(const Value &message) {
        this->_set(message);
    }
};

/// An adapter, adapting the message_filters::SimpleFilter to our Observable (two different implementations of almost the same concept).
/// Does nothing else than what message_filters::Subscriber does:
/// https://github.com/ros2/message_filters/blob/humble/include/message_filters/subscriber.h#L349
// We neeed the base to be able to recognize interpolatable nodes for example
template<typename _Value, class Base = Observable<_Value> >
struct SimpleFilterAdapter : public Base, public message_filters::SimpleFilter<_Value> {
    using Value = _Value;
    SimpleFilterAdapter() {
        this->on_change([this](const Value &msg) {
            using Event = message_filters::MessageEvent<const Value>;
            /// TODO HACK, fix properly, do not deref in sub
            auto msg_ptr = std::make_shared<Value>(msg);
            this->signalMessage(Event(msg_ptr));
        });
    }
};

/// Wrap the message filters package 
/// TODO this needs to check whether all inputs have the same QoS, so we will have do a walk
/// TODO adapt queue size automatically if we detect very different frequencies so that synchronization still works. 
/// I would guess it works if the lowest frequency topic has a frequency of at least 1/queue_size, if the highest frequency topic has a frequency of one.
template<typename... Messages>
class SynchronizerObservable : public Observable< std::tuple<const typename Messages::ConstPtr ...> > { 
public:
    using Self = SynchronizerObservable<Messages...>;
    using Value = std::tuple<const typename Messages::ConstPtr ...>;
    /// Approx time will work as exact time if the stamps are exactly the same, so I wonder why the `TImeSynchronizer` uses by default ExactTime
    using Policy = message_filters::sync_policies::ApproximateTime<Messages...>;
    using Sync = message_filters::Synchronizer<Policy>;
    using Inputs = std::tuple< std::shared_ptr<SimpleFilterAdapter<Messages>>... >;
    SynchronizerObservable(uint32_t queue_size) :
         inputs_(std::make_shared<SimpleFilterAdapter<Messages>>()...), /// They are dynamically allocated as is every other Observable
         queue_size_(queue_size) {
            
        synchronizer_ = std::make_shared<Sync>(Policy(queue_size_));
        /// Connect with the input observables
        std::apply([this](auto &... input_filters) { synchronizer_->connectInput(*input_filters...);},inputs_);
        synchronizer_->setAgePenalty(0.50); /// TODO not sure why this is needed, present in example code
        synchronizer_->registerCallback(&Self::sync_callback, this); /// That is the only overload that we can use here that works with the legacy pre-variadic template code
    }

    const auto &inputs() const { return inputs_; }
private:
    void sync_callback(typename Messages::ConstPtr... messages) {
        this->_set(std::forward_as_tuple(messages...));
    }

    /// The input filters
    uint32_t queue_size_{10};

    Inputs inputs_;
    std::shared_ptr<Sync> synchronizer_;
    
};

// A transform broadcaster observable 
class TransformPublisherObservable : public Observable<geometry_msgs::msg::TransformStamped> {
public:
    using Value = geometry_msgs::msg::TransformStamped;
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
    using Value = std::pair<Request, Response>;

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
    using Value = Response;
    /// The type of the required input
    using Parent = std::shared_ptr< Observable<  std::shared_ptr<Request> > >;

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

struct Nothing {};
/// A dev/null observable, i.e. a observable that cannot have children (you realize now that there are much worse names for this, right ?).
struct DevNullObservable : public Observable<Nothing> { };

/// A graph, owning the observables TODO decide on base-class, since the graph only needs to own the data, we could even use std::any
struct DataFlowGraph {
    /// TODO store the computation 
    struct EdgeData {};
    using VertexData = std::shared_ptr<AttachableNode>;
    /// vecS needed so that the vertex id's are consecutive integers
    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexData, EdgeData>;

    /// Create a new node from data
    /// TODO we can accept a std::array here actually and avoid the dynamic mem alloc
    void add_vertex_with_parents(std::shared_ptr<AttachableNode> vertex_data, const std::vector<size_t> &parents) {
        vertex_data->index = add_vertex(vertex_data, graph_); /// TODO very ugly, but Obs needs to know the index so we can connect them here
        for(const auto &parent : parents) {   
            add_edge(parent, vertex_data->index.value(), EdgeData{}, graph_);
        }
    }
    /// Returns an iterable of all the vertices
    auto get_vertices_index_range() { return boost::make_iterator_range(vertices(graph_)); }

    bool empty() const { return num_vertices(graph_) == 0; }
    void clear() { graph_.clear(); }

    void topo_sort() {
        //std::vector< VertexData > c;
        //topological_sort(graph_, std::back_inserter(c));
    }

    Graph graph_;
};

/// A context, essentially the data-flow graph. Basis for the class-based API of ICEY.
struct Context : public std::enable_shared_from_this<Context> {
    template<typename ParameterT>
    auto declare_parameter(const std::string &name, const std::optional<ParameterT> &maybe_default_value= std::nullopt, 
        const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor(), 
            bool ignore_override = false) {
        return create_observable<ParameterObservable<ParameterT>>(name, maybe_default_value, parameter_descriptor, ignore_override);
    }

    template<typename MessageT>
    auto create_subscription(const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos(),
        const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions()) {
        return create_observable< SubscriptionObservable<MessageT> >(name, qos, options);
    }

    auto create_transform_subscription(const std::string &target_frame, const std::string &source_frame) {
        return create_observable<TransformSubscriptionObservable>(target_frame, source_frame);
    }

    template<typename MessageT>
    void create_publisher(std::shared_ptr<Observable<MessageT>> parent, const std::string &topic_name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
        create_observable_with_parent<PublisherObservable<MessageT>>(parent->index.value(), topic_name, qos);
    }
    
    void create_transform_publisher(std::shared_ptr<Observable<geometry_msgs::msg::TransformStamped>> parent) {
        create_observable_with_parent<TransformPublisherObservable>(parent->index.value());
    }
    
    auto create_timer(const ROSAdapter::Duration &interval, bool use_wall_time = false, bool is_one_off_timer = false) {
        return create_observable<TimerObservable>(interval, use_wall_time, is_one_off_timer);
    }
    
    template<typename ServiceT> 
    auto create_service(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
        return create_observable<ServiceObservable<ServiceT>>(service_name, qos);
    }

    /// Add a service client
    template<typename ServiceT> 
    auto create_client(typename ClientObservable<ServiceT>::Parent parent, 
         const std::string & service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
        return create_observable<ClientObservable<ServiceT>>(service_name, qos);
    }
    
    /// Synchronize observables using approximate time
    template<typename... Parents>
    auto synchronize(Parents ... parents) { 
        uint32_t queue_size = 10; // TODO update automatically 
        assert_icey_was_not_initialized();        
        /// TODO The following is duped with create_observable_with_parents because we need to connect many inputs to many outputs, 
        /// maybe solve with vector then. 
        auto sync = std::make_shared< SynchronizerObservable< typename remove_shared_ptr_t<Parents>::Value ...> >(queue_size);
        sync->context = shared_from_this();
        /// Connect the inputs of the synchronizer to the parents
        std::apply([&](const auto &... inputs) {( icey_connect(parents, inputs), ...);}, sync->inputs());
        data_flow_graph_.add_vertex_with_parents(sync, std::vector<size_t>(parents->index.value() ...));
        return sync;
    }

    /// Here are the filters. First, the most basic filter: fuse. It fuses the inputs and updates the output if any of the inputs change.
    /// Parents must be of type Observable
    /// TODO Think this filter through, it is like Promise.any but it requres that all were received at least once etc.
    /*template<typename... Parents>
    auto fuse(Parents && ... parents) { 
        /// Remote shared_ptr TODO write proper type trait for this
        using ReturnType = std::tuple<typename std::remove_reference_t<decltype(*parents)>::Value...>;
        auto resulting_observable = std::make_shared< Observable<ReturnType> >();  /// A result is rhs, i.e. read-only
        data_flow_graph_.add_vertex_with_parents(resulting_observable, {parents->index.value()...}); /// And add to the graph
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
    }*/

    /// Serialize, pipe arbitrary number of parents of the same type into one. Needed for the control-flow where the same publisher can be called from multiple callbacks
    template<typename... Parents>
    auto serialize(Parents ... parents) { 
        /// TODO assert all types are the same -> cannot take variable number of same type arguments
        using Parent = decltype(*std::get<0>(std::forward_as_tuple(parents...)));
        using ParentValue = typename std::remove_reference_t<Parent>::Value;
        /// First, create a new observable
        auto child = create_observable_with_parents<Observable<ParentValue>>({parents->index.value()...});
        /// Now connect each parent with the child with the idendity function
        ([&]{ 
            const auto cb = [child](const auto &new_val) {
                child->_set(new_val);
            };
            /// This should be called "parent", "parent->on_change()". This is essentially a for-loop over all parents, but C++ cannot figure out how to create a readable syntax, instead we got these fold expressions
            parents->on_change(cb);
        }(), ...);
        return child; /// Return the underlying pointer. We can do this, since internally everything is stored reference-counted.
    }
    /*
    template<class ParentValue>
    auto serialize(std::shared_ptr<ParentValue> ... parents) { 
        auto child = create_observable_with_parents<Observable<ParentValue>>(std::forward_as_tuple(parents...));
        ([&]{ 
            const auto cb = [child](const auto &new_val) {
                child->_set(new_val);
            };
            /// This should be called "parent", "parent->on_change()". This is essentially a for-loop over all parents, but C++ cannot figure out how to create a readable syntax, instead we got these fold expressions
            parents->on_change(cb);
        }(), ...);
        return child; /// Return the underlying pointer. We can do this, since internally everything is stores reference-counted.
    }
    */

    /// When reference updates, we pass all others parents, 
    /// TODO need BufferedObservable for this, but using this filter is discouraged

    /*
    template<class Reference, class... Parents>
    auto sync_with_reference(Reference && reference, Parents && ... parents) { 
        /// Remote shared_ptr TODO write proper type trait for this
        using ReturnType = std::tuple<Reference, typename std::remove_reference_t<decltype(*parents)>::Value...>;
        auto resulting_observable = std::make_shared< Observable<ReturnType> >();  /// A result is rhs, i.e. read-only
        data_flow_graph_.add_vertex_with_parents(resulting_observable, {reference, parents->index.value()...}); /// And add to the graph
        const auto f_continued = [resulting_observable, reference](auto&&... parents) {
            auto all_argunments_arrived = (parents->value_ && ... && true);
            if(all_argunments_arrived) 
                resulting_observable->_set(std::make_tuple(reference, parents->value_.value()...));
        };
        /// This should be called "parent", "parent->on_change()". This is essentially a for-loop over all parents, but C++ cannot figure out how to create a readable syntax, instead we got these fold expressions
        reference->on_change(std::bind(f_continued, std::forward<Parents>(parents)...));
        return resulting_observable; /// Return the underlying pointer. We can do this, since internally everything is stores reference-counted.
    }
    */

    template<typename F>
    auto unpack_if_calling_tuple(F && f) {
        return [f=std::move(f)](const auto &new_value) {
                apply_if_tuple(f, new_value);
        };
    }

    template<typename Parent, typename F>
    auto then(Parent parent, F && f) {     
        // assert parent has index
        using ReturnType = decltype( apply_if_tuple(f, std::declval<typename remove_shared_ptr_t<Parent>::Value>()) );
        if constexpr (std::is_void_v<ReturnType>) {
            /// Create a dummy node to be able to add the edge in the graph
            auto child = create_observable_with_parent<DevNullObservable>(parent->index.value());
            //connect(parent, child)
            parent->on_change(unpack_if_calling_tuple(f));
        } else {
            using ObsValue = typename remove_optional<ReturnType>::type;
            auto child = parent->context.lock()->template create_observable_with_parent<Observable<ObsValue>>(parent->index.value());

            const auto f_continued = [child, f=std::move(f)](const auto &new_value) {
                auto ret = apply_if_tuple(f, new_value);
                if constexpr (is_optional_v<ReturnType>) {
                    if(ret) {
                        child->_set(*ret);
                    }
                } else {
                    child->_set(ret);
                }
            };
            parent->on_change(f_continued);
            return child;
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

    // TODO maybe rem, derive from graph
    bool empty() const { return data_flow_graph_.empty(); }
    void clear() { data_flow_graph_.clear(); }

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

    /// This attaches all the ICEY signals to the node, meaning it creates the subcribes etc. It initializes everything in a pre-defined order.
    void initialize(ROSAdapter::NodeHandle &node) {
        if(empty()) {
            std::cout << "WARNING: Nothing to spawn, try first to create some signals/states" << std::endl;
            return;
        }
        attach_everything_to_node(node);
        was_initialized_ = true;
    }

protected:    
    void assert_icey_was_not_initialized() {
        if(was_initialized_) 
            throw std::invalid_argument("You are not allowed to add signals after ICEY was initialized. The graph must be static");
    }
    
    /// Overload for source observables, ones that have no parents
    template<class O, typename... Args>
    std::shared_ptr<O> create_observable(Args &&... args) {
        return create_observable_with_parent<O>(std::nullopt, std::forward<Args>(args)...);
    }
    
    /// Overload for a single parent
    template<class O, typename... Args>
    std::shared_ptr<O> create_observable_with_parent(std::optional<size_t> maybe_parent_id, Args &&... args) {
        std::vector<size_t> parent_ids;
        if(maybe_parent_id) {
            parent_ids.push_back(*maybe_parent_id);
        }
        return create_observable_with_parents<O>(parent_ids, std::forward<Args>(args)...);
    }

    /// Creates a new observable of type O using by passing it args to the constructor and adds it to the graph. The parents are added as edges to the graph for which only the vertex id's are needed .
    /// It does not connect the observables together, meaning they cannot communicate after calling this but instead icey_connect must be called
    template<class O, typename... Args>
    std::shared_ptr<O> create_observable_with_parents(const std::vector<size_t> &parent_ids, Args &&... args) {
        assert_icey_was_not_initialized();
        auto observable = std::make_shared<O>(std::forward<Args>(args)...);
        observable->context = shared_from_this();
        data_flow_graph_.add_vertex_with_parents(observable, parent_ids);
        return observable;
    }
    
    /// Connects the parent with it's child. TODO we should not allow different Value types, we currently only need this for the service that receives a request and stores 
    /// *both* the request and the respose. After fixing the service, this should be "icey_connect(Obs<Value> parent, Obs<Value> child)"
    template<class Value1, class Value2>
    void icey_connect(Value1 parent, Value2 child) {
        parent->on_change([child](const auto &new_value) { child->_set(new_value); });
    }

    template<class Value1, class Value2>
    void create_connection(Value1 parent, Value2 child) {
        parent->on_change([child](const auto &new_value) { child->_set(new_value); });
    }

    void attach_everything_to_node(ROSAdapter::NodeHandle &node) {
        if(icey_debug_print)
            std::cout << "[icey::Context] attach_everything_to_node() start" << std::endl;
        /// First, sort the attachables by priority: first parameters, then publishers, services, subsribers etc.
        /// We could do bin-sort here which runs in linear time, but the standard library does not have an implementation for it.

        auto vertex_index_range = data_flow_graph_.get_vertices_index_range();
        std::vector<int> vertex_index_range_copy(vertex_index_range.begin(), vertex_index_range.end());
        /// TODO WE NEED TO SORT AGAIN BELOW
        std::sort(vertex_index_range_copy.begin(), vertex_index_range_copy.end(), 
            [this](const auto &v1, const auto &v2) { return data_flow_graph_.graph_[v1]->attach_priority() 
                < data_flow_graph_.graph_[v2]->attach_priority();});

        size_t i_vertex = 0;
        /// Now attach everything to the ROS-Node, this creates the parameters, publishers etc.
        /// Now, allow for attaching additional nodes after we got the parameters. After attaching, parameters immediatelly have their values. 
        /// For this, first attach only the parameters (prio 0)
        if(icey_debug_print)
            std::cout << "[icey::Context] Attaching parameters ..." << std::endl;
        for(; i_vertex < num_vertices(data_flow_graph_.graph_); i_vertex++) {
            const auto &attachable = data_flow_graph_.graph_[i_vertex];
            if(attachable->attach_priority() == 1) /// Stop after attachning all parameters
                break;
            attachable->attach_to_node(node); /// Attach
        }
        if(icey_debug_print)
            std::cout << "[icey::Context] Attaching parameters finished." << std::endl;
        after_parameter_initialization(); /// Call user callback. Here, more vertices may be added 

        if(icey_debug_print)
            std::cout << "[icey::Context] Analyzing data flow graph ... " << std::endl;
        analyze_dfg(); /// Now analyze the graph before attaching everything to detect cycles as soon as possible, especially before the node is started and everything crashes

        if(icey_debug_print)
            std::cout << "[icey::Context] Maybe new vertices... " << std::endl;
        /// TODO WE NEED TO SORT AGAIN BELOW
        // node_attach_priority = binary predicate
        // get_vertex_in_order(node_attach_priority)
        /// If additional vertices have been added to the DFG, attach it as well 
        for(; i_vertex < num_vertices(data_flow_graph_.graph_); i_vertex++) {
            const auto &attachable = data_flow_graph_.graph_[i_vertex];
            attachable->attach_to_node(node); /// Attach
        }
        if(icey_debug_print)
            std::cout << "[icey::Context] attach_everything_to_node() finished. " << std::endl;
    }

    /// TODO 
    void analyze_dfg() {
        data_flow_graph_.topo_sort();
        /// do_topological_sort_on_dfg_graph()
        // assert_dfg_graph_is_acyclic() 
        /// analyze_dependencies() -> computes the number of threads needed
    }
    
    DataFlowGraph data_flow_graph_;
    bool was_initialized_{false}; /// Indicates whether initialize() was called. Used to ensure the graph is static, i.e. no items are added after initially initilizing.

    std::function<void()> after_parameter_initialization_cb_;
    std::function<void()> on_node_destruction_cb_;
};

/// The ROS node, additionally owning the data-flow graph (DFG) that contains the observables. 
/// This class is needed to ensure that the context lives for as long as the node lives.
class ROSNodeWithDFG : public ROSAdapter::Node {
public:
    using Base = ROSAdapter::Node;
    using Base::Base; // Take over all base class constructors
    void icey_initialize() {
        icey().initialize(*this);
    }
    Context &icey() { return *this->icey_context_; } 
    std::shared_ptr<Context> icey_context_{std::make_shared<Context>()};
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
