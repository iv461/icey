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
    virtual void attach_to_node(const std::shared_ptr<ROSAdapter::NodeHandle> &) {}
};

/// Just a helper for timers etc. TODO solve more elegantly
struct NodeAttachableFunctor : NodeAttachable {
    using F = std::function<void(const std::shared_ptr<ROSAdapter::NodeHandle> &)>;
    explicit NodeAttachableFunctor(F f): f(f) {}
    void attach_to_node(const std::shared_ptr<ROSAdapter::NodeHandle> &node_handle) {   
        f(node_handle);
    }
    F f;
};

/// TODO do we need this ? Only used for graph. A read-only observable, with no value. Atatchnbel to ROS-node 
/// TODO everything that captures this in a lambda should be noncopyable. Currently there are only the subscribers. 
/// But how do we achive transparently copying around only references ?
struct ObservableBase : public NodeAttachable, private boost::noncopyable {
    size_t index{0}; /// The index of this observable in list of vertices in the data-flow graph. We have to store it here becasue
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

    /// Create another promise as a child. (this is very similar to JavaScript's .then() )
    /*template<typename F>
    auto then(F && f) {
        then(*this, f);
    }*/

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

    static auto create(const std::string &name) {
        return std::make_shared<Self>(name);
    }
    
    virtual void attach_to_node(const std::shared_ptr<ROSAdapter::NodeHandle> &node_handle) {
        node_handle->add_subscription<StateValue>(name_, [this](const StateValue &new_value) {
            this->_set(new_value);
        });
    }

    SubscribedState(const std::string &name): name_(name) {}
    std::string name_; 
};


/// A signal for subscribing to /tf and obtaining a transform.
struct TransformSignal : public Observable<geometry_msgs::msg::TransformStamped> {
public:
    using Base = Observable<geometry_msgs::msg::TransformStamped>;
    using StateValue = Base::StateValue;
    
    static auto create(const std::string &frame1, const std::string &frame2) {
        return std::make_shared<TransformSignal>(frame1, frame2);
    }

    virtual void attach_to_node(const std::shared_ptr<ROSAdapter::NodeHandle> &node_handle) {
        node_handle->add_tf_subscription(frame1_, frame2_, [this](const StateValue &new_value) {
            this->_set(new_value);
        });
    }

    /// TODO make ctor private
    TransformSignal(const std::string &frame1, const std::string &frame2) : 
        frame1_(frame1), frame2_(frame2) {}

    std::string frame1_;
    std::string frame2_;
  
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


    /// TODO: Accept here Qos and other settings the create_publisher normally accepts
    void publish(const std::string &name, std::optional<double> max_frequency = std::nullopt) {
        //static_assert(rclcpp::is_ros_compatible_type<StateValue>::value, "The function has to return a publishable ROS message (no primitive types are possible)");
        name_ = name;
        max_frequency = max_frequency;
    }

    void attach_to_node(const std::shared_ptr<ROSAdapter::NodeHandle> &node_handle) {
        if(name_ == "") {
            return; /// Do not do anything if we should not publish.
        }

        auto publish = node_handle->add_publication<StateValue>(name_, max_frequency_);
        this->on_change([publish](const auto &new_value) {
            std::cout << "[PublishableState] value changed, publishing .." << std::endl;
            publish(new_value);
        });
    }

    std::string name_;
    std::optional<double> max_frequency_;
};

/// A node in the data-flow graph.
template<typename Data>
struct Node {
    explicit Node(const Data &_data): data(_data) {}
    Data data;
    std::vector<size_t> in_edges;
    std::vector<size_t> out_edges;
};

/// A graph, owning the observables TODO decide on base-class, since the graph only needs to own the data, we could even use std::any
struct Graph {
    using NodeData = std::shared_ptr<ObservableBase>;
    using NodeT = Node<NodeData>;
    /// Create a new node from data
    NodeT& add_vertex(const NodeData &node_data) {
        node_data->index = vertices.size(); /// TODO very ugly, but Obs needs to know the index
        vertices.emplace_back(node_data);
        return vertices.back();
    }
    NodeT& add_vertex_with_parents(const NodeData &node_data, const std::vector<size_t> &parents) {
        auto &new_node = add_vertex(node_data);
        new_node.in_edges = parents;
        return new_node;
    }
    std::vector<NodeT> vertices;
};

/// The ROS node, owning the data-flow graph (DFG) that contains the observables 
struct ROSNodeWithDFG {
    Graph graph;
    std::shared_ptr<ROSAdapter::Node> node;
};

/// Global state, used to enable a simple, purely functional API and to notify for mis-use
struct GState: public ROSNodeWithDFG {
    std::vector<NodeAttachable> staged_node_attachables; /// Other attachables that do not require storing in the node
    ~GState() {
        if(!graph.vertices.empty() && !node) {
            std::cout << "WARNING: You created some signals/states/timers, but no node was created, did you forget to call icey::spawn() ?" << std::endl;
        }
    }
};

GState g_state;

/// Enable API icey::node
std::shared_ptr<ROSAdapter::Node> &node = g_state.node;

template<typename StateValue>
auto create_signal(const std::string &name) {
    /// TODO ASSERT no node
    auto signal = SubscribedState<StateValue>::create(name);
    /// Attach to graph and return vertex
    g_state.graph.add_vertex(signal);
    return signal;
}

auto create_transform_signal(const std::string &frame1, const std::string &frame2) {
    /// TODO ASSERT no node
    auto tf_signal = TransformSignal::create(frame1, frame2);
    /// Attach to graph and return vertex
    g_state.graph.add_vertex(tf_signal);
    return tf_signal;
}

/// A writable signal, i.e. publisher
template<typename StateValue>
auto create_state(const std::string &name, std::optional<double> max_frequency = std::nullopt) {
    /// TODO ASSERT no node
    auto state = PublishableState<StateValue>::create();
    state->publish(name, max_frequency);
    g_state.graph.add_vertex(state);
    return state;
}


template<typename F> 
void create_timer(const ROSAdapter::Duration &interval, F cb) {
    const auto timer_attachable = NodeAttachableFunctor([interval, cb = std::move(cb)](const auto &node_handle) 
        { node_handle->add_timer(interval, std::move(cb)); });
    g_state.staged_node_attachables.push_back(timer_attachable);
}

/// Provide a service 
template<typename CallbackT> 
void create_service(const std::string &service_name, CallbackT && callback, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    const auto service_attachable = NodeAttachableFunctor([service_name, callback = std::move(callback), qos](const auto &node_handle) 
        { node_handle->add_service(service_name, std::move(callback), qos); });
    g_state.staged_node_attachables.push_back(service_attachable);
}

/// Add a service client
template<typename Service> 
void create_client(const std::string & service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    const auto client_attachable = NodeAttachableFunctor([service_name, qos](const auto &node_handle) 
        { node_handle->template add_client<Service>(service_name, qos); });
    g_state.staged_node_attachables.push_back(client_attachable);
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
    g_state.graph.add_vertex_with_parents(resulting_observable, node_parents); /// And add to the graph

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
    auto resulting_observable = PublishableState<ReturnType>::create();
    g_state.graph.add_vertex_with_parents(resulting_observable, {parent->index}); /// And add to the graph

    parent->on_change([resulting_observable, f=std::move(f)](const auto &new_value) {
        auto result = f(new_value);
        resulting_observable->_set(result);  
    });
    return resulting_observable;
}


/// This initializes a node with a name and attaches everything to it. It initializes everything in a pre-defined order.
std::shared_ptr<ROSAdapter::Node> create_node(std::optional<std::string> node_name = std::nullopt) {
    if(g_state.graph.vertices.empty()) {
        std::cout << "WARNING: Nothing to spawn, try first to create some signals/states" << std::endl;
        return {};
    }
    /// TODO [Feature] spawn anonymous node
    auto concrete_name = node_name.has_value() ? node_name.value() : std::string("jighe385");

    auto node = std::make_shared<ROSAdapter::Node>(concrete_name);

    /// TODO FIRST DO TOPO SORT, FIRST ATTACH PARAMETERS, THEN SUBS, THEN PUBS
    /// First, attach to the ROS node all vertices in the DFG 
    for( auto &vertex : g_state.graph.vertices) {
        vertex.data->attach_to_node(g_state.node); /// Attach
    }
    /// Then, add all other misc 
    for( auto &attachable : g_state.staged_node_attachables) {
        attachable.attach_to_node(g_state.node); /// Attach
    }
    return node;
}

/// Blocking spawn of an existing node
void spawn(int argc, char **argv, std::shared_ptr<ROSAdapter::Node> node) {
    rclcpp::init(argc, argv);
    rclcpp::spin(node);
    /// TODO
    //staged_node_attachables.clear();
    rclcpp::shutdown();
}

/// Blocking spawn of a node using the global state
void spawn(int argc, char **argv, 
    std::optional<std::string> node_name = std::nullopt) {
    g_state.node = create_node(node_name);
    if(g_state.node)
        spawn(argc, argv, g_state.node);
}

}
