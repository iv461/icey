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
struct ObservableBase : public NodeAttachable, private boost::noncopyable {
    size_t index{0}; /// The index of this observable in list of vertices in the data-flow graph. We have to store it here becasue
};

/// An observable holding a value
template<typename _StateValue>
class Observable : public ObservableBase {
public:
    using StateValue = _StateValue;
    using Cb = std::function<void(const StateValue&)>;

    /// Register to be notified when smth. changes
    void on_change(Cb cb) {
        notify_list_.push_back(cb);
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

    static auto create(const std::string &name) {
        return std::make_shared<Self>(name);
    }
    
    void attach_to_node(const std::shared_ptr<ROSAdapter::NodeHandle> &node_handle) {
        node_handle->add_subscription<StateValue>(name_, [this](const StateValue &new_value) {
            this->_set(new_value);
        });
    }

    SubscribedState(const std::string &name): name_(name) {}
    std::string name_; 
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

    void publish(const std::string &name, std::optional<double> max_frequency = std::nullopt) {
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

/// Global node object adapter, used to conveniently wrap getting the underlying node
/// TODO  we need smth. to interface the global vairbale like icey::node
struct GlobNode {
    std::shared_ptr<ROSAdapter::NodeHandle> node;
};

/// The ROS node, owning the data-flow graph (DFG) that contains the observables 
struct ROSNodeWithDFG {
    Graph graph;
    std::shared_ptr<ROSAdapter::NodeHandle> node;
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

template<typename StateValue>
auto create_signal(const std::string &name) {
    /// TODO ASSERT no node
    auto signal = SubscribedState<StateValue>::create(name);
    /// Attach to graph and return vertex
    g_state.graph.add_vertex(signal);
    return signal;
}

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
        { node_handle->add_timer(interval, cb); });
    g_state.staged_node_attachables.push_back(timer_attachable);
}

/// Provide a service 
template<typename F> 
void create_service(std::string service_name, F cb) {
    const auto service_attachable = NodeAttachableFunctor([service_name, cb = std::move(cb)](const auto &node_handle) 
        { node_handle->add_service(service_name, cb); });
    g_state.staged_node_attachables.push_back(service_attachable);
}

/// Parents must be of type Observable
template<typename F, typename... Parents>
auto compute_based_on(F f, Parents && ... parents) { 
    /*static_assert(std::is_invocable_v<decltype(f)>, ///TODO does not work 
                  "The first argument to compute_based_on() must be a callable");
    static_assert(std::is_invocable_v<decltype(f), parents...>,
                  "The given function to compute_based_on() must be callable with all the arguments that were given, that are of the same type of the subscribed ROS messages");*/
    using ReturnType = decltype(f(parents->value_.value()...));
    static_assert(rclcpp::is_ros_compatible_type<ReturnType>::value, "The function has to return a publishable ROS message (no primitive types are possible)");

    auto resulting_observable = PublishableState<ReturnType>::create();  /// A result is rhs, i.e. read-only    

    const std::vector<size_t> node_parents{parents->index...};
    g_state.graph.add_vertex_with_parents(resulting_observable, node_parents); /// And add to the graph

     ([&]{ 
        const auto f_continued = [resulting_observable, f](auto&&... parents) {
            auto all_argunments_arrived = (parents && ... && true);
            if(all_argunments_arrived) {
                auto result = f(parents->value_.value()...);
                resulting_observable->_set(result);
            }
        };
        parents->on_change(std::bind(f_continued, std::forward<Parents>(parents)...));
     }(), ...);
    return resulting_observable;
}

/// Blocking spawn of a node
void spawn(int argc, char **argv, 
    std::optional<std::string> node_name = std::nullopt) {

    if(g_state.graph.vertices.empty()) {
        std::cout << "WARNING: Nothing to spawn, try first to create some signals/states" << std::endl;
        return;
    }

    rclcpp::init(argc, argv);

    /// TODO [Feature] spawn anonymous node
    auto concrete_name = node_name.has_value() ? node_name.value() : std::string("jighe385");

    g_state.node = std::make_shared<ROSAdapter::Node>(concrete_name);

    /// TODO FIRST DO TOPO SORT, FIRST ATTACH PARAMETERS, THEN SUBS, THEN PUBS
    /// First, attach to the ROS node all vertices in the DFG 
    for( auto &vertex : g_state.graph.vertices) {
        vertex.data->attach_to_node(g_state.node); /// Attach
    }
    /// Then, add all other misc 
    for( auto &attachable : g_state.staged_node_attachables) {
        attachable.attach_to_node(g_state.node); /// Attach
    }
    

    rclcpp::spin(g_state.node);

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
