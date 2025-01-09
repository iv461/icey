/// Provides functional API by declaring a global variable
#pragma once 

namespace icey {

struct MultiNodeWrap {
    std::unordered_map<std::string, std::shared_ptr<ROSNodeWithDFG>> nodes;
    std::shared_ptr<ROSNodeWithDFG> operator()(const std::string &node_name) {
        if (!nodes.count(node_name)) {
            throw std::runtime_error("There is no node called '" + node_name + "'");
        }
        return nodes.at(node_name);
    }
    Node *operator->() {
        //// Get the first node, assuming there is only one
        if(nodes.empty()) {
            throw std::runtime_error("There were no nodes spawned");
        } else if (nodes.size() != 1) {
            throw std::runtime_error("More than one node was spawned, so you need to use icey::node(name) when accessing the node!");
        }
        return nodes.begin()->second.get();
    }
};

/// Global state, used to enable a simple, purely functional API and to notify for misuse. 
/// It simply stages all operations that are to be performed before the node is created. 
// Then, once icey::spawn is called, it creates a node and flushes the operations.
struct GState {
    MultiNodeWrap nodes;

    /// Create the global state node and flush the staged icey-observables from the global state
    auto create_node(const std::string &name) {
        nodes.nodes.emplace(name, std::make_shared<ROSNodeWithDFG>(name));
        auto node = nodes.nodes.at(name);
        /// TODO maybe copy base
        node->icey_dfg_graph_ = staged_context.icey_dfg_graph_;
        node->icey_initialize();

        /// After committing the changes, 
        staged_context.icey_dfg_graph_.vertices.clear();
        return node;
    }

    Context staged_context;
    
    ~GState() {
        if(!staged_context.icey_dfg_graph_.vertices.empty() && nodes.nodes.empty()) {
            std::cout << "WARNING: You created some signals/states/timers, but no node was created, did you forget to call icey::spawn() ?" << std::endl;
        }
    }
};

GState g_state;

/// Enable API icey::node
auto &node = g_state.nodes;

template<typename StateValue>
auto create_subscription(const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) { 
    return g_state.staged_context.create_subscription<StateValue>(name, qos); 
};

auto create_transform_subscription(const std::string &frame1, const std::string &source_frame) {
    return g_state.staged_context.create_transform_subscription(frame1, source_frame);
}

template<typename StateValue>
auto create_publisher(std::shared_ptr<Observable<StateValue>> parent, const std::string &topic_name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
    return g_state.staged_context.create_publisher<StateValue>(parent, topic_name, qos);
}

auto create_timer(const ROSAdapter::Duration &interval, bool use_wall_time = false) {
    return g_state.staged_context.create_timer(interval, use_wall_time);
}

template<typename ServiceT> 
auto create_service(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    return g_state.staged_context.create_service<ServiceT>(service_name, qos);
}

template<typename Service> 
auto create_client(typename ClientObs<Service>::Parent parent, 
         const std::string & service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
            return g_state.staged_context.create_client<Service>(parent, service_name, qos);
}

 /// Now the filters
template<typename... Parents>
auto fuse(Parents && ... parents) { 
    return g_state.staged_context.fuse(std::forward<Parents>(parents)...);
}

template<typename Parent, typename F>
auto then(Parent &parent, F && f) {
    return g_state.staged_context.then(parent, std::move(f));
}

/// Blocking spawn of a node using the global state
void spawn(int argc, char **argv, std::string node_name) {
    rclcpp::init(argc, argv);
    return spawn(argc, argv, g_state.create_node(node_name));
}

/// non-blocking spawn of a node using the global state
auto spawn_async(int argc, char **argv, std::string node_name) {
    if(!rclcpp::contexts::get_global_default_context()) /// Create a context if it is the first spawn
        rclcpp::init(argc, argv);
    return spawn_async(argc, argv, g_state.create_node(node_name));
}

}