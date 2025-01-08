/// Provides functional API by declaring a global variable
#pragma once 

/// Global state, used to enable a simple, purely functional API and to notify for misuse. 
/// It simply stages all operations that are to be performed before the node is created and then flushes them once icey::spawn is called.
struct GState {
    std::shared_ptr<ROSNodeWithDFG> node;

    /// Create the global state node and flush the staged icey-observables from the global state
    void create_node(std::string name) {
        node = std::make_shared<ROSNodeWithDFG>(name);
        /// TODO maybe copy base
        node->icey_dfg_graph_ = staged_context.icey_dfg_graph_;
        node->icey_node_attachables_ = staged_context.icey_node_attachables_;
        node->icey_initialize();
    }

    Context staged_context;
    
    ~GState() {
        if(!staged_context.icey_dfg_graph_.vertices.empty() && !node) {
            std::cout << "WARNING: You created some signals/states/timers, but no node was created, did you forget to call icey::spawn() ?" << std::endl;
        }
    }
};

GState g_state;

template<typename StateValue>
auto create_signal(const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) { 
    return g_state.staged_context.create_signal<StateValue>(name, qos); 
};

auto create_transform_signal(const std::string &frame1, const std::string &frame2) {
    return g_state.staged_context.create_transform_signal(frame1, frame2);
}

template<typename StateValue>
auto create_state(const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos(), std::optional<double> max_frequency = std::nullopt) {
    return g_state.staged_context.create_state<StateValue>(name, qos, max_frequency);
}

template<typename CallbackT> 
void create_timer(const ROSAdapter::Duration &interval, CallbackT && callback) {
    g_state.staged_context.create_timer(interval, std::move(callback));
}

template<typename ServiceT, typename CallbackT> 
void create_service(const std::string &service_name, CallbackT && callback, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    g_state.staged_context.create_service<ServiceT>(service_name, std::move(callback), qos);
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
    g_state.create_node(node_name);
    spawn(argc, argv, g_state.node);
}

/// Enable API icey::node
std::shared_ptr<Node> &node = g_state.node;