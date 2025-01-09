/// Provides functional API by declaring a global variable
#pragma once 

/// Global state, used to enable a simple, purely functional API and to notify for misuse. 
/// It simply stages all operations that are to be performed before the node is created. 
// Then, once icey::spawn is called, it creates a node and flushes the operations.
struct GState {
    std::shared_ptr<ROSNodeWithDFG> node;

    /// Create the global state node and flush the staged icey-observables from the global state
    void create_node(const std::string &name) {
        node = std::make_shared<ROSNodeWithDFG>(name);
        /// TODO maybe copy base
        node->icey_dfg_graph_ = staged_context.icey_dfg_graph_;
        node->icey_initialize();
        staged_context.icey_dfg_graph_.vertices.clear();
    }

    Context staged_context;
    
    ~GState() {
        if(!staged_context.icey_dfg_graph_.vertices.empty() && !node) {
            std::cout << "WARNING: You created some signals/states/timers, but no node was created, did you forget to call icey::spawn() ?" << std::endl;
        }
    }
};

GState g_state;
/// Enable API icey::node
std::shared_ptr<Node> &node = g_state.node;

template<typename StateValue>
auto create_subscription(const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) { 
    return g_state.staged_context.create_subscription<StateValue>(name, qos); 
};

auto create_transform_subscription(const std::string &frame1, const std::string &source_frame) {
    return g_state.staged_context.create_transform_subscription(frame1, source_frame);
}

template<typename StateValue>
auto create_publisher(const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos(), std::optional<double> max_frequency = std::nullopt) {
    return g_state.staged_context.create_publisher<StateValue>(name, qos, max_frequency);
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
    g_state.create_node(node_name);
    spawn(argc, argv, g_state.node);
}
