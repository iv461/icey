/// Provides functional API by declaring a global variable
#pragma once 

namespace icey {

struct MultiNodeWrap {
    std::unordered_map<std::string, std::shared_ptr<ROSNodeWithDFG>> nodes;
    std::shared_ptr<ROSNodeWithDFG> operator()(const std::string &node_name) {
        if (!nodes.count(node_name)) {
            throw std::runtime_error("There is no node called '" + node_name + "', do you have a typo or did you forget to first call icey::spawn() ?");
        }
        return nodes.at(node_name);
    }
    Node *operator->() {
        //// Get the first node, assuming there is only one
        if(nodes.empty()) {
            throw std::runtime_error("There are no nodes, did you forget to first call icey::spawn() ?");
        } else if (nodes.size() != 1) {
            throw std::runtime_error("More than one node was spawned, you need to use icey::node(name) when accessing the node");
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
        static_cast<Context&>(*node) = staged_context;
        /// After committing the changes, clear them so that another node can be spawned
        staged_context.icey_dfg_graph_.vertices.clear();
        node->icey_initialize();
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

template<typename MessageT>
auto create_subscription(const std::string &topic_name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) { 
    return g_state.staged_context.create_subscription<MessageT>(topic_name, qos); 
};

auto create_transform_subscription(const std::string &target_frame, const std::string &source_frame) {
    return g_state.staged_context.create_transform_subscription(target_frame, source_frame);
}

template<typename MessageT>
auto create_publisher(std::shared_ptr<Observable<MessageT>> parent, const std::string &topic_name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
    return g_state.staged_context.create_publisher<MessageT>(parent, topic_name, qos);
}

auto create_timer(const ROSAdapter::Duration &interval, bool use_wall_time = false, bool is_one_off_timer = false) {
    return g_state.staged_context.create_timer(interval, use_wall_time, is_one_off_timer);
}

template<typename ServiceT> 
auto create_service(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    return g_state.staged_context.create_service<ServiceT>(service_name, qos);
}

template<typename ServiceT> 
auto create_client(typename ClientObservable<ServiceT>::Parent parent, 
         const std::string & service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
            return g_state.staged_context.create_client<ServiceT>(parent, service_name, qos);
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
    if(!rclcpp::contexts::get_global_default_context()->is_valid()) /// Create a context if it is the first spawn
        rclcpp::init(argc, argv);
    spawn(g_state.create_node(node_name));
}

/// Create a node from the staged global state. Clears the global state so that multiple nodes can be created
auto create_node(int argc, char **argv, std::string node_name) {
    if(!rclcpp::contexts::get_global_default_context()->is_valid()) /// Create a context if it is the first spawn
        rclcpp::init(argc, argv);
    return g_state.create_node(node_name); 
}

}