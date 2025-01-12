/// Provides functional API by declaring a global variable
#pragma once 

namespace icey {

/// Global state, used to enable a simple, purely functional API.
/// It simply stages all operations that are to be performed before the node is created.
// Then, once icey::spawn is called, it creates a node and flushes the creation of subscribers, publishers etc.
struct GlobalState {
    /// Access an already spawned node by it's name, icey::node(<name>). This method is usefull when multiple nodes are spawned in the same process.
    std::shared_ptr<ROSNodeWithDFG> operator()(const std::string &node_name) {
        if (!nodes.count(node_name)) {
            throw std::runtime_error("There is no node called '" + node_name + "', do you have a typo or did you forget to first call icey::spawn() ?");
        }
        return nodes.at(node_name);
    }

    /// Access the node, in case only one was spawned, e.g. icey::node->get_logger()
    Node *operator->() { return get_node(); }
    /// Allow using this node in a context where a rclrpp::Node is needed, mind that ROSNodeWithDFG derives from rclrpp::Node
    operator ROSNodeWithDFG*() { return get_node();; }

    auto create_new_node(const std::string &name) {
        nodes.emplace(name, std::make_shared<ROSNodeWithDFG>(name));    
        auto node = nodes.at(name); 
        currently_initializing_node_ = node; /// Assign so that adding new vertices in icey_initialize() uses the existing context
        node->icey_context_ = staged_context;
        /// After committing the context to the new node, create a new empty context so that another node can be spawned later
        staged_context = std::make_shared<Context>();
        node->icey_initialize();
        currently_initializing_node_ = {}; // We are done with initializing
        return node;
    }

    Context& get_context() {
        if(currently_initializing_node_) /// If we are configuring the graph in the after_parameters() callback
            return *currently_initializing_node_->icey_context_;
        return *staged_context;
    }

    ~GlobalState() {
        if(!staged_context->empty()) {
            std::cout << "WARNING: You created some signals/states/timers, but no node was created, did you forget to call icey::spawn() ?" << std::endl;
        }
    }

private:
    // Access the node, in case only one was spawned
    ROSNodeWithDFG *get_node() {
        if(nodes.empty()) {
            throw std::runtime_error("There are no nodes, did you forget to first call icey::spawn() ?");
        } else if (nodes.size() != 1) {
            throw std::runtime_error("More than one node was spawned, you need to use icey::node(<name>) when accessing the node.");
        }
        //// Get the first node after having checked there is only one
        return nodes.begin()->second.get();
    }

    std::unordered_map<std::string, std::shared_ptr<ROSNodeWithDFG>> nodes;
    std::shared_ptr<Context> staged_context{std::make_shared<Context>()}; /// Must be a shared_ptr because of observables reference it
    std::shared_ptr<ROSNodeWithDFG> currently_initializing_node_; /// The node that is currently in the initialization phase after the ROS-parameters have been obtained but adding stuff to the graph is still possible.
};

GlobalState g_state;

/// Enable API icey::node
auto &node = g_state;

template<typename ParameterT>
auto declare_parameter(const std::string &name, const std::optional<ParameterT> &maybe_default_value=std::nullopt, 
        const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor(), 
            bool ignore_override = false) {
    return g_state.get_context().declare_parameter<ParameterT>(name, maybe_default_value, parameter_descriptor, ignore_override);
}
template<typename MessageT>
auto create_subscription(const std::string &topic_name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos(),
        const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions()) { 
    return g_state.get_context().create_subscription<MessageT>(topic_name, qos, options); 
};

auto create_transform_subscription(const std::string &target_frame, const std::string &source_frame) {
    return g_state.get_context().create_transform_subscription(target_frame, source_frame);
}

template<typename MessageT>
void create_publisher(std::shared_ptr<Observable<MessageT>> parent, const std::string &topic_name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
    g_state.get_context().create_publisher<MessageT>(parent, topic_name, qos);
}

void create_transform_publisher(std::shared_ptr<Observable<geometry_msgs::msg::TransformStamped>> parent){
    g_state.get_context().create_transform_publisher(parent);
}

auto create_timer(const ROSAdapter::Duration &interval, bool use_wall_time = false, bool is_one_off_timer = false) {
    return g_state.get_context().create_timer(interval, use_wall_time, is_one_off_timer);
}

template<typename ServiceT> 
auto create_service(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    return g_state.get_context().create_service<ServiceT>(service_name, qos);
}

template<typename ServiceT> 
auto create_client(typename ClientObservable<ServiceT>::Parent parent, 
         const std::string & service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
            return g_state.get_context().create_client<ServiceT>(parent, service_name, qos);
}

/// Now some extra events 

/// Register something that is called immediatelly after we received all paramters from ROS
void after_parameter_initialization(std::function<void()> cb) {
    return g_state.get_context().register_after_parameter_initialization_cb(cb);
}

void on_node_destruction(std::function<void()> cb) {
    return g_state.get_context().register_on_node_destruction_cb(cb);
}


/// Now the filters
template<typename... Parents>
auto synchronize(Parents && ... parents) { 
    return g_state.get_context().synchronize(std::forward<Parents>(parents)...);
}

/*template<typename... Parents>
auto fuse(Parents && ... parents) { 
    return g_state.get_context().fuse(std::forward<Parents>(parents)...);
}*/

template<typename... Parents>
auto serialize(Parents && ... parents) { 
    return g_state.get_context().serialize(std::forward<Parents>(parents)...);
}

template<typename Parent, typename F>
auto then(Parent &parent, F && f) {
    return g_state.get_context().then(parent, std::move(f));
}

template<int index, class Parent>
auto get_nth(Parent & parent) { 
    return g_state.get_context().get_nth<index, Parent>(parent);
}

/// Blocking spawn of a node using the global state
void spawn(int argc, char **argv, std::string node_name) {
    if(!rclcpp::contexts::get_global_default_context()->is_valid()) /// Create a context if it is the first spawn
        rclcpp::init(argc, argv);
    spawn(g_state.create_new_node(node_name));
}

/// Create a node from the staged global state. Clears the global state so that multiple nodes can be created
auto create_node(int argc, char **argv, std::string node_name) {
    if(!rclcpp::contexts::get_global_default_context()->is_valid()) /// Create a context if it is the first spawn
        rclcpp::init(argc, argv);
    return g_state.create_new_node(node_name); 
}

}