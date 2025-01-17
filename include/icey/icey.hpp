#pragma once

#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <tuple>
#include <unordered_map>

/// TODO figure out where boost get's pulled in, we do not explicitly depend on it, but it's a
/// dependecy of one of our deps. It's not rclcpp and not message_filters.
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/mp11.hpp>  /// For template meta-programming, operation on tuples etc.
#include <boost/hana.hpp>
#include <boost/hana/ext/std/tuple.hpp> /// Needed so that we do not need the custom hana tuples everywhere: https://stackoverflow.com/a/34318002
#include <boost/noncopyable.hpp>
#include <boost/type_index.hpp>
#include <icey/bag_of_metaprogramming_tricks.hpp>

namespace mp = boost::mp11;
namespace hana = boost::hana;

namespace icey {

bool icey_debug_print = false;
using Time = ROS2Adapter::Time;
class Context;

/// A node in the DFG-graph
class DFGNode {
public:
  /// We bould the node starting from the root, to allow a context-free graph creation, we refer to
  /// the parents
  std::weak_ptr<Context> context;
  /// The index of this observable in list of vertices in the data-flow graph. We have
  /// to store it here because of cyclic dependency. None if this observable was not
  /// attached to the graph yet.
  std::optional<size_t> index;
  // The class name, i.e. the type, for example SubscriberObservable<>
  std::string class_name;
  /// A name to identify it among multiple with the same type, usually the topic
  /// or service name
  std::string name;

  /// TODO this looks clean but it really isn't, nodes should not know whether 
  using RunGraphEngineCb = std::function<void(size_t)>;
  void register_run_graph_engine(RunGraphEngineCb cb) { run_graph_engine_ = cb; }
  RunGraphEngineCb run_graph_engine_;
};

/// Everything that can be "attached" to a node, publishers, subscribers, clients, TF subscriber
/// etc.
class NodeAttachable  {
public:
  virtual void attach_to_node(ROSAdapter::NodeHandle &) {
    if (this->was_attached_) throw std::invalid_argument("NodeAttachable was already attached");
    was_attached_ = true;
  }
  /// Priority at which to attach this, needed to implement an order of initialization:
  // 0: parameters
  // 1: publishers
  // 2: services
  // 3: subscribers
  // 4: clients
  // 5: timers
  virtual size_t attach_priority() const { return attach_priority_; }
  size_t attach_priority_{0};
  bool was_attached_{false};
};

/// Needed for storing in the graph as well as to let the compiler check for correct types
struct ObservableBase : public DFGNode, public NodeAttachable, private boost::noncopyable {
};

template <typename... Args>
struct observable_traits {
  static_assert(
      (is_shared_ptr<Args> && ...),
      "The arguments must be a shared_ptr< icey::Observable >, but it is not a shared_ptr");
  static_assert((std::is_base_of_v<ObservableBase, remove_shared_ptr_t<Args>> && ...),
                "The arguments must be an icey::Observable");
};

template <class T>
constexpr void assert_observable_holds_tuple() {
  static_assert(is_tuple_v<typename remove_shared_ptr_t<T>::Value>,
                "The Observable must hold a tuple as a value for unpacking.");
}

/// Some observable traits
template<class T> 
using obs_val = typename remove_shared_ptr_t<T>::Value;

template<class T> 
using obs_err = typename remove_shared_ptr_t<T>::ErrorValue;

template<class T> 
using obs_vor = typename remove_shared_ptr_t<T>::ValueORError;

/// The ROS message that a observable holds
template<class T>
using obs_msg = remove_shared_ptr_t<obs_val<T>>;

// Assert that all Observables types hold the same value
template <typename First, typename... Rest>
constexpr void assert_all_observable_values_are_same() {
  observable_traits<First, Rest...>{};  /// Only Observables are expected to have ::Value
  // Static assert that each T::Value is the same as First::Value
  static_assert((std::is_same_v<typename remove_shared_ptr_t<First>::Value,
                                typename remove_shared_ptr_t<Rest>::Value> &&
                 ...),
                "The values of all the observables must be the same");
}

struct Nothing {}; 

/// An observable. Similar to a promise in JS.
/// TODO consider CRTP, would also be beneficial for PIMPL
template <typename _Value, typename _ErrorValue = Nothing>
class Observable : public ObservableBase, public std::enable_shared_from_this<Observable<_Value, _ErrorValue>> {
  friend class Context;  /// To prevent misuse, only the Context is allowed to call set or
                         /// register_on_change_cb callbacks
public:
  using Value = _Value;
  using ErrorValue = _ErrorValue;
  using Self = Observable<Value, ErrorValue>;
  using MaybeValue = std::optional<Value>;
  using ValueORError = std::variant<std::monostate, Value, ErrorValue>;
  using Handler = std::function<void()>;

  /// Creates a new Observable that changes it's value to y every time the value x of the parent
  /// observable changes, where y = f(x).
  template <typename F>
  auto then(F &&f) {
    return this->context.lock()->template done<true>(this->shared_from_this(), f);
  }

  template <typename F>
  auto except(F &&f) {
    return this->context.lock()->template done<false>(this->shared_from_this(), f);
  }

  /// Create a ROS publisher and publish this observable.
  void publish(const std::string &topic_name,
               const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
    return this->context.lock()->create_publisher(this->shared_from_this(), topic_name, qos);
  }

  /// Call a service, this observable holds the request.
  /*auto call_service(const std::string &topic_name,
               const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
    return this->context.lock()->create_client(this->shared_from_this(), topic_name, qos);
  }*/

  virtual bool has_error() {
      return std::holds_alternative<ErrorValue>(value_);
  }

  virtual bool has_value() const { 
      return std::holds_alternative<Value>(value_); 
  }

  virtual const Value &value() const { return std::get<Value>(value_); }
  virtual const ErrorValue &error() const { return std::get<ErrorValue>(value_); }


//protected:
  void _register_handler(Handler cb) {
      handlers_.emplace_back(std::move(cb));
  }

  /// Set without notify
  void resolve(const MaybeValue &x) { 
      if(x) 
        std::get<Value>(value_) = x;
      else 
        value_ = std::monostate{};
  }
  void reject(const ErrorValue &x) { std::get<ErrorValue>(value_) = x; }

  void resolve_and_notify(const MaybeValue &x) {   
    this->resolve(x);
    this->notify();
  }
  void reject_and_notify(const ErrorValue &x) {
    this->reject(x);
    this->notify();
  }

  /*
  if (icey_debug_print)
      std::cout << "[" + this->class_name + ", " + this->name + "] _set was called" << std::endl;
  */  
  
  /// Notify about error or value, depending on the state. If there is no value, it does not notify
  void _notify() {
    if (run_graph_engine_) 
      run_graph_engine_(this->index.value());
  
    if(this->has_value() || this->has_error()) {
      for (auto cb : handlers_) cb();
    }
  }
  
  /// The last received value, it is buffered. It is buffered only to be able to do graph mode.
  ValueORError value_; 
  std::vector<Handler> handlers_;
  
};

struct DevNullObservable : public Observable<Nothing> {};

/// An observable that holds the last received value. Fires initially an event if a default_value is
/// set
template <typename _Value>
class ParameterObservable : public Observable<_Value> {
public:
  using Value = _Value;
  using Base = Observable<_Value>;
  using MaybeValue = typename Base::MaybeValue;

  /// TODO FIX ARG DUP, capture hana sequence maybe in cb maybe
  ParameterObservable(const std::string &parameter_name, const MaybeValue &default_value,
                      const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
                          rcl_interfaces::msg::ParameterDescriptor(),
                      bool ignore_override = false)
      : parameter_name_(parameter_name),
        default_value_(default_value),
        parameter_descriptor_(parameter_descriptor),
        ignore_override_(ignore_override) {
    this->attach_priority_ = 0;
    this->name = parameter_name_;
  }

  /// Parameters are initialized always at the beginning, so we can provide getters for the value.
  /// Note that has_value() must be checked beforehand since if no default value was provided, this
  /// function will throw std::bad_optional_access()
  const Value &value() const override {
    /// TODO do we want to allow this if the user provided a default value ? I think not, since it
    /// adds conceptual inconcistency to the otherwise inaccessible Observables
    if (!this->value_
             .has_value()) {  /// First, check if this observable was accessed before spawning. This
                              /// is needed to provide a nice  error message since confusing
                              /// syncronous code will likely happen for users.
      throw std::runtime_error(
          "[parameter '" + parameter_name_ +
          "'] You cannot access the parameters before spawning the node, you can only access them "
          "inside callbacks (which are triggered after calling icey::spawn())");
    }
    return this->value_.value();
  }

protected:
  void attach_to_node(ROSAdapter::NodeHandle &node_handle) override {
    if (icey_debug_print)
      std::cout << "[ParameterObservable] attaching parameter '" + parameter_name_ + "' to node ..."
                << std::endl;
    /// Declare on change handler
    node_handle.declare_parameter<Value>(
        parameter_name_, default_value_,
        [this](const rclcpp::Parameter &new_param) {
          Value new_value = new_param.get_value<Value>();
          this->resolve_and_notify(new_value);
        },
        parameter_descriptor_, ignore_override_);
    /// Set default value
    if (default_value_) {
      Value initial_value;
      node_handle.get_parameter_or(parameter_name_, initial_value, *default_value_);
      this->resolve_and_notify(initial_value);
    }
  }
  std::string parameter_name_;
  MaybeValue default_value_;
  rcl_interfaces::msg::ParameterDescriptor parameter_descriptor_;
  bool ignore_override_;
};

/// A subscriber observable, always stores a shared pointer to the message as it's value
template <typename _Message>
class SubscriptionObservable : public Observable < typename _Message::SharedPtr > {
  friend class Context;
public:
  using Value = typename _Message::SharedPtr;
  using Message = _Message;
  SubscriptionObservable(const std::string &name, const ROSAdapter::QoS &qos,
                         const rclcpp::SubscriptionOptions &options)
      : name_(name), qos_(qos), options_(options) {
    this->attach_priority_ = 3;
    this->name = name_;
  }

protected:
  void attach_to_node(ROSAdapter::NodeHandle &node_handle) override {
    if (icey_debug_print) std::cout << "[SubscriptionObservable] attach_to_node()" << std::endl;
    node_handle.add_subscription<Value>(
        name_, [this](typename Message::SharedPtr new_value) { this->resolve_and_notify(new_value); },
        qos_, options_);
  }

  std::string name_;
  ROSAdapter::QoS qos_;
  const rclcpp::SubscriptionOptions options_;
};

/// An interpolatable observable is one that buffers the incoming messages using a circular buffer and
/// allows to query the message at a given point, using interpolation. It is an essential
/// for using lookupTransform with TF. It is used by synchronizers to synchronize a topic exactly at
/// a given time point.
struct InterpolateableObservableBase {}; // TODO basically only there to be able to recognize observables in the synchronizer

template <typename T>
constexpr auto hana_is_interpolatable(T) {
    if constexpr(std::is_base_of_v<InterpolateableObservableBase, T>)
        return hana::bool_c<true>;
    else 
        return hana::bool_c<false>;    
}

template <typename _Message>
struct InterpolateableObservable : public InterpolateableObservableBase,
                                   public Observable < typename _Message::SharedPtr, std::string > {
  
  using MaybeValue = std::optional < typename _Message::SharedPtr >;
  /// Get the measurement at a given time point. Returns nothing if the buffer is empty or
  /// an extrapolation would be required.
  virtual MaybeValue get_at_time(const rclcpp::Time &time) const = 0;
};

/// A subscription for single transforms. It implements InterpolateableObservable but by using
/// lookupTransform, not an own buffer
struct TransformSubscriptionObservable
    : public InterpolateableObservable<geometry_msgs::msg::TransformStamped> {
public:
  using Message = geometry_msgs::msg::TransformStamped;
  using MaybeValue = InterpolateableObservable<Message>::MaybeValue;

  TransformSubscriptionObservable(const std::string &target_frame, const std::string &source_frame)
      : target_frame_(target_frame), source_frame_(source_frame) {
    this->attach_priority_ = 3;
    this->name = "source_frame: " + source_frame_ + ", target_frame: " + target_frame_;
  }
  
  MaybeValue get_at_time(const rclcpp::Time &time) const override {
    try {
      // Note that this call does not wait, the transform must already have arrived. This works
      // because get_at_time() is called by the synchronizer
      auto tf_msg = tf2_listener_->buffer_.lookupTransform(target_frame_, source_frame_, time);
      return std::make_shared<Message>(tf_msg); // For the sake of consistency, messages are always returned as shared pointers. Since lookupTransform gives us a value, we copy it over to a shared pointer.
    } catch (tf2::TransformException &e) {
      /// TODO notify except if registered
      return {};
    }
  }

protected:
  void attach_to_node(ROSAdapter::NodeHandle &node_handle) {
    if (icey_debug_print)
      std::cout << "[TransformSubscriptionObservable] attach_to_node()" << std::endl;
    tf2_listener_ = node_handle.add_tf_subscription(
        target_frame_, source_frame_,
        [this](const geometry_msgs::msg::TransformStamped &new_value) {
          this->resolve_and_notify(std::make_shared<Message>(new_value));
        });
  }

  std::shared_ptr<ROSAdapter::TFListener> tf2_listener_;
  std::string target_frame_;
  std::string source_frame_;
};


/// Timer signal, saves the number of ticks as the value and also passes the timerobject as well to
/// the callback
struct TimerObservable : public Observable<size_t> {
  using Value = size_t;
  TimerObservable(const ROSAdapter::Duration &interval, bool use_wall_time, bool is_one_off_timer)
      : interval_(interval), use_wall_time_(use_wall_time), is_one_off_timer_(is_one_off_timer) {
    this->attach_priority_ = 5;
    this->name = "timer";
  }

protected:
  void attach_to_node(ROSAdapter::NodeHandle &node_handle) {
    if (icey_debug_print) std::cout << "[TimerObservable] attach_to_node()" << std::endl;
    ros_timer_ = node_handle.add_timer(interval_, use_wall_time_, [this]() {
      this->resolve_and_notify(ticks_counter_++);
      if (is_one_off_timer_) ros_timer_->cancel();
    });
  }

  rclcpp::TimerBase::SharedPtr ros_timer_;
  ROSAdapter::Duration interval_;
  bool use_wall_time_{false};
  bool is_one_off_timer_{false};
  Value ticks_counter_{0};
};

/// A publishabe state, read-only. Value can be either a Message or shared_ptr<Message> 
template <typename _Value>
class PublisherObservable : public Observable<_Value> {
  friend class Context;

public:
  using Value = _Value;
  using Base = Observable<_Value>;
  using Message = remove_shared_ptr_t<Value>; 
  static_assert(rclcpp::is_ros_compatible_type< Message >::value,
                "A publisher must use a publishable ROS message (no primitive types are possible)");

  PublisherObservable(const std::string &topic_name,
                      const ROSAdapter::QoS qos = ROS2Adapter::DefaultQos())
      : topic_name_(topic_name), qos_(qos) {
    this->attach_priority_ = 1;
    this->name = topic_name;
  }

protected:
  void attach_to_node(ROSAdapter::NodeHandle &node_handle) {
    if (icey_debug_print) std::cout << "[PublisherObservable] attach_to_node()" << std::endl;
    auto publisher = node_handle.add_publication<Message>(topic_name_, qos_);
    this->_register_handler([this, publisher]() { 
                // We cannot pass over the pointer since publish expects a unique ptr and we got a shared_ptr. 
        // We cannot just create a unique_ptr because we cannot ensure we won't use the message even if use_count is one because use_count is meaningless in a multithreaded program. 
        const auto &new_value = this->value(); /// There can be no error
        if constexpr(is_shared_ptr<Value>)
          publisher(*new_value);
        else 
          publisher(new_value);
    });
  }

  std::string topic_name_;
  ROSAdapter::QoS qos_{ROS2Adapter::DefaultQos()};
};

struct AnyComputation {
  /// executes and notifies
  std::function<void()> f;
  bool requires_waiting_on_ros{false}; /// Is this computation a service call currently. 
};

/// Wrap the message_filters official ROS package. In the following, "MFL" refers to the
/// message_filters package. An adapter, adapting the message_filters::SimpleFilter to our
/// Observable (two different implementations of almost the same concept). Does nothing else than
/// what message_filters::Subscriber does:
/// https://github.com/ros2/message_filters/blob/humble/include/message_filters/subscriber.h#L349
// We neeed the base to be able to recognize interpolatable nodes for example
template <typename _Message, class _Base = Observable< typename _Message::SharedPtr >>
struct SimpleFilterAdapter : public _Base, public message_filters::SimpleFilter<_Message> {
  SimpleFilterAdapter() {
    this->_register_handler([this]() {
      using Event = message_filters::MessageEvent<const _Message>;
      const auto &new_value = this->value(); /// There can be no error
      this->signalMessage(Event(new_value));
    });
  }
};

/// TODO this needs to check whether all inputs have the same QoS, so we will have do a walk
/// TODO adapt queue size automatically if we detect very different frequencies so that
/// synchronization still works. I would guess it works if the lowest frequency topic has a
/// frequency of at least 1/queue_size, if the highest frequency topic has a frequency of one.
template <typename... Messages>
class SynchronizerObservable : public Observable<std::tuple<typename Messages::SharedPtr...>> {
public:
  using Self = SynchronizerObservable<Messages...>;
  using Value = std::tuple<typename Messages::SharedPtr...>;
  /// Approx time will work as exact time if the stamps are exactly the same, so I wonder why the
  /// `TImeSynchronizer` uses by default ExactTime
  using Policy = message_filters::sync_policies::ApproximateTime<Messages...>;
  using Sync = message_filters::Synchronizer<Policy>;
  using Inputs = std::tuple<std::shared_ptr<SimpleFilterAdapter<Messages>>...>;

  SynchronizerObservable(uint32_t queue_size)
      : inputs_(std::make_shared<SimpleFilterAdapter<Messages>>()...),  /// They are dynamically
                                                                        /// allocated as is every
                                                                        /// other Observable
        queue_size_(queue_size) {
    this->create_mfl_synchronizer();
  }

  const auto &inputs() const { return inputs_; }

private:
  void create_mfl_synchronizer() {
    synchronizer_ = std::make_shared<Sync>(Policy(queue_size_));
    /// Connect with the input observables
    std::apply([this](auto &...input_filters) { synchronizer_->connectInput(*input_filters...); },
               inputs_);
    synchronizer_->setAgePenalty(
        0.50);  /// TODO not sure why this is needed, present in example code
    synchronizer_->registerCallback(&Self::sync_callback,
                                    this);  /// That is the only overload that we can use here that
                                            /// works with the legacy pre-variadic template code
  }

  void sync_callback(typename Messages::SharedPtr... messages) {
    this->resolve_and_notify(std::forward_as_tuple(messages...));
  }

  /// The input filters
  uint32_t queue_size_{10};

  Inputs inputs_;
  std::shared_ptr<Sync> synchronizer_;
};

// A transform broadcaster observable
class TransformPublisherObservable : public Observable<geometry_msgs::msg::TransformStamped> {
  friend class Context;

public:
  using Value = geometry_msgs::msg::TransformStamped;

protected:
  void attach_to_node(ROSAdapter::NodeHandle &node_handle) {
    if (icey_debug_print)
      std::cout << "[TransformPublisherObservable] attach_to_node()" << std::endl;
    auto publish = node_handle.add_tf_broadcaster_if_needed();
    this->_register_handler([this, publish]() { 
        const auto &new_value = this->value(); /// There can be no error
        publish(new_value); 
    });
  }
};

/// A service observable, storing
template <typename _ServiceT>
struct ServiceObservable
    : public Observable<std::pair<std::shared_ptr<typename _ServiceT::Request>,
                                  std::shared_ptr<typename _ServiceT::Response>>> {
  using Request = std::shared_ptr<typename _ServiceT::Request>;
  using Response = std::shared_ptr<typename _ServiceT::Response>;
  using Value = std::pair<Request, Response>;

  ServiceObservable(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS())
      : service_name_(service_name), qos_(qos) {
    this->attach_priority_ = 2;
  }

protected:
  void attach_to_node(ROSAdapter::NodeHandle &node_handle) {
    if (icey_debug_print) std::cout << "[ServiceObservable] attach_to_node()" << std::endl;
    node_handle.add_service<_ServiceT>(
        service_name_,
        [this](Request request, Response response) {
          this->resolve_and_notify(std::make_pair(request, response));
        },
        qos_);
  }
  std::string service_name_;
  ROSAdapter::QoS qos_;
};

/// A service client is a remote procedure call (RPC). It is a computation, and therefore an edge in the DFG
template <typename _ServiceT>
struct ServiceClientComputation : public AnyComputation, public NodeAttachable {
  using Request = typename _ServiceT::Request::SharedPtr;
  using Response = typename _ServiceT::Response::SharedPtr;
  using Client = rclcpp::Client<_ServiceT>;
  using OutputObservable = Observable <Response, rclcpp::FutureReturnCode>;

  ServiceClientComputation(const std::string &service_name)
      : service_name_(service_name ){
    this->attach_priority_ = 4;
    this->name = service_name_;
    this->requires_waiting_on_ros = true;
  }

  void call(Request request) {
      using Future = typename Client::SharedFutureWithRequest;
      client_->async_send_request(
        request, [this](Future response_futur) { 
          if(response_futur.valid()) {
            this->output_->resolve_and_notify(response_futur.get());
          } else {
            /// TOOD
            this->output_->reject_and_notify(rclcpp::FutureReturnCode::TIMEOUT);
            /// TODO Do the weird cleanup thing
          }

      });
  }
  std::shared_ptr<OutputObservable> output_;
protected:
  void attach_to_node(ROSAdapter::NodeHandle &node_handle) {
    if (icey_debug_print) std::cout << "[ClientObservable] attach_to_node()" << std::endl;
    client_ = node_handle.add_client<_ServiceT>(service_name_);
  }
  typename Client::SharedPtr client_;
  std::string service_name_;
  //ROSAdapter::QoS qos_; TODO add for Iron/Jazzy, Humble had still the old APIs (rmw_*)
};

/// A graph, owning the observables TODO decide on base-class, since the graph only needs to own the
/// data, we could even use std::any
struct DataFlowGraph {
  /// TODO store the computation
  using EdgeData = AnyComputation;
  using VertexData = std::shared_ptr<ObservableBase>;
  /// vecS needed so that the vertex id's are consecutive integers
  using Graph =
      boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexData, EdgeData>;

  void add_v(VertexData vertex_data) {
    if (vertex_data->index.has_value())
      throw std::invalid_argument(
          "[DataFlowGraph::add_v] This vertex was already added to the graph, since it's index was "
          "set. Observables must be added only once to the graph");
    vertex_data->index = add_vertex(vertex_data,
                                    graph_);  /// TODO Obs needs to know the index so we can connect
                                              /// them here, can we do better ?
  }

  /// Create new edges from a child node to multiple parent nodes. Parent nodes must have been
  /// inserted already
  /// TODO we can accept a std::array here actually and avoid the dynamic mem alloc
  void add_edges(size_t child_id, const std::vector<size_t> &parent_ids,
                 const std::vector<EdgeData> &edges_data) {
    for (size_t i = 0; i < parent_ids.size(); i++) {
      const auto &parent_id = parent_ids.at(i);
      const auto &edge_data = edges_data.at(i);
      add_edge(parent_id, child_id, edge_data, graph_);
    }
  }

  // Returns an index remapping for the indices that sort them by a predicate. You should
  // never mutate the order of the vertices, meaning sorting them directly, but instead use this function. 
  //This is because the vertex data (the Observables) reference their index in the list of vertices.
  std::vector<size_t> get_vertices_ordered_by(std::function<bool(size_t, size_t)> predicate) {
    auto vertex_index_range = boost::make_iterator_range(vertices(graph_));
    std::vector<size_t> vertex_index_range_copy(vertex_index_range.begin(),
                                                vertex_index_range.end());
    std::sort(vertex_index_range_copy.begin(), vertex_index_range_copy.end(), predicate);
    return vertex_index_range_copy;
  }

  /// Returns an iterable of all the vertices
  auto get_vertices_index_range() { return boost::make_iterator_range(vertices(graph_)); }

  bool empty() const { return num_vertices(graph_) == 0; }
  void clear() { graph_.clear(); }

  // In case the graph is not a DAG, we compute the strong components to display to the user the
  // vertices that form loops
  std::vector<std::set<int>> compute_strong_components() {
    // Compute strongly connected components
    auto &g = graph_;
    std::vector<int> component(num_vertices(g));  // Component indices
    int num_scc = boost::strong_components(g, &component[0]);

    // Map each component to its vertices
    std::vector<std::set<int>> components(num_scc);
    for (size_t v = 0; v < component.size(); ++v) {
      components[component[v]].insert(v);
    }
    return components;
  }

  void print_strong_components(const std::vector<std::set<int>> &components) {
    // Identify and print vertices in cycles
    std::cout << "Vertices in cycles (including self-loops):" << std::endl;
    for (const auto &comp : components) {
      if (comp.size() > 1 ||
          (comp.size() == 1 && boost::edge(*comp.begin(), *comp.begin(), graph_).second)) {
        for (int v : comp) {
          std::cout << v << " ";
        }
        std::cout << std::endl;
      }
    }
  }

  std::optional<std::vector<size_t>> try_topological_sorting() {
    std::vector<size_t> topological_order;
    try {
      topological_sort(graph_, std::back_inserter(topological_order));
      /// Boost outputs the topological order in reversed order. ..
      std::reverse(topological_order.begin(), topological_order.end());
      return topological_order;
    } catch (const boost::not_a_dag &) {
      return {};
    }
  }

  void print() {
    auto &g = graph_;

    std::cout << "Created data-flow graph:\nVertices:" << std::endl;
    for (auto v : boost::make_iterator_range(vertices(g))) {
      std::cout << "Vertex " << v << ": " << g[v]->class_name << ": " << g[v]->name << std::endl;
    }

    // Print adjacency list
    std::cout << "\nAdjacency List:" << std::endl;
    for (auto e : boost::make_iterator_range(edges(g))) {
      auto source_vertex = source(e, g);
      auto target_vertex = target(e, g);
      std::cout << "Edge from Vertex " << source_vertex << " (" << g[source_vertex]->name
                << ") to Vertex " << target_vertex << " (" << g[target_vertex]->name << ")"
                << std::endl;
    }
  }

  Graph graph_;
};


 /// Graph engine, it executes the event propagation in graph mode given a data-flow graph.
 /// This event propagation executes computations over the eges. It is designed so that edges can contain Service calls 
 /// and therefore during the propagation, the GraphEngine can hand over to ROS at any time. When encountering an edge 
 /// that requries waiting on ROS, the propagation returns, remembering which edges it already propagated. When ROS triggers 
 /// an observable the next time, it continues to propagate.
struct GraphEngine {
  explicit GraphEngine(std::shared_ptr<DataFlowGraph>  graph) : data_flow_graph_(graph), 
        propagation_state_(graph) {}

  void analyze_data_flow_graph() {
    auto maybe_topological_order = data_flow_graph_->try_topological_sorting();
    if (!maybe_topological_order) {
      auto components = data_flow_graph_->compute_strong_components();
      data_flow_graph_->print_strong_components(components);
      throw std::runtime_error(
          "The graph created is not a directed acyclic graph (DAG), meaning it has loops. Please "
          "review the created vertices and remove the loops.");
    }
    topological_order_ = maybe_topological_order.value();
    if (icey_debug_print)
      data_flow_graph_->print();
  }

   /// Executes the graph mode. This function gets called if some of the inputs change and propagates
  /// the result. TODO Add literature reference, how is this algorithm called ? Found nothing in Tardos and Kleinberg
  void run_graph_mode(size_t signaling_vertex_id) {
    if (icey_debug_print)
      std::cout << "[icey::GraphEngine] Got event from vertex " << signaling_vertex_id
                << ", graph execution starts ..." << std::endl;

    auto &graph_ = data_flow_graph_->graph_;
    /// We traverse the vertices in the topological order and check which value changed.
    /// If a value changed, then for all its outgoing edges the computation is executed and these
    /// vertices are marked as changed.
    propagation_state_.vertex_changed_.at(signaling_vertex_id) = 1;
    size_t edge_index = 0;
    auto &v = propagation_state_.index_in_topo_order_;
    for (; v < topological_order_.size(); v++) {

      if (!propagation_state_.vertex_changed_.at(v)) {
        if (icey_debug_print)
          std::cout << "Vertex " << v << " did not change, skipping ..." << std::endl;
        continue;
      }
      if (icey_debug_print) std::cout << "Vertex " << v << " changed, propagating ..." << std::endl;

      for (auto edge : boost::make_iterator_range(out_edges(v, graph_))) {
        if(propagation_state_.propagated_edges_.at(edge_index) == 1) { /// TODO consider pre-computing CSR edges list.
          continue; /// Skip edges that were previously already propagated
        }
        edge_index++;
        auto target_vertex = target(edge, graph_);  // Get the target vertex
        auto &edge_data = graph_[edge];         // Access edge data

        /// Now this edge counts a propagated, regardless of whether we have to wait on ROS or not because when at the time ROS notifies us, the edge was already propagated.
        propagation_state_.propagated_edges_.at(edge_index) = 1;
        if(edge_data.requires_waiting_on_ros) {
          if (icey_debug_print)
            std::cout << "Edge execution requires waiting on ROS, returning...  " << target_vertex << " ..." << std::endl;  
          return; /// We return here, we will be called again when ROS is done
        } else {
          if (icey_debug_print)
            std::cout << "Executing edge to  " << target_vertex << " ..." << std::endl;
          if(edge_data.f) {
            edge_data.f();  /// Execute the computation, pass over the value
            /// And not notify ROS (Call this to enable publishers to work. Only publishers have something in the notify
            /// list)
          }
          propagation_state_.vertex_changed_.at(target_vertex) = 1; /// After executing the computation, the targed vertex changed.
        }
      }
    }
    propagation_state_.reset(); /// If we finished propagation, reset the state of the algorithm 
    if (icey_debug_print) std::cout << "Propagation finished. " << std::endl;
  }


  /// The propagation state remembers how far the propagation mode got the last time in case 
  /// we need to interrupt the propagation to wait on ROS. 
  struct PropagationState {
    explicit PropagationState(std::shared_ptr<DataFlowGraph> graph) {
      vertex_changed_.resize(num_vertices(graph->graph_), 0);
      propagated_edges_.resize(num_edges(graph->graph_), 0); 
    }

    /// When propagation finishes, this state is reset so that next time something changes, it is propagated again.
    void reset()  {
      index_in_topo_order_ = 0;
      std::fill(vertex_changed_.begin(), vertex_changed_.end(), 0);
      std::fill(propagated_edges_.begin(), propagated_edges_.end(), 0);
    }

    size_t index_in_topo_order_{0}; // The index of the element in the topological_order_ array we were iterating last time
    std::vector<int> vertex_changed_; /// Which vertex index, i.e. observable 
    std::vector<int> propagated_edges_; /// A boolean vector that indicates which edges where propagated (1 if yes, 0 otherwise). This is an edge list as in CSR storage.

  };


  /// The data-flow graph the graph engine operates on. It contains observables as vertices and computations as edges.
  std::shared_ptr<DataFlowGraph> data_flow_graph_;

  PropagationState propagation_state_;
  /// The pre-computed topological order of vertex indices.
  std::vector<size_t> topological_order_;
};


/// Creates the data-flow graph and asserts that it is not edited one obtain() is called
/// A context, creates and manages the data-flow graph. Basis for the class-based API of ICEY.
struct Context : public std::enable_shared_from_this<Context> {
  virtual ~Context() {
    if (on_node_destruction_cb_) on_node_destruction_cb_();
  }

  /// Register callback to be called after all parameters have been attached
  /// For the functional API
  void register_after_parameter_initialization_cb(std::function<void()> cb) {
    after_parameter_initialization_cb_ = cb;
  }

  void register_on_node_destruction_cb(std::function<void()> cb) { on_node_destruction_cb_ = cb; }

  void initialize(ROSAdapter::NodeHandle &node) {
    if (data_flow_graph_->empty()) {
      std::cout << "WARNING: Nothing to spawn, try first to create some signals/states"
                << std::endl;
      return;
    }
    attach_graph_to_node(node);
    was_initialized_ = true;
  }

  void attach_graph_to_node(ROSAdapter::NodeHandle &node) {
    if (icey_debug_print)
      std::cout << "[icey::Context] attach_graph_to_node() start" << std::endl;

    /// First, sort the attachables by priority: first parameters, then publishers, services,
    /// subsribers etc. We could do bin-sort here which runs in linear time, but the standard
    /// library does not have an implementation for it. 
    // TODO honoring this order is not needed in ROS 2 anymore since there is an executor
    const auto attach_priority_order = [this](size_t v1, size_t v2) {
      return data_flow_graph_->graph_[v1]->attach_priority() <
             data_flow_graph_->graph_[v2]->attach_priority();
    };
    /// Now attach everything to the ROS-Node, this creates the parameters, publishers etc.
    /// Now, allow for attaching additional nodes after we got the parameters. After attaching,
    /// parameters immediatelly have their values.
    if (icey_debug_print) std::cout << "[icey::Context] Attaching parameters ..." << std::endl;

    for (auto i_vertex : data_flow_graph_->get_vertices_ordered_by(attach_priority_order)) {
      const auto &attachable = data_flow_graph_->graph_[i_vertex];
      if (attachable->attach_priority() == 1)  /// Stop after attachning all parameters
        break;
      attachable->attach_to_node(node);  /// Attach
    }

    if (icey_debug_print)
      std::cout << "[icey::Context] Attaching parameters finished." << std::endl;
    if (after_parameter_initialization_cb_)
      after_parameter_initialization_cb_();  /// Call user callback. Here, more vertices may be
                                             /// added


    if (icey_debug_print) std::cout << "[icey::Context] Attaching maybe new vertices  ... " << std::endl;
    /// If additional vertices have been added to the DFG, attach it as well

    for (auto i_vertex : data_flow_graph_->get_vertices_ordered_by(attach_priority_order)) {
      const auto &attachable = data_flow_graph_->graph_[i_vertex];
      if (attachable->attach_priority() == 0) continue;
      attachable->attach_to_node(node);  /// Attach
    }
    if (icey_debug_print)
      std::cout << "[icey::Context] Attaching finished, node starts... " << std::endl;

    if(!use_eager_mode_) {
      if (icey_debug_print)
        std::cout << "[icey::Context] Creating graph engine  ... " << std::endl;

      graph_engine_ = std::make_shared<GraphEngine>(data_flow_graph_);
      graph_engine_->analyze_data_flow_graph();
      /// TODO maybe solve better, the nodes only know about the context. The context owns the graph engine, 
      // so when the nodes are notified, they can only notify the context. So we have to direct through the context 
      /// the action to the graph 
      register_source_verices();
      
    }
  }

  /// Registers the run_graph_mode callback for every source vertex in the DFG so that the graph engine is triggered if a subsriber delivers something 
  void register_source_verices() {
    const auto N = num_vertices(data_flow_graph_->graph_);
    for(size_t i_vertex = 0; i_vertex < N; i_vertex++) {
      /// TODO register by flag
      bool is_source = in_degree(i_vertex, data_flow_graph_->graph_) == 0;
      if(is_source) {
        auto source_vertex = data_flow_graph_->graph_[i_vertex];
        source_vertex->register_run_graph_engine([source_vertex](size_t node_id) {
          source_vertex->context.lock()->graph_engine_->run_graph_mode(node_id);
        });
      }
    }
  }

  template <typename ParameterT>
  auto declare_parameter(const std::string &name,
                         const std::optional<ParameterT> &maybe_default_value = std::nullopt,
                         const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
                             rcl_interfaces::msg::ParameterDescriptor(),
                         bool ignore_override = false) {
    return create_observable<ParameterObservable<ParameterT>>(
        name, maybe_default_value, parameter_descriptor, ignore_override);
  }

  template <typename MessageT>
  auto create_subscription(
      const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos(),
      const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions()) {
    auto observable = create_observable<SubscriptionObservable<MessageT>>(name, qos, options);
    return observable;
  }

  auto create_transform_subscription(const std::string &target_frame,
                                     const std::string &source_frame) {
    return create_observable<TransformSubscriptionObservable>(target_frame, source_frame);
  }

  template <class Parent>
  void create_publisher(Parent parent, const std::string &topic_name,
                        const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
    observable_traits<Parent>{};
    using MessageT = typename remove_shared_ptr_t<Parent>::Value;
    auto child = create_observable<PublisherObservable<MessageT>>(topic_name, qos);
    connect_with_identity(child, parent);
  }

  template <class Parent>
  void create_transform_publisher(Parent parent) {
    observable_traits<Parent>{};
    auto child = create_observable<TransformPublisherObservable>();
    connect_with_identity(child, parent);
  }

  auto create_timer(const ROSAdapter::Duration &interval, bool use_wall_time = false,
                    bool is_one_off_timer = false) {
    return create_observable<TimerObservable>(interval, use_wall_time, is_one_off_timer);
  }

  template <typename ServiceT>
  auto create_service(const std::string &service_name,
                      const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    return create_observable<ServiceObservable<ServiceT>>(service_name, qos);
  }

  /// Add a service client
  template <typename ServiceT, typename Parent>
  auto create_client(Parent parent, const std::string &service_name,
                     const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    observable_traits<Parent>{}; // obs_val since the Request must be a shared_ptr
    static_assert(std::is_same_v< typename ServiceT::Request, obs_val<Parent> >, "The parent triggering the service must hold a value of type Request::SharedPtr");
    using Comp = ServiceClientComputation<ServiceT>;
    auto child = create_observable< typename Comp::OutputObservable >();

    Comp comp(service_name);
    /// TODO create_with continuation
    return child;
  }

  /// Synchronizer that given a reference signal at its first argument, ouputs all the other topics
  // interpolated
  // TODO specialize when reference is a tuple of messages. In this case, we compute the arithmetic
  template<class Reference, class... Parents> 
  auto sync_with_reference(Reference reference, Parents ...parents) {
    observable_traits<Reference>{};
    observable_traits<Parents...>{};

    /// TODO consider hana as well auto has_header = hana::is_valid([](auto&& x) -> decltype((void)x.header) { });
    using namespace message_filters::message_traits;
    static_assert(HasHeader<obs_msg<Reference>>::value, "The ROS message type must have a header with the timestamp to be synchronized");

    using Output = std::tuple< obs_val<Reference>, std::optional< obs_val<Parents> >...>;
    auto parents_tuple = std::make_tuple(parents...);
    auto child = create_observable< Observable<Output> >();

    /// TOOD somehow does not work
    //auto all_are_interpolatables = hana::all_of(parents_tuple,  [](auto t) { return hana_is_interpolatable(t); }); 
    //static_assert(all_are_interpolatables, "All inputs must be interpolatable when using the sync_with_reference");

    AnyComputation computation = create_computation<true>(reference, child, 
      [parents_tuple](const obs_val<Reference> &new_value) {  
        auto parent_maybe_values = hana::transform(parents_tuple, [&](auto parent) {
              return parent->get_at_time(rclcpp::Time(new_value->header.stamp));
        });
        return hana::prepend(parent_maybe_values, new_value);
    });

    std::vector<size_t> parent_ids{reference->index.value(), parents->index.value()...};
    /// The interpolatable topics do not update. (they also do not register with the graph) So we create empty computations for them.
    std::vector<AnyComputation> edges_data(parent_ids.size());
    edges_data.at(0) = computation;

    data_flow_graph_->add_edges(child->index.value(), parent_ids, edges_data);
    return child;
  }


  /// Synchronizer that synchronizes non-interpolatable signals by matching the time-stamps
  /// approximately
  template <typename... Parents>
  auto synchronize_approx_time(Parents... parents) {
    observable_traits<Parents...>{};
    //auto parents_tuple = std::make_tuple(parents...);
    static_assert(sizeof...(Parents), "You need to synchronize at least two inputs.");

    uint32_t queue_size = 10;
    auto synchronizer = create_observable<SynchronizerObservable< obs_msg<Parents>...>>(queue_size);

    std::vector<size_t> parent_ids{parents->index.value()...};
    /// Connect the parents to the inputs of the synchronizer
    std::vector<AnyComputation> edges_data =
        create_identity_computation_mimo(std::forward_as_tuple(parents...), synchronizer->inputs());
    /// We add it manually to the graph since we want to represent the topology that is required for
    /// the dependency analysis. The synchronizer is a single node, altough it has N input observables
    data_flow_graph_->add_edges(synchronizer->index.value(), parent_ids, edges_data);
    return synchronizer;
  }

  template <typename... Parents>
  auto synchronize(Parents... parents) {
    observable_traits<Parents...>{};
    static_assert(sizeof...(Parents), "You need to synchronize at least two inputs.");
    auto parents_tuple = std::make_tuple(parents...);
    
    auto interpolatables = hana::remove_if(parents_tuple, [](auto t) { return not  hana_is_interpolatable(t); });
    auto non_interpolatables = hana::remove_if(parents_tuple, [](auto t) { return hana_is_interpolatable(t); });

    constexpr int num_interpolatables = hana::length(interpolatables);
    constexpr int num_non_interpolatables = hana::length(non_interpolatables);
    static_assert(num_interpolatables == 0 || num_non_interpolatables >= 1,
                  "You are trying to synchronize only interpolatable signals. This does not work, "
                  "you need to "
                  "have at least one non-interpolatable signal that is the common time for all the "
                  "interpolatables.");
    // This can only either be one or more than one. Precondition is that we synchronize at least
    // two entities. Given the condition above, the statement follows.
    if constexpr (num_non_interpolatables > 1) {
      /// We need the ApproxTime
      auto approx_time_output = hana::unpack(non_interpolatables, [this](auto ...parents) { return synchronize_approx_time(parents...);});
      if constexpr (num_interpolatables > 1) {
        /// We have interpolatables and non-interpolatables, so we need to use both synchronizers
        return hana::unpack(hana::prepend(interpolatables, approx_time_output), [this](auto ref, auto ...ints) { return sync_with_reference(ref, ints...); });
      } else {
        return approx_time_output;
      }
    } else {
      // Otherwise, we only need a sync with reference
      return hana::unpack(hana::prepend(interpolatables, std::get<0>(non_interpolatables)), 
        [this](auto ref, auto ...ints) { return sync_with_reference(ref, ints...); });
    }
  }

  /// Serialize, pipe arbitrary number of parents of the same type into one. Needed for the
  /// control-flow where the same publisher can be called from multiple callbacks
  template <typename... Parents>
  auto serialize(Parents... parents) {
    observable_traits<Parents...>{};
    // assert_all_observable_values_are_same<Parents...>();
    using Parent = decltype(*std::get<0>(std::forward_as_tuple(parents...)));
    using ParentValue = typename std::remove_reference_t<Parent>::Value;
    /// First, create a new observable
    auto child = create_observable<Observable<ParentValue>>();
    /// Now connect each parent with the child with the identity function
    connect_with_identity(child, parents...);
    return child;
  }

  /// Promise register, if register_both is true, resolve and reject handles are registered, otherwise only the reject handler is registered that resolves with the error
  template <bool resolve, typename Parent, typename F>
  auto done(Parent parent, F &&f) {
    observable_traits<Parent>{};
    /// TODO static_assert here signature for better error messages
    /// Return type depending of if the it is called when the Promise resolves or rejects
    using ReturnType = 
          std::conditional_t<resolve,
          decltype(apply_if_tuple(f, std::declval< obs_val< Parent> >())),
          decltype(apply_if_tuple(f, std::declval< obs_err< Parent> >()))>;

    /// Now we want to call resolve only if it is not none, so strip optional
    using ReturnTypeSome = typename remove_optional<ReturnType>::type;
    if constexpr (std::is_void_v<ReturnType>) {
      /// create a dummy to satisfy the graph
      auto child = create_observable<DevNullObservable>();
      connect<resolve>(child, parent, std::move(f));
      /// return nothing so that no computation can be made based on the result
    } else if(is_variant_v<ReturnType>) { /// But it may be an result type
      /// In this case we want to be able to pass over the same error TODO 
    } else {
      /// The resulting observable always has the same ErrorValue so that it can pass through the error
      using OutputObservable = Observable<ReturnTypeSome, obs_err<Parent> >;
      auto child = create_observable< OutputObservable >();
      connect<resolve>(child, parent, std::move(f));
      return child;
    }
  }

  /// Now we can construct higher-order filters.

  /// For a tuple-observable, get it's N'th element
  template <int index, class Parent>
  auto get_nth(Parent &parent) {
    assert_observable_holds_tuple<Parent>();
    return done<true>(parent, [](const auto &...args) {  /// Need to take variadic because then()
                                                   /// automatically unpacks tuples
      return std::get<index>(
          std::forward_as_tuple(args...));  /// So we need to pack this again in a tuple
    });
  }

  // TODO maybe rem, derive from graph
  bool empty() const { return data_flow_graph_->empty(); }
  void clear() { data_flow_graph_->clear(); }


  bool use_eager_mode_{true};  // Use eager mode or graph mode TODO rem, do the wiring in the executor
  bool was_initialized_{false};

//protected: TODO for testing
  void assert_icey_was_not_initialized() {
    if (was_initialized_)
      throw std::invalid_argument(
          "You are not allowed to add signals after ICEY was initialized. The graph must be "
          "static");
  }

  /// Creates a new observable of type O by passing the args to the constructor. It also adds it as
  /// a vertex to the graph.
  template <class O, typename... Args>
  std::shared_ptr<O> create_observable(Args &&...args) {
    assert_icey_was_not_initialized();
    auto observable = std::make_shared<O>(std::forward<Args>(args)...);
    data_flow_graph_->add_v(observable);  /// Register with graph to obtain a vertex ID
    observable->context = shared_from_this();
    observable->class_name = boost::typeindex::type_id_runtime(*observable).pretty_name();
    return observable;
  }

  
  
  /// Creates a computation object that is stored in the edges of the graph for the graph mode.
  /// If eager mode is enabled, it additionally registers the computation so that it is immediatelly
  /// run
  /// TODO generalize, allow for creating async computes by passing continuation (maybe move this logic into Obs)
  template <bool resolve, class Input, class Output, class F>
  AnyComputation create_computation(Input input, Output output, F &&f) {
    observable_traits<Input, Output>{};

    using Value = obs_val<Input>;
    using ErrorValue = obs_err<Input>;
    using FunctionArgument = std::conditional_t<resolve, Value, ErrorValue >;

    // TODO use std::invoke_result
    using ReturnType = 
          std::conditional_t<resolve,
          decltype(apply_if_tuple(f, std::declval< FunctionArgument >())),
          decltype(apply_if_tuple(f, std::declval< FunctionArgument >()))>;

    /// TODO hof, hana::unpack
    const auto on_new_value = [f = std::move(f)](const FunctionArgument &new_value) -> ReturnType {
      if constexpr (std::is_void_v<ReturnType>) {
        apply_if_tuple(f, new_value);
      } else {
        return apply_if_tuple(f, new_value);
      }
    };

    /// TODO refactor, bind
    auto notify = [output]() { output->_notify(); };
    auto call_and_resolve = [output, f = std::move(on_new_value)](const FunctionArgument &new_value) {
        if constexpr (std::is_void_v<ReturnType>) {
          f(new_value);
        } else {
          ReturnType result = f(new_value);
          output->resolve(result); /// Do not notify, only set (may be none)
        }
    };
    
    auto resolve_if_needed = [input, output, call_and_resolve=std::move(call_and_resolve)]() {
      if constexpr (!resolve) { /// If we .catch() 
        if(input->has_error())  { /// Then only resolve with the error if there is one
          call_and_resolve(input->error());
        } /// Else do nothing
      } else {
        if(input->has_value()) { 
          call_and_resolve(input->value());
        } else if (input->has_error()) {
          output->reject(input->error()); /// Important: do not execute computation, but propagate the error
        }
      }
    };

    AnyComputation computation;
    computation.f = [=]() { resolve_if_needed(); notify(); };//hana::compose(notify, compute, get_input_value);

    if (this->use_eager_mode_) {
      input->_register_handler(computation.f);
    }
  
    return computation;
  }

  /// Makes a connection from parent to child with the given function F and adds it to the data-flow
  /// graph.
  template <bool resolve, class Child, class Parent, class F>
  void connect(Child child, Parent parent, F &&f) {
    // TODO assert parent has index
    std::vector<size_t> parent_ids;
    parent_ids.push_back(parent->index.value());

    std::vector<AnyComputation> edges_data;
    auto computation = create_computation<resolve>(parent, child, std::move(f));
    edges_data.push_back(computation);
    /// Add connections to the graph
    data_flow_graph_->add_edges(child->index.value(), parent_ids, edges_data);
  }

  /// Same but idetity, TODO fix code dup
  template <class Child, typename... Parents>
  void connect_with_identity(Child child, Parents... parents) {
    std::vector<size_t> parent_ids;
    // TODO maybe use std::array to get rid of this check
    if constexpr (sizeof...(Parents) == 1) {
      /// TODO do better ? take_
      const auto &first_parent = std::get<0>(std::forward_as_tuple(parents...));
      parent_ids.push_back(first_parent->index.value());
    } else {
      parent_ids = {parents->index.value()...};
    }

    /// TODO use hana::for_each
    std::vector<AnyComputation> edges_data;
    (
        [&] {
          /// This should be called "parent"
          auto f = create_identity_computation(parents, child);
          edges_data.push_back(f);
        }(),
        ...);
    data_flow_graph_->add_edges(child->index.value(), parent_ids, edges_data);
  }

  /// Create a identity computation between input and output.
  template <class Input, class Output>
  AnyComputation create_identity_computation(Input input, Output output) {
    observable_traits<Input, Output>{};
    /// TODO Assert Input is not a tuple or return tuple if it is
    /// TOOD use hana::id
    /// idendity computation always propagates both resolve and reject
    return create_computation<true>(input, output, [](auto &&x) { return std::move(x); });
  }

  /// Connects each of the N inputs with the corresponding N outputs using identity
  template <class Inputs, class Outputs>
  std::vector<AnyComputation> create_identity_computation_mimo(Inputs inputs, Outputs outputs) {
    static_assert(std::tuple_size_v<Inputs> == std::tuple_size_v<Outputs>,
                  "Inputs and outputs must have the same size");
    // Generate a sequence of indices corresponding to the tuple size
    using Indices = mp::mp_iota_c<std::tuple_size_v<Inputs>>;
    std::vector<AnyComputation> computations;
    // Iterate over the indices
    mp::mp_for_each<Indices>([&](auto I) {
      auto f = create_identity_computation(std::get<I>(inputs), std::get<I>(outputs));
      computations.push_back(f);
    });
    return computations;
  }

  /// The data-flow graph that this context creates. Everyghing is stored and owned by this graph. 
  std::shared_ptr<DataFlowGraph> data_flow_graph_{std::make_shared<DataFlowGraph>()};

  std::shared_ptr<GraphEngine> graph_engine_;
  std::function<void()> after_parameter_initialization_cb_;
  std::function<void()> on_node_destruction_cb_;
};


/// The ROS node, additionally owning the data-flow graph (DFG) that contains the observables.
/// This class is needed to ensure that the context lives for as long as the node lives.
class ROSNodeWithDFG : public ROSAdapter::Node {
public:
  using Base = ROSAdapter::Node;
  using Base::Base;  // Take over all base class constructors

  /// This attaches all the ICEY signals to the node, meaning it creates the subcribes etc. It
  /// initializes everything in a pre-defined order.
  void icey_initialize() { icey().initialize(*this); }

  /// Returns the context that is needed to create ICEY observables
  Context &icey() { return *this->icey_context_; }


  std::shared_ptr<Context> icey_context_{std::make_shared<Context>()};
};

/// Blocking spawn of an existing node.
void spawn(std::shared_ptr<ROSAdapter::Node> node) {
  /// We use single-threaded executor because the MT one can starve due to a bug
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}

void spin_nodes(const std::vector<std::shared_ptr<ROSNodeWithDFG>> &nodes) {
  rclcpp::executors::SingleThreadedExecutor executor;
  /// This is how nodes should be composed according to ROS maintainer wjwwood:
  /// https://robotics.stackexchange.com/a/89767 He references
  /// https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp
  // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), num_threads);
  for (const auto &node : nodes) executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}

/// API aliases
using Node = ROSNodeWithDFG;

}  // namespace icey

#include <icey/functional_api.hpp>
