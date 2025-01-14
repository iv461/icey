#pragma once

#include <fmt/core.h>
#include <fmt/ostream.h>

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
#include <boost/noncopyable.hpp>
#include <boost/type_index.hpp>
#include <icey/bag_of_metaprogramming_tricks.hpp>

namespace mp = boost::mp11;

namespace icey {

bool icey_debug_print = false;
using Time = ROS2Adapter::Time;
class Context;

/// A node in the DFG-graph, corresponds to a node-attachable
class DFGNode {
public:
  /// We bould the node starting from the root, to allow a context-free graph creation, we refer to
  /// the parents
  std::weak_ptr<Context> context;
  std::optional<size_t>
      index;  /// The index of this observable in list of vertices in the data-flow graph. We have
              /// to store it here because of cyclic dependency. None if this observable was not
              /// attached to the graph yet.
  std::string class_name;  // The class name, i.e. the type, for example SubscriberObservable<>
  std::string name;  /// A name to identify it among multiple with the same type, usually the topic
                     /// or service name
};

/// Everything that can be "attached" to a node, publishers, subscribers, clients, TF subscriber
/// etc.
class NodeAttachable : private boost::noncopyable {
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

/// An buffer, storing the last received value
template <typename _Value>
struct Buffer {
  using Value = _Value;
  using MaybeValue = std::optional<Value>;

  virtual bool has_value() const { return value_.has_value(); }
  virtual const Value &value() const { return value_.value(); }

protected:
  /// The last received value.
  MaybeValue value_;
};
/// Needed for storing in the graph as well as to let the compiler check for correct types
struct ObservableBase : public DFGNode, public NodeAttachable {};

/*template <typename... Types>
struct AssertAllAreObservables;

// Specialization for variadic types
template <typename First, typename... Rest>
struct AssertAllAreObservables<First, Rest...> {
    static_assert((std::is_same_v<typename First::Foo, typename Rest::Foo> && ...),
                  "All T::Foo types must be the same");
    // Optional: Add other static assertions here if needed
};
*/

template <typename... Args>
struct observable_traits {
  static_assert(
      (is_shared_ptr<Args> && ...),
      "The arguments must be a shared_ptr< icey::Observable >, but it is not a shared_ptr");
  static_assert((std::is_base_of_v<ObservableBase, remove_shared_ptr_t<Args>> && ...),
                "The arguments must be an icey::Observable");
};

template <class T>
constexpr void assert_is_observable() {
  static_assert(
      is_shared_ptr<T>,
      "The argument must be a shared_ptr< icey::Observable >, but it is not a shared_ptr");
  static_assert(std::is_base_of_v<ObservableBase, remove_shared_ptr_t<T>>,
                "The argument must be an icey::Observable");
}

template <class T>
constexpr void assert_observable_holds_tuple() {
  assert_is_observable<T>();
  static_assert(is_tuple_v<typename remove_shared_ptr_t<T>::Value>,
                "The Observable must hold a tuple as a value for unpacking.");
}

template <typename... Args>
constexpr void assert_are_observables() {
  (assert_is_observable<Args>(), ...);
}

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

/// An observable. Similar to a promise in JS.
/// TODO consider CRTP, would also be beneficial for PIMPL
template <typename _Value>
class Observable : public ObservableBase,
                   public Buffer<_Value>,
                   public std::enable_shared_from_this<Observable<_Value>> {
  friend class Context;  /// To prevent misuse, only the Context is allowed to register on_change
                         /// callbacks
public:
  using Value = _Value;
  using ErrorValue = std::string;
  using Self = Observable<_Value>;

  using OnResolve = std::function<void(const Value &)>;
  using OnReject = std::function<void(const ErrorValue &)>;

  /// Creates a new Observable that changes it's value to y every time the value x of the parent
  /// observable changes, where y = f(x).
  template <typename F>
  auto then(F &&f) {
    return this->context.lock()->then(this->shared_from_this(), f);
  }

  /// Create a ROS publisher for this observable.
  void publish(const std::string &topic_name,
               const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
    return this->context.lock()->create_publisher(this->shared_from_this(), topic_name, qos);
  }

protected:
  /// Register to be notified when smth. changes
  virtual void on_change(OnResolve &&cb) {
    notify_list_.emplace_back(std::move(cb));  /// TODO rename to children ?
  }
  /// set and notify all observers about the new value
  virtual void _set(const Value &new_value) {
    this->value_ = new_value;
    if (icey_debug_print)
      std::cout << "[" + this->class_name + ", " + this->name + "] _set() called, this->value_: "
                << this->value_.has_value() << std::endl;

    for (auto cb : notify_list_) {
      cb(new_value);
    }
  }

  void _reject(const ErrorValue &error) {
    for (auto cb : reject_cbs_) cb(error);
  }

  std::vector<OnResolve> notify_list_;
  std::vector<OnReject> reject_cbs_;
};

/// An observable storing the last received value TODO rem
template <typename _Value>
struct BufferedObservable : public Observable<_Value> {
  using Base = Observable<_Value>;
  using Value = _Value;
  using MaybeValue = std::optional<Value>;
};

struct Nothing {};
/// For fucntions returning void a dummy observable
struct DevNullObservable : public Observable<Nothing> {};

/// An interpolatable observable is one that buffers the incoming values using a circular buffer and
/// allows to query the message at a given point, optionally using interpolation. It is an essential
/// for using lookupTransform with TF. It is used by synchronizers to synchronize a topic exactly at
/// a given time point.
struct InterpolateableObservableBase {};
template <typename _Value>
struct InterpolateableObservable : public InterpolateableObservableBase,
                                   public BufferedObservable<_Value> {
  using Value = _Value;
  using Base = BufferedObservable<_Value>;
  using MaybeValue = typename Base::MaybeValue;
  /// Get the closest measurement to a given time point. Returns nothing if the buffer is empty or
  /// an extrapolation would be required.
  virtual MaybeValue get_at_time(const Time &time_point) const = 0;
};

template <class T>
using is_interpolatable = std::is_base_of<InterpolateableObservableBase, T>;

/// An observable that holds the last received value. Fires initially an event if a default_value is
/// set
template <typename _Value>
class ParameterObservable : public BufferedObservable<_Value> {
public:
  using Value = _Value;
  using Base = BufferedObservable<_Value>;
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
          this->value_ = new_value;  /// Store value
          this->_set(new_value);     /// notify
        },
        parameter_descriptor_, ignore_override_);
    /// Set default value
    if (default_value_) {
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

template <typename _Value>
class SubscriptionObservable : public Observable<_Value> {
  friend class Context;

public:
  using Value = _Value;
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
        name_, [this](std::shared_ptr<Value> new_value) { this->_set(*new_value); }, qos_,
        options_);
  }

  std::string name_;
  ROSAdapter::QoS qos_;
  const rclcpp::SubscriptionOptions options_;
};

/// A subscription for single transforms. It implements InterpolateableObservable but by using
/// lookupTransform, not an own buffer
struct TransformSubscriptionObservable
    : public InterpolateableObservable<geometry_msgs::msg::TransformStamped> {
public:
  TransformSubscriptionObservable(const std::string &target_frame, const std::string &source_frame)
      : target_frame_(target_frame), source_frame_(source_frame) {
    this->attach_priority_ = 3;
    this->name = "source_frame: " + source_frame_ + ", target_frame: " + target_frame_;
  }

  /// TODO override: Is TF buffer empty ? -> Needed to distinguitsh TF errors
  virtual bool has_value() const { return this->value_.has_value(); }

protected:
  MaybeValue get_at_time(const Time &time_point) const override {
    std::optional<geometry_msgs::msg::TransformStamped> tf_msg;
    try {
      // Note that this call does not wait, the transform must already have arrived. This works
      // because get_at_time() is called by the synchronizer
      tf_msg = tf2_listener_->buffer_.lookupTransform(target_frame_, source_frame_, time_point);
    } catch (tf2::TransformException &e) {
    }
    return tf_msg;
  }

  void attach_to_node(ROSAdapter::NodeHandle &node_handle) {
    if (icey_debug_print)
      std::cout << "[TransformSubscriptionObservable] attach_to_node()" << std::endl;
    tf2_listener_ = node_handle.add_tf_subscription(
        target_frame_, source_frame_,
        [this](const geometry_msgs::msg::TransformStamped &new_value) { this->_set(new_value); });
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
      this->_set(ticks_counter_++);
      if (is_one_off_timer_) ros_timer_->cancel();
    });
  }

  rclcpp::TimerBase::SharedPtr ros_timer_;
  ROSAdapter::Duration interval_;
  bool use_wall_time_{false};
  bool is_one_off_timer_{false};
  Value ticks_counter_{0};
};

/// A publishabe state, read-only
template <typename _Value>
class PublisherObservable : public Observable<_Value> {
  friend class Context;

public:
  using Value = _Value;
  using Base = Observable<_Value>;
  static_assert(rclcpp::is_ros_compatible_type<Value>::value,
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
    auto publish = node_handle.add_publication<Value>(topic_name_, qos_);
    this->on_change([publish](const auto &new_value) { publish(new_value); });
  }

  virtual void on_change(typename Base::OnResolve &&cb) override {
    if (icey_debug_print)
      std::cout << "[PublisherObservable on " + topic_name_ + "] on_change()  " << std::endl;
    Base::on_change(std::move(cb));
  }

  virtual void _set(const Value &new_value) {  /// TODO DBG
    if (icey_debug_print)
      std::cout << "[PublisherObservable] _set() called, notify_list_.size(): "
                << this->notify_list_.size() << std::endl;
    Base::_set(new_value);
  }
  std::string topic_name_;
  ROSAdapter::QoS qos_{ROS2Adapter::DefaultQos()};
};

/// A publishabe state, can be written. Needed where we need to publish by reacting on external
/// events
// that are not in control of ROS. This is needed for hardware drivers for example
template <typename _Value>
class WritablePublisherObservable : public PublisherObservable<_Value> {
public:
  using Value = _Value;
  void publish(const Value &message) { this->_set(message); }
};

struct AnyComputation {
  std::function<void()> f;
};

/// TODO maybe for easier debugging ?
template <class _Input, class _Output>
struct Computation : public AnyComputation {
  std::shared_ptr<Observable<_Input>> input;
  std::shared_ptr<Observable<_Output>> output;
};

/// Wrap the message_filters official ROS package. In the following, "MFL" refers to the
/// message_filters package. An adapter, adapting the message_filters::SimpleFilter to our
/// Observable (two different implementations of almost the same concept). Does nothing else than
/// what message_filters::Subscriber does:
/// https://github.com/ros2/message_filters/blob/humble/include/message_filters/subscriber.h#L349
// We neeed the base to be able to recognize interpolatable nodes for example
template <typename _Value, class _Base = Observable<_Value>>
struct SimpleFilterAdapter : public _Base, public message_filters::SimpleFilter<_Value> {
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
    this->_set(std::forward_as_tuple(messages...));
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
    this->on_change([publish](const auto &new_value) { publish(new_value); });
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
          this->_set(std::make_pair(request, response));
        },
        qos_);
  }
  std::string service_name_;
  ROSAdapter::QoS qos_;
};

template <typename _ServiceT>
struct ClientObservable : public Observable<typename _ServiceT::Response> {
  using Request = typename _ServiceT::Request;
  using Response = typename _ServiceT::Request;
  using Value = Response;
  /// The type of the required input
  using Parent = std::shared_ptr<Observable<std::shared_ptr<Request>>>;

  ClientObservable(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS())
      : service_name_(service_name), qos_(qos) {
    this->attach_priority_ = 4;
    this->name = service_name_;
  }

protected:
  void attach_to_node(ROSAdapter::NodeHandle &node_handle) {
    if (icey_debug_print) std::cout << "[ClientObservable] attach_to_node()" << std::endl;
    auto client = node_handle.add_client<_ServiceT>(service_name_, qos_);
    this->on_change([this, client](std::shared_ptr<Request> request) {
      client->async_send_request(request,
                                 [this](auto response_futur) { this->_set(response_futur.get()); });
    });
  }

  std::string service_name_;
  ROSAdapter::QoS qos_;
};

/// A graph, owning the observables TODO decide on base-class, since the graph only needs to own the
/// data, we could even use std::any
struct DataFlowGraph {
  /// TODO store the computation
  using EdgeData = AnyComputation;
  using VertexData = std::shared_ptr<ObservableBase>;
  /// vecS needed so that the vertex id's are consecutive integers
  using Graph =
      boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexData, EdgeData>;

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

  // Returns an index remapping for the indices that sort them by a predicate. Note that you should
  // never mutate the order of the vertices, meaning sorting them directly. This is because the
  // vertex data (the Observables) references this index.
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
  void compute_strong_components() {
    // Compute strongly connected components
    auto &g = graph_;
    std::vector<int> component(num_vertices(g));  // Component indices
    int num_scc = boost::strong_components(g, &component[0]);

    // Map each component to its vertices
    std::vector<std::set<int>> components(num_scc);
    for (size_t v = 0; v < component.size(); ++v) {
      components[component[v]].insert(v);
    }

    // Identify and print vertices in cycles
    std::cout << "Vertices in cycles (including self-loops):" << std::endl;
    for (const auto &comp : components) {
      if (comp.size() > 1 ||
          (comp.size() == 1 && boost::edge(*comp.begin(), *comp.begin(), g).second)) {
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

/// A context, creates and manages the data-flow graph. Basis for the class-based API of ICEY.
struct Context : public std::enable_shared_from_this<Context> {
  virtual ~Context() {
    if (on_node_destruction_cb_) on_node_destruction_cb_();
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
  void create_publisher(Parent &&parent, const std::string &topic_name,
                        const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
    observable_traits<Parent>{};
    using MessageT = typename remove_shared_ptr_t<Parent>::Value;
    auto child = create_observable<PublisherObservable<MessageT>>(topic_name, qos);
    connect_with_identity(child, parent);
  }

  template <typename MessageT>
  auto create_writable_publisher(const std::string &topic_name,
                                 const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQos()) {
    return create_observable<WritablePublisherObservable<MessageT>>(topic_name, qos);
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
  template <class ServiceT, class Parent>
  auto create_client(Parent parent, const std::string &service_name,
                     const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    observable_traits<Parent>{};
    return create_observable<ClientObservable<ServiceT>>(service_name, qos);
  }

  /*
  /// Synchronizer that given a reference signal at its first argument, ouputs all the other topics
  interpolated
  // TODO specialize when reference is a tuple of messages. In this case, we compute the arithmetic
  mean of all the header stamps. template<class Reference, class... Parents> auto
  sync_with_reference(Reference && reference, Parents && ... parents) {
      /// Remote shared_ptr TODO write proper type trait for this
      using ReturnType = std::tuple<Reference, typename
  std::remove_reference_t<decltype(*parents)>::Value...>; auto resulting_observable =
  std::make_shared< Observable<ReturnType> >();  /// A result is rhs, i.e. read-only
      data_flow_graph_.add_vertex_with_parents(resulting_observable, {reference,
  parents->index.value()...}); /// And add to the graph const auto f_continued =
  [resulting_observable, reference](auto&&... parents) { auto all_argunments_arrived =
  (parents->value_ && ... && true); if(all_argunments_arrived)
              resulting_observable->_set(std::make_tuple(reference, parents->value_.value()...));
      };
      /// This should be called "parent", "parent->on_change()". This is essentially a for-loop over
  all parents, but C++ cannot figure out how to create a readable syntax, instead we got these fold
  expressions reference->on_change(std::bind(f_continued, std::forward<Parents>(parents)...));
      return resulting_observable; /// Return the underlying pointer. We can do this, since
  internally everything is stores reference-counted.
  }
  */

  /// Synchronizer that synchronizes non-interpolatable signals by matching the time-stamps
  /// approximately
  template <typename... Parents>
  auto synchronize_approx_time(Parents... parents) {
    observable_traits<Parents...>{};
    using inputs = std::tuple<remove_shared_ptr_t<Parents>...>;
    static_assert(mp::mp_size<inputs>::value >= 2, "You need to synchronize at least two inputs.");

    uint32_t queue_size = 10;
    auto synchronizer =
        create_observable<SynchronizerObservable<typename remove_shared_ptr_t<Parents>::Value...>>(
            queue_size);

    auto parent_ids = std::vector<size_t>(parents->index.value()...);
    /// Connect the parents to the inputs of the synchronizer
    std::vector<AnyComputation> edges_data =
        create_identity_computation_mimo(std::forward_as_tuple(parents...), synchronizer->inputs());
    // std::apply([&](const auto &... child) {( icey_connect(parents, child), ...);},
    // sync->inputs());
    /// Add vertex data, parent id's and edges data
    /// We add it manually to the graph since we want to represent the topology that is required for
    /// the
    // dependency analysis. The synchronizer is a single node, altough it has N input observables
    data_flow_graph_.add_edges(synchronizer->index.value(), parent_ids, edges_data);
    return synchronizer;
  }

  template <typename... Parents>
  auto synchronize(Parents... parents) {
    observable_traits<Parents...>{};
    using inputs = std::tuple<remove_shared_ptr_t<Parents>...>;
    static_assert(mp::mp_size<inputs>::value >= 2, "You need to synchronize at least two inputs.");
    using partitioned = mp::mp_partition<inputs, is_interpolatable>;
    using interpolatables = mp::mp_first<partitioned>;
    using non_interpolatables = mp::mp_second<partitioned>;
    constexpr int num_interpolatables = mp::mp_size<interpolatables>::value;
    constexpr int num_non_interpolatables = mp::mp_size<non_interpolatables>::value;
    static_assert(num_interpolatables == 0 || num_non_interpolatables >= 1,
                  "You are trying to synchronize only interpolatable signals. This does not work, "
                  "you need to "
                  "have at least one non-interpolatable signal that is the common time for all the "
                  "interpolatables.");

    // auto non_interpolatables_v = filter_tuple(parents, not is_interpolatable);
    /// auto interpolatables_v = filter_tuple(parents, is_interpolatable);
    // This can only either be one or more than one. Precondition is that we synchronize at least
    // two entities. Given the condition above, the statement follows.
    if constexpr (num_non_interpolatables > 1) {
      /// We need the ApproxTime
      // auto approx_time_output = synchronize_approx_time(non_interpolatables_v);
      if constexpr (num_interpolatables > 1) {
        /// We have interpolatables and non-interpolatables, so we need to use both synchronizers
        // return sync_with_reference(approx_time_output, interpolatables_v...);
      }
    } else {
      // Otherwise, we only need a sync with reference
      /// return sync_with_reference(std::get<0>(non_interpolatables_v), interpolatables_v...);
    }
    return synchronize_approx_time(parents...);
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

  template <typename Parent, typename F>
  auto then(Parent parent, F &&f) {
    observable_traits<Parent>{};
    /// TODO static_assert here signature for better error messages
    using ReturnType =
        decltype(apply_if_tuple(f, std::declval<typename remove_shared_ptr_t<Parent>::Value>()));
    using ObsValue = typename remove_optional<ReturnType>::type;
    if constexpr (std::is_void_v<ReturnType>) {
      /// create a dummy to satisfy the graph
      auto child = create_observable<DevNullObservable>();
      connect(child, parent, std::move(f));
      /// return nothing so that no computation can be made based on the result
    } else {
      auto child = create_observable<Observable<ObsValue>>();
      connect(child, parent, std::move(f));
      return child;
    }
  }
  /// Now we can construct higher-order filters.

  /// For a tuple-observable, get it's N'th element
  template <int index, class Parent>
  auto get_nth(Parent &parent) {
    assert_observable_holds_tuple<Parent>();
    return then(parent, [](const auto &...args) {  /// Need to take variadic because then()
                                                   /// automatically unpacks tuples
      return std::get<index>(
          std::forward_as_tuple(args...));  /// So we need to pack this again in a tuple
    });
  }

  // TODO maybe rem, derive from graph
  bool empty() const { return data_flow_graph_.empty(); }
  void clear() { data_flow_graph_.clear(); }

  /// Register callback to be called after all parameters have been attached
  /// For the functional API
  void register_after_parameter_initialization_cb(std::function<void()> cb) {
    after_parameter_initialization_cb_ = cb;
  }

  void register_on_node_destruction_cb(std::function<void()> cb) { on_node_destruction_cb_ = cb; }

  /// This attaches all the ICEY signals to the node, meaning it creates the subcribes etc. It
  /// initializes everything in a pre-defined order.
  void initialize(ROSAdapter::NodeHandle &node) {
    if (empty()) {
      std::cout << "WARNING: Nothing to spawn, try first to create some signals/states"
                << std::endl;
      return;
    }
    attach_everything_to_node(node);
    was_initialized_ = true;
  }

  bool use_eager_mode_{true};  // Use eager mode or graph mode

protected:
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
    data_flow_graph_.add_v(observable);  /// Register with graph to obtain a vertex ID
    observable->context = shared_from_this();
    observable->class_name = boost::typeindex::type_id_runtime(*observable).pretty_name();
    return observable;
  }

  /// Creates a computation object that is stored in the edges of the graph for the graph mode.
  /// If eager mode is enabled, it additionaly registers the computation so that it is immediatelly
  /// run
  template <class Input, class Output, class F>
  AnyComputation create_computation(Input input, Output output, F &&f) {
    observable_traits<Input, Output>{};
    using ReturnType =
        decltype(apply_if_tuple(f, std::declval<typename remove_shared_ptr_t<Input>::Value>()));
    AnyComputation computation;

    const auto on_new_value = [f = std::move(f)](const auto &new_value) {
      return apply_if_tuple(f, new_value);
    };
    const auto on_new_value_void = [f = std::move(f)](const auto &new_value) {
      apply_if_tuple(f, new_value);
    };

    if constexpr (std::is_void_v<ReturnType>) {
      /// The computation has no arguments, it obtains the buffered value from the input and calls
      /// the function
      computation.f = [input, f = std::move(on_new_value_void)]() {
        f(input->value());
      };

      if (this->use_eager_mode_) {
        input->on_change(
            [f = std::move(on_new_value_void)](const auto &new_value) { f(new_value); });
      }
    } else {
      computation.f = [input, output, f = std::move(on_new_value)]() {
        output->value_ = f(input->value_.value());  /// Do not notify, only set
      };
      if (this->use_eager_mode_) {
        input->on_change([output, f = std::move(on_new_value)](const auto &new_value) {
          auto ret = f(new_value);
          if constexpr (is_optional_v<ReturnType>) {
            if (ret) {
              output->_set(*ret);
            }
          } else {
            output->_set(ret);
          }
        });
      }
    }

    // And register the callback for running the graph_loop if not already done (we have to call
    // smth when the subscriber callback get's called, right)
    if_needed_register_run_graph_mode(input);

    return computation;
  }

  /// Makes a connection from parent to child with the given function F and adds it to the data-flow
  /// graph.
  template <class Child, class Parent, class F>
  void connect(Child child, Parent parent, F &&f) {
    // TODO assert parent has index
    std::vector<size_t> parent_ids;
    parent_ids.push_back(parent->index.value());

    std::vector<AnyComputation> edges_data;
    auto computation = create_computation(parent, child, std::move(f));
    edges_data.push_back(computation);
    /// Add connections to the graph
    data_flow_graph_.add_edges(child->index.value(), parent_ids, edges_data);
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

    std::vector<AnyComputation> edges_data;
    (
        [&] {
          /// This should be called "parent"
          auto f = create_identity_computation(parents, child);
          edges_data.push_back(f);
        }(),
        ...);
    data_flow_graph_.add_edges(child->index.value(), parent_ids, edges_data);
  }

  /// Create a identity computation between input and output.
  template <class Input, class Output>
  AnyComputation create_identity_computation(Input input, Output output) {
    observable_traits<Input, Output>{};
    /// TODO Assert Input is not a tuple or return tuple if it is
    return create_computation(input, output, [](auto &&x) { return std::move(x); });
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

  void attach_everything_to_node(ROSAdapter::NodeHandle &node) {
    if (icey_debug_print)
      std::cout << "[icey::Context] attach_everything_to_node() start" << std::endl;

    /// First, sort the attachables by priority: first parameters, then publishers, services,
    /// subsribers etc. We could do bin-sort here which runs in linear time, but the standard
    /// library does not have an implementation for it.
    const auto attach_priority_order = [this](size_t v1, size_t v2) {
      return data_flow_graph_.graph_[v1]->attach_priority() <
             data_flow_graph_.graph_[v2]->attach_priority();
    };
    /// Now attach everything to the ROS-Node, this creates the parameters, publishers etc.
    /// Now, allow for attaching additional nodes after we got the parameters. After attaching,
    /// parameters immediatelly have their values.
    if (icey_debug_print) std::cout << "[icey::Context] Attaching parameters ..." << std::endl;

    for (auto i_vertex : data_flow_graph_.get_vertices_ordered_by(attach_priority_order)) {
      const auto &attachable = data_flow_graph_.graph_[i_vertex];
      if (attachable->attach_priority() == 1)  /// Stop after attachning all parameters
        break;
      attachable->attach_to_node(node);  /// Attach
    }

    if (icey_debug_print)
      std::cout << "[icey::Context] Attaching parameters finished." << std::endl;
    if (after_parameter_initialization_cb_)
      after_parameter_initialization_cb_();  /// Call user callback. Here, more vertices may be
                                             /// added

    if (icey_debug_print)
      std::cout << "[icey::Context] Analyzing data flow graph ... " << std::endl;
    analyze_dfg();  /// Now analyze the graph before attaching everything to detect cycles as soon
                    /// as possible, especially before the node is started and everything crashes

    if (icey_debug_print) std::cout << "[icey::Context] Attaching everything ... " << std::endl;
    /// If additional vertices have been added to the DFG, attach it as well

    for (auto i_vertex : data_flow_graph_.get_vertices_ordered_by(attach_priority_order)) {
      const auto &attachable = data_flow_graph_.graph_[i_vertex];
      if (attachable->attach_priority() == 0) continue;
      attachable->attach_to_node(node);  /// Attach
    }
    if (icey_debug_print)
      std::cout << "[icey::Context] Attaching finished, node starts... " << std::endl;
  }

  /// TODO
  void analyze_dfg() {
    auto maybe_topological_order = data_flow_graph_.try_topological_sorting();
    if (!maybe_topological_order) {
      data_flow_graph_.compute_strong_components();
      throw std::runtime_error(
          "The graph created is not a directed acyclic graph (DAG), meaning it has loops. Please "
          "review the created vertices and remove the loops.");
    }
    topological_order_ = maybe_topological_order.value();

    data_flow_graph_.print();

    /// analyze_dependencies() -> computes the number of threads needed
  }

  /// Register that on change, this observable will run the graph mode of the context
  template <class Parent>
  void if_needed_register_run_graph_mode(Parent parent) {
    if (this->use_eager_mode_) {
      return;
    }
    /// In graph mode, no notify callbacks are registered except a single "run graph mode callback".
    /// So we just check whether the notify list empty
    if (parent->notify_list_.empty()) {
      parent->on_change([parent](const auto &) {
        parent->context.lock()->run_graph_mode(
            parent->index.value());  // context is actually this, but we avoid capturing this in
                                     // objects that we do not directly own
      });
    }
  }

  /// Executes the graph mode. This function gets called if some of the inputs change and propagates
  /// the result.
  /// TODO deal with param stage, parameters fire always immediatelly after attaching.
  /// but at this point, the graph is not there yet. Maybe we can cache the value_changed array and
  /// process it later ?
  void run_graph_mode(size_t signaling_vertex_id) {
    if (icey_debug_print)
      std::cout << "[icey::Context] Got event from vertex " << signaling_vertex_id
                << ", graph execution starts ..." << std::endl;

    auto &graph_ = data_flow_graph_.graph_;
    /// We traverse the vertices in the topological order and check which value changed.
    /// If a value changed, then for all its outgoing edges the computation is executed and these
    /// vertices are marked as changed.
    std::vector<int> value_changed(num_vertices(graph_), 0);
    value_changed.at(signaling_vertex_id) = 1;
    for (auto v : topological_order_) {
      if (!value_changed.at(v)) {
        if (icey_debug_print)
          std::cout << "Vertex " << v << " did not change, skipping ..." << std::endl;
        continue;
      }
      if (icey_debug_print) std::cout << "Vertex " << v << " changed, propagating ..." << std::endl;

      for (auto edge : boost::make_iterator_range(out_edges(v, graph_))) {
        auto target_vertex = target(edge, graph_);  // Get the target vertex

        value_changed.at(target_vertex) = 1;
        if (icey_debug_print)
          std::cout << "Executing edge to  " << target_vertex << " ..." << std::endl;
        auto &edge_data = graph_[edge];  // Access edge data
        edge_data.f();
        /// TODO call notify on leaves, we need to publish ..
      }
    }
    if (icey_debug_print) std::cout << "Propagation finished. " << std::endl;
  }

  DataFlowGraph data_flow_graph_;
  std::vector<size_t> topological_order_;

  /// Indicates whether initialize() was called. Used to ensure the graph is static,
  /// i.e. no items are added after initially initilizing.
  bool was_initialized_{false};

  std::function<void()> after_parameter_initialization_cb_;
  std::function<void()> on_node_destruction_cb_;
};

struct GraphEngine {};

/// The ROS node, additionally owning the data-flow graph (DFG) that contains the observables.
/// This class is needed to ensure that the context lives for as long as the node lives.
class ROSNodeWithDFG : public ROSAdapter::Node {
public:
  using Base = ROSAdapter::Node;
  using Base::Base;  // Take over all base class constructors
  void icey_initialize() { icey().initialize(*this); }
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
