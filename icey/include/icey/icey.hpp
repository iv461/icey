#pragma once
#include <any>
#include <boost/noncopyable.hpp>
#include <boost/hana.hpp>
#include <boost/hana/ext/std/tuple.hpp>  /// Needed so that we do not need the custom hana tuples everywhere: https://stackoverflow.com/a/34318002
#include <boost/type_index.hpp>
#include <coroutine>
#include <functional>
#include <icey/impl/field_reflection.hpp>
#include <icey/impl/stream.hpp>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <tuple>
#include <unordered_map>

/// Support for lifecycle nodes:
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

/// TF2 support:
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
// Message filters library: (.h so that this works with humble as well)
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace icey {

template <class T>
struct t_is_shared_ptr : std::false_type {};

template <class T>
struct t_is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template <class T>
constexpr bool is_shared_ptr = t_is_shared_ptr<T>::value;

namespace hana = boost::hana;
inline bool icey_debug_print = false;
inline bool icey_coro_debug_print = false;
using Clock = std::chrono::system_clock;
using Time = std::chrono::time_point<Clock>;
using Duration = Clock::duration;

/// A helper to abstract regular rclrpp::Nodes and LifecycleNodes.
/// Similar to the NodeInterfaces class: https://github.com/ros2/rclcpp/pull/2041
/// which doesn't look like it's going to come for Humble:
/// https://github.com/ros2/rclcpp/issues/2309
struct NodeInterfaces {
  template <class _Node>
  explicit NodeInterfaces(_Node *node)
      : node_base_(node->get_node_base_interface()),
        node_graph_(node->get_node_graph_interface()),
        node_clock_(node->get_node_clock_interface()),
        node_logging_(node->get_node_logging_interface()),
        node_timers_(node->get_node_timers_interface()),
        node_topics_(node->get_node_topics_interface()),
        node_services_(node->get_node_services_interface()),
        node_parameters_(node->get_node_parameters_interface()),
        node_time_source_(node->get_node_time_source_interface()) {
    if constexpr (std::is_base_of_v<rclcpp_lifecycle::LifecycleNode, _Node>)
      maybe_lifecycle_node = node;
    else if constexpr (std::is_base_of_v<rclcpp::Node, _Node>)
      maybe_regular_node = node;
    else
      static_assert(std::is_array_v<int>,
                    "NodeInterfaces must be constructed either from a rclcpp::Node or a "
                    "rclcpp_lifecycle::LifecycleNode");
  }
  /// This getter is needed for ParameterEventHandler. Other functions likely require this interface
  /// too.
  auto get_node_base_interface() const { return node_base_; }
  auto get_node_graph_interface() const { return node_graph_; }
  auto get_node_clock_interface() const { return node_clock_; }
  auto get_node_logging_interface() const { return node_logging_; }
  auto get_node_timers_interface() const { return node_timers_; }
  auto get_node_topics_interface() const { return node_topics_; }
  auto get_node_services_interface() const { return node_services_; }
  auto get_node_parameters_interface() const { return node_parameters_; }
  auto get_node_time_source_interface() const { return node_time_source_; }
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr node_time_source_;

  /// This is set to either of the two, depending on which node we got.
  /// It has to be a raw pointer since this nodeInterface is needed during node construction
  rclcpp::Node *maybe_regular_node{nullptr};
  rclcpp_lifecycle::LifecycleNode *maybe_lifecycle_node{nullptr};
};

/// A weak pointer that supports operator->.
template<class T>
struct Weak {
  Weak() = default;
  Weak(std::shared_ptr<T> p) : p_(p) {}
  T * operator->() const { 
    if(!p_.lock()) throw std::bad_weak_ptr();
      return p_.lock().get(); 
  }
  T * get() const { 
    if(!p_.lock()) throw std::bad_weak_ptr();
      return p_.lock().get(); 
  }

  std::weak_ptr<T> p_;
};

/// A transform listener that allows to subscribe on a single transform between two coordinate
/// systems. It is implemented similarly to the tf2_ros::TransformListener.
/// Every time a new message is receved on /tf, it checks whether a relevant transforms (i.e. ones
/// we subscribed) was received. It is therefore an asynchronous interface to TF, similar to the
/// tf2_ros::AsynchBuffer. But the key difference is that tf2_ros::AsynchBuffer can only deliver the
/// transform once, it is therefore a
/// [promise](https://github.com/ros2/geometry2/blob/rolling/tf2_ros/src/buffer.cpp#L179), not a
/// stream. We want however to receive a continous stream of transforms, like a subscriber. This
/// class is used to implement the TransformSubscriptionStream.
struct TFListener {
  using TransformMsg = geometry_msgs::msg::TransformStamped;
  using OnTransform = std::function<void(const TransformMsg &)>;
  using OnError = std::function<void(const tf2::TransformException &)>;
  using GetFrame = std::function<std::string()>;

  explicit TFListener(const NodeInterfaces &node) : node_(node) { init(); }

  /// Add notification for a single transform.
  void add_subscription(const GetFrame &target_frame, const GetFrame &source_frame,
                        const OnTransform &on_transform, const OnError &on_error) {
    subscribed_transforms_.emplace_back(target_frame, source_frame, on_transform, on_error);
  }

  const NodeInterfaces &node_; /// Hold weak reference to bevause the Node owns the NodeInterfaces as well, so we avoid circular reference
  /// We take a tf2_ros::Buffer instead of a tf2::BufferImpl only to be able to use ROS-time API
  /// (internally TF2 has it's own timestamps...), not because we need to wait on anything (that's
  /// what tf2_ros::Buffer does in addition to tf2::BufferImpl).
  std::shared_ptr<tf2_ros::Buffer> buffer_;

private:
  using TransformsMsg = tf2_msgs::msg::TFMessage::ConstSharedPtr;

  struct TFSubscriptionInfo {
    TFSubscriptionInfo(const GetFrame &target_frame, const GetFrame &source_frame,
                       OnTransform on_transform, OnError on_error)
        : target_frame(target_frame),
          source_frame(source_frame),
          on_transform(on_transform),
          on_error(on_error) {}
    GetFrame target_frame;
    GetFrame source_frame;
    std::optional<TransformMsg> last_received_transform;
    OnTransform on_transform;
    OnError on_error;
  };

  void init() {
    init_tf_buffer();
    const rclcpp::QoS qos = tf2_ros::DynamicListenerQoS();
    const rclcpp::QoS &static_qos = tf2_ros::StaticListenerQoS();
    auto topics_if = node_.node_topics_;
    message_subscription_tf_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        topics_if, "/tf", qos, [this](TransformsMsg msg) { on_tf_message(msg, false); });
    message_subscription_tf_static_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        topics_if, "/tf_static", static_qos,
        [this](TransformsMsg msg) { on_tf_message(msg, true); });
  }

  void init_tf_buffer() {
    /// This code is a bit tricky. It's about asynchronous programming essentially. The official
    /// example is rather incomplete
    /// ([official](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html))
    /// . I'm following this example instead:
    /// https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/src/imu_transformer.cpp#L16
    /// See also the follwing discussions:
    /// https://answers.ros.org/question/372608/?sort=votes
    /// https://github.com/ros-navigation/navigation2/issues/1182
    /// https://github.com/ros2/geometry2/issues/446
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_.node_clock_->get_clock());
    auto timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(node_.node_base_, node_.node_timers_);
    buffer_->setCreateTimerInterface(timer_interface);
  }

  /// Store the received transforms in the buffer.
  void store_in_buffer(const tf2_msgs::msg::TFMessage &msg_in, bool is_static) {
    std::string authority = "Authority undetectable";
    for (const auto &transform : msg_in.transforms) {
      try {
        buffer_->setTransform(transform, authority, is_static);
      } catch (const tf2::TransformException &ex) {
        std::string temp = ex.what();
        RCLCPP_ERROR(node_.node_logging_->get_logger(),
                     "Failure to set received transform from %s to %s with error: %s\n",
                     transform.child_frame_id.c_str(), transform.header.frame_id.c_str(),
                     temp.c_str());
      }
    }
  }

  /// This simply looks up the transform in the buffer at the latest stamp and checks if it
  /// changed with respect to the previously received one. If the transform has changed, we know
  /// we have to notify.
  void maybe_notify(TFSubscriptionInfo &info) {
    try {
      /// Note that this does not wait/thread-sleep etc. This is simply a lookup in a
      /// std::vector/tree.
      geometry_msgs::msg::TransformStamped tf_msg =
          buffer_->lookupTransform(info.target_frame(), info.source_frame(), tf2::TimePointZero);
      if (!info.last_received_transform || tf_msg != *info.last_received_transform) {
        info.last_received_transform = tf_msg;
        info.on_transform(tf_msg);
      }
    } catch (const tf2::TransformException &e) {
      info.on_error(e);
    }
  }

  void notify_if_any_relevant_transform_was_received() {
    for (auto &tf_info : subscribed_transforms_) {
      maybe_notify(tf_info);
    }
  }

  void on_tf_message(const TransformsMsg &msg, bool is_static) {
    store_in_buffer(*msg, is_static);
    notify_if_any_relevant_transform_was_received();
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_;
  std::vector<TFSubscriptionInfo> subscribed_transforms_;
};

/// A node interface that does the same as a rclcpp::Node (of lifecycle node), but additionally
/// implements holding the shared pointers to subscribers/timers/publishers etc., i.e. provides
/// bookkeeping, so that you do not have to do it.
class NodeBookkeeping : public NodeInterfaces {
public:
  using MaybeError = std::optional<std::string>;
  /// The parameter validation function that allows the parameter update if no error message is
  /// returned and put_errors the parameter update with the error message otherwise.
  using FValidate = std::function<MaybeError(const rclcpp::Parameter &)>;
  template<class V>
  using GetValue = std::function<V()>;

  explicit NodeBookkeeping(const NodeInterfaces &node_interfaces) : NodeInterfaces(node_interfaces) {}

  NodeBookkeeping(const NodeBookkeeping &) = delete;
  NodeBookkeeping &operator=(const NodeBookkeeping &) = delete;
  NodeBookkeeping(NodeBookkeeping &&) = delete;
  NodeBookkeeping &operator=(NodeBookkeeping && ) = delete;
  
  /// All the streams that were created are owned by the Context.
  std::vector<std::shared_ptr<impl::StreamImplBase>> stream_impls_;

  /// Adds a stream impl to the list so that it does not go out of scope.
  template<class StreamImpl>
  Weak<StreamImpl> create_stream_impl() { 
    auto impl = impl::create_stream<StreamImpl>();
    stream_impls_.push_back(impl);
    return impl;
  }
  
  /// Declares a parameter and registers a validator callback and a callback that will get called
  /// when the parameters updates.
  template <class ParameterT, class CallbackT>
  auto add_parameter(const std::string &name, const ParameterT &default_value,
                     CallbackT &&update_callback,
                     const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor = {},
                     FValidate f_validate = {}, bool ignore_override = false) {
    parameter_validators_.emplace(name, f_validate);
    add_parameter_validator_if_needed();
    auto param =
        node_parameters_->declare_parameter(name, rclcpp::ParameterValue(default_value), parameter_descriptor, ignore_override);
    auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(static_cast<NodeInterfaces&>(*this));
    auto cb_handle =
        param_subscriber->add_parameter_callback(name, std::forward<CallbackT>(update_callback));
    parameters_.emplace(name, std::make_pair(param_subscriber, cb_handle));
    return param;
  }
  template <class Msg, class F>
  void add_subscription(const std::string &topic, F &&cb, const rclcpp::QoS &qos,
                        const rclcpp::SubscriptionOptions &options) {
    subscribers_[topic] =
        rclcpp::create_subscription<Msg>(node_topics_, topic, qos, cb, options);
  }

  template <class Msg>
  auto add_publisher(const std::string &topic, const rclcpp::QoS &qos,
      const rclcpp::PublisherOptions publisher_options) {
    return rclcpp::create_publisher<Msg>(node_topics_, topic, qos, publisher_options);
  }

  template <class CallbackT>
  auto add_timer(const Duration &time_interval, CallbackT &&callback,
                 rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    /// We have not no normal timer in Humble, this is why only wall_timer is supported
    return rclcpp::create_wall_timer(time_interval, std::forward<CallbackT>(callback), group,
                                  node_base_.get(), node_timers_.get());
  }

  template <class ServiceT, class CallbackT>
  auto add_service(const std::string &service_name, CallbackT &&callback,
                   const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                   rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    return rclcpp::create_service<ServiceT>(node_base_, node_services_,
                                                    service_name, std::forward<CallbackT>(callback),
                                                    qos.get_rmw_qos_profile(), group);
  }

  template <class Service>
  auto add_client(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                  rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    return
        rclcpp::create_client<Service>(node_base_, node_graph_, node_services_,
                                       service_name, qos.get_rmw_qos_profile(), group);
  }

  /// Subscribe to a transform on tf between two frames
  template <class OnTransform, class OnError>
  auto add_tf_subscription(GetValue<std::string> target_frame, GetValue<std::string> source_frame,
                           OnTransform &&on_transform, OnError &&on_error) {
    add_tf_listener_if_needed();
    tf2_listener_->add_subscription(target_frame, source_frame, on_transform, on_error);
    return tf2_listener_;
  }

  auto get_tf_buffer() {
    add_tf_listener_if_needed();
    return tf2_listener_->buffer_;
  }

  auto add_tf_broadcaster_if_needed() {
    if (!tf_broadcaster_)
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(static_cast<NodeInterfaces&>(*this));
    return tf_broadcaster_;
  }

private:
  /// The internal rclcpp::Node does not consistently call the free function (i.e. rclcpp::create_*)
  /// but instead prepends the sub_namespace. (on humble) This seems to me like a patch, so we have
  /// to apply it here as well. This is unfortunate, but needed to support both lifecycle nodes and
  /// regular nodes without templates.
  static std::string extend_name_with_sub_namespace(const std::string &name,
                                                    const std::string &sub_namespace) {
    std::string name_with_sub_namespace(name);
    if (!sub_namespace.empty() && name.front() != '/' && name.front() != '~') {
      name_with_sub_namespace = sub_namespace + "/" + name;
    }
    return name_with_sub_namespace;
  }

  /// Installs the validator callback
  void add_parameter_validator_if_needed() {
    if (validate_param_cb_) return;
    auto const set_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto &parameter : parameters) {
        /// We want to skip validating parameters that we didn't declare, for example here have
        /// other parameters like qos_overrides./tf.publisher.durability.
        if (!parameter_validators_.contains(parameter.get_name())) continue;
        const auto &validator = parameter_validators_.at(parameter.get_name());
        auto maybe_error = validator(parameter);
        if (maybe_error) {
          result.successful = false;
          result.reason = *maybe_error;
          break;
        }
      }
      return result;
    };
    this->validate_param_cb_ =
        node_parameters_->add_on_set_parameters_callback(set_param_cb);
  }

  void add_tf_listener_if_needed() {
    if (tf2_listener_)  /// We need only one subscription on /tf, but we can have multiple
                              /// transforms on which we listen to
      return;
    tf2_listener_ = std::make_shared<TFListener>(static_cast<NodeInterfaces&>(*this));
  }

  std::unordered_map<std::string, std::pair<std::shared_ptr<rclcpp::ParameterEventHandler>,
                                            std::shared_ptr<rclcpp::ParameterCallbackHandle>>>
      parameters_;
  std::unordered_map<std::string, FValidate> parameter_validators_;

  /// Callback for validating a list of parameter updates.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_param_cb_;

  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscribers_;

  /// TF Support
  std::shared_ptr<TFListener> tf2_listener_;
  /// This is a simple wrapper around a publisher that publishes on /tf. There is really nothing intereseting under the
  /// hood of tf2_ros::TransformBroadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

class Context;
/// The default augmentation of a stream implementation: The context, holding ROS-related stuff, and
/// some names for easy debugging.
class StreamImplDefault {
public:
  /// A weak reference to the Context, it is needed so that Streams can create more streams that
  /// need access to the ROS node, i.e. `.publish`.
  std::weak_ptr<Context> context;
  /// The class name, i.e. the name of the type, for example "SubscriberStream<std::string>"
  std::string class_name;
  /// A name to identify this node among multiple ones with the same type, usually the topic
  /// or service name
  std::string name;
};

/// A tag to be able to recognize the type "Stream", all types deriving from StreamTag satisfy the `AnyStream` concept. \sa AnyStream
struct StreamTag {};

template <class T>
constexpr bool is_stream = std::is_base_of_v<StreamTag, T>;

template <typename T, typename Base>
concept ConvertibleTo = std::convertible_to<T, Base>;

/// A stream type with any error or value type. \sa StreamTag
template <class T>
concept AnyStream = std::is_base_of_v<StreamTag, T>;

/// A stream type is error-free, meaning its Error is Nothing.
template <class T>
concept ErrorFreeStream = AnyStream<T> && std::is_same_v<ErrorOf<T>, Nothing>;

template<class T>
constexpr auto is_error_free = std::is_same_v<ErrorOf<T>, Nothing>;

/// Any Stream that holds a value V
template <class T, class V, class E=Nothing>
concept StreamOf = AnyStream<T> && std::is_same_v<ValueOf<T>, Nothing>;

/// A stream type is error-free, meaning its Error is Nothing.
//concept DurationLike = Duration || double;

/// A type that is either
template <class T, typename Base>
concept Like = std::convertible_to<T, Base>;// || (AnyStream<T> && std::is_same_v<ValueOf<T>, Base>);

template <class T>
constexpr void assert_stream_holds_tuple() {
  static_assert(is_tuple_v<MessageOf<T>>, "The Stream must hold a tuple as a value for unpacking.");
}

// Assert that all Streams types hold the same value
template <AnyStream First, AnyStream... Rest>
constexpr void assert_all_stream_values_are_same() {
  // Static assert that each T::Value is the same as First::Value
  static_assert((std::is_same_v<MessageOf<First>, MessageOf<Rest>> && ...),
                "The values of all the streams must be the same");
}

template <class T>
struct crtp {
  T &underlying() { return static_cast<T &>(*this); }
  T const &underlying() const { return static_cast<T const &>(*this); }
};


/// Implements the required interface of C++20's coroutines so that Streams can be used with
/// co_await syntax and inside coroutines.
template <class DerivedStream>
struct StreamCoroutinesSupport : public crtp<DerivedStream> {
  /// We are a promise
  using promise_type = DerivedStream;

  StreamCoroutinesSupport() {
    if (icey_coro_debug_print) std::cout << get_type_info() << " Constructor called" << std::endl;
  }

  ~StreamCoroutinesSupport() {
    if (icey_coro_debug_print) std::cout << get_type_info() << " Destructor called" << std::endl;
  }

  std::string get_type_info() const {
    std::stringstream ss;
    auto this_class = boost::typeindex::type_id_runtime(*this).pretty_name();
    ss << "[" << this_class << " @ 0x" << std::hex << size_t(this) << " (impl @ "
       << (this->underlying().impl().p_.lock() ? std::to_string(size_t(this->underlying().impl().get())) : "nullptr") << ")] ";
    return ss.str();
  }

  /// We are still a promise
  DerivedStream get_return_object() {
    // std::cout << get_type_info() <<   " get_return_object called" << std::endl;
    return this->underlying();
  }

  /// We never already got something since this is a stream (we always first need to spin the
  /// ROS executor to get a message), so we never suspend.
  std::suspend_never initial_suspend() {
    // std::cout << get_type_info() <<   " initial_suspend called" << std::endl;
    return {};
  }

  /// return_value returns the value of the Steam.
  auto return_value() { return this->underlying().impl()->value(); }

  /// If the return_value function is called with a value, it *sets* the value, makes sense
  /// right ? No ? Oh .. (Reference:
  /// https://devblogs.microsoft.com/oldnewthing/20210330-00/?p=105019)
  template <class T>
  void return_value(const T &x) const {
    if (icey_coro_debug_print)
      std::cout << this->underlying().get_type_info() << " return value for "
                << boost::typeindex::type_id_runtime(x).pretty_name() << " called " << std::endl;
    this->underlying().impl()->set_value(x);
  }
  /// We already handle exceptions in the Stream, so here we do nothing.
  void unhandled_exception() {}

  /// We do not need to do anything at the end, no extra cleanups needed.
  std::suspend_never final_suspend() const noexcept { return {}; }

  /// Make a Value to a stream (promisify it) if needed, meaning if it is not already a Stream.
  /// We also need to get the context from the given Stream and set it to our stream,
  /// since the compiler creates new Streams by just calling operator new. This creates Streams with
  /// no context, somethign we do not want. But luckily, we always got a Stream that already has a
  /// context so we obtain it from there.
  template <class ReturnType>
  auto await_transform(ReturnType x) {
    if (icey_coro_debug_print) {
      auto to_type = boost::typeindex::type_id_runtime(x).pretty_name();
      std::cout << this->underlying().get_type_info() << " await_transform called to " << to_type
                << std::endl;
    }
    if constexpr (is_stream<ReturnType>) {
      this->underlying().impl()->context =
          x.impl()->context;  /// Get the context from the input stream
      return x;
    } else {
      return this->underlying().template transform_to<ReturnType, Nothing>();
    }
  }

  /// Spin the ROS executor until this Stream has something (a value or an error).
  void spin_executor() {
    this->underlying().impl()->context.lock()->spin_executor_until_stream_has_some(
        this->underlying());
  }

  /// Returns whether the stream has a value or an error.
  bool await_ready() { return !this->underlying().impl()->has_none(); }
  /// Spin the ROS event loop until we have a value or an error.
  bool await_suspend(auto) {
    if (icey_coro_debug_print)
      std::cout << this->underlying().get_type_info() << " await_suspend called" << std::endl;
    if (this->underlying().impl()->context.expired()) {
      throw std::logic_error("Stream has not context");
    }
    this->spin_executor();
    return false;  /// Resume the current coroutine, see
                   /// https://en.cppreference.com/w/cpp/language/coroutines
  }

  /// Take the value out of the stream and return it (this function gets called after await_suspend
  /// has finished spinning the ROS executor). \returns If an error is possible (ErrorValue is not
  /// Nothing), we return a Result<Value, ErrorValue>, otherwise we return just the Value to not
  /// force the user to write unnecessary error handling/unwraping code.
  auto await_resume() {
    if (icey_coro_debug_print)
      std::cout << this->underlying().get_type_info() << " await_resume called" << std::endl;
    return this->underlying().impl()->take();
  }
};

/// Adds a default extention inside the impl::Stream by default.
/// Handy to not force the user to declare this, i.e. to not leak implementation details.
template <class Base>
struct WithDefaults : public Base, public StreamImplDefault {};

template <class V>
struct TimeoutFilter;
template <class V>
struct ServiceClient;
template <class V>
struct Buffer;

/// \brief A stream, an abstraction over an asynchronous sequence of values.
/// It has a state of type Result and a list of callbacks that get notified when this state changes.
/// It is conceptually very similar to a promise in JavaScript except that state transitions are not
/// final. This is the base class for all the other streams.
///
/// \tparam _Value the type of the value
/// \tparam _ErrorValue the type of the error. It can also be an exception.
/// \tparam ImplBase a class from which the implementation (impl::Stream) derives, used as an
/// extention point. \note This class does not any fields other than a pointer to the actual
/// implementation, `std::shared_ptr<Impl>`, i.e. it uses the PIMPL idiom. When deriving from this
/// class to implement new Streams, you should never add additional fields because this Stream may
/// go out of scope. Instead, put the additional fields that you need in a separate struct
/// `MyStreamImpl` and pass it as the `ImplBase` template parameter. Then, these fields become
/// available through `impl().<my_field>`, i.e. the Impl-class will derive from ImplBase. This is
/// how you should extend the Stream class when implementing your own Streams.
template <class _Value, class _ErrorValue = Nothing, class ImplBase = Nothing>
class Stream : public StreamTag,
               public StreamCoroutinesSupport<Stream<_Value, _ErrorValue, ImplBase>> {
  static_assert(std::is_default_constructible_v<ImplBase>, "Impl must be default constructable");
  friend Context;
public:
  using Value = _Value;
  using ErrorValue = _ErrorValue;
  using Self = Stream<_Value, _ErrorValue, ImplBase>;
  /// The actual implementation of the Stream.
  using Impl = impl::Stream<Value, ErrorValue, WithDefaults<ImplBase>, WithDefaults<Nothing>>;

#ifdef ICEY_DEBUG_PRINT_STREAM_ALLOCATIONS
  Stream() {
    std::cout << "Created new Stream: " << this->get_type_info() << std::endl;
  }
  ~Stream() {
    std::cout << "Destroying Stream: " << this->get_type_info() << std::endl;
  }
#else 
  /// Leaves stream in invalid state. Needed for some delayed stuff like ParameterStream \todo rem
  Stream() {}
#endif  

  /// Create s new stream using the context. 
  explicit Stream(NodeBookkeeping &book) { 
    book.stream_impls_.push_back(impl_);
  }
  explicit Stream(std::shared_ptr<Impl> impl): impl_(impl) {}

  /// Returns a weak pointer to the implementation.
  Weak<Impl> impl() const { return impl_; }

  /// \returns A new Stream that changes it's value to y every time this
  /// stream receives a value x, where y = f(x).
  /// The type of the returned stream is:
  /// - Stream<Nothing, _ErrorValue> if F is (X) -> void
  /// - Stream<NewValue, NewError> if F is (X) -> Result<NewValue, NewError>
  /// - Stream<NewValue, _ErrorValue> if F is (X) -> std::optional<NewValue>
  /// - Stream<Y, _ErrorValue> otherwise
  /// \tparam F: Must be (X) -> Y, where X is:
  ///  - V_1, ..., V_n if Value is std::tuple<V_1, ..., V_n>
  ///  - Value otherwise
  template <class F>
  auto then(F &&f) {
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream cannot have values, so you cannot register then() on it.");
    return create_from_impl(impl()->then(std::forward<F>(f)));
  }

  /// \returns A new Stream that changes it's value to y every time this
  /// stream receives an error x, where y = f(x).
  /// The type of the returned stream is:
  /// - Stream<Nothing, Nothing> if F is (X) -> void
  /// - Stream<NewValue, NewError> if F is (X) -> Result<NewValue, NewError>
  /// - Stream<NewValue, Nothing> if F is (X) -> std::optional<NewValue>
  /// - Stream<Y, Nothing> otherwise
  /// \tparam F: Must be (X) -> Y, where X is:
  ///  - V_1, ..., V_n if Value is std::tuple<V_1, ..., V_n>
  ///  - Value otherwise
  template <class F>
  auto except(F &&f) {
    static_assert(!std::is_same_v<ErrorValue, Nothing>,
                  "This stream cannot have errors, so you cannot register except() on it.");
    return create_from_impl(impl()->except(std::forward<F>(f)));
  }

  /// Connect this Stream to the given output stream so that the output stream receives all the
  /// values.
  /// \todo remove this, use unwrap
  template <AnyStream Output>
  void connect_values(Output output) {
    this->impl()->register_handler([output_impl = output.impl()](const auto &new_state) {
      if (new_state.has_value()) {
        output_impl->put_value(new_state.value());
      } else if (new_state.has_error()) {
      }
    });
  }

  /// Creates a ROS publisher by creating a new stream of type PublisherType and connecting it to
  /// this stream.
  template <AnyStream PublisherType, class... Args>
  void publish(Args &&...args) {
    assert_we_have_context();
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream does not have a value, there is nothing to publish, so you cannot "
                  "call publish() on it.");
    /// We create this through the context to register it for attachment to the ROS node
    auto output = this->impl()->context.lock()->template create_stream<PublisherType>(
        std::forward<Args>(args)...);
    this->connect_values(output);
  }

  /// \return A new Stream that errors on a timeout, i.e. when this stream has not received any
  /// value for some time `max_age`. \param create_extra_timer If set to false, the timeout will
  /// only be detected after at least one message was received. If set to true, an extra timer is
  /// created so that timeouts can be detected even if no message is received.
  TimeoutFilter<Value> timeout(const Duration &max_age, bool create_extra_timer = true) {
    assert_we_have_context();
    /// We create this through the context to register it for the ROS node
    return this->impl()->context.lock()->template create_stream<TimeoutFilter<_Value>>(
        *this, max_age, create_extra_timer);
  }

  /// Publish the value of this Stream.
  void publish(const std::string &topic_name,
               const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS()) {
    assert_we_have_context();
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream does not have a value, there is nothing to publish, so you cannot "
                  "call publish() on it.");
    this->impl()->context.lock()->create_publisher(*this, topic_name, qos);
  }

  /// Publish a transform using the `TFBroadcaster` in case this Stream holds a Value of type
  /// `geometry_msgs::msg::TransformStamped`.
  void publish_transform() {
    assert_we_have_context();
    static_assert(std::is_same_v<Value, geometry_msgs::msg::TransformStamped>,
                  "The stream must hold a Value of type "
                  "geometry_msgs::msg::TransformStamped[::SharedPtr] to be able to call "
                  "publish_transform() on it.");
    this->impl()->context.lock()->create_transform_publisher(*this);
  }

  /// Calls a ROS service with the Value that this Stream holds. It returns a new ServiceClient
  /// stream, which gets called by this stream.
  template <class ServiceT>
  ServiceClient<ServiceT> call_service(const std::string &service_name, const Duration &timeout,
                                       const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    assert_we_have_context();
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream does not have a value, there is nothing to publish, you cannot "
                  "call publish() on it.");
    return this->impl()->context.lock()->template create_client<ServiceT>(*this, service_name,
                                                                          timeout, qos);
  }

  /// Unpacks an Stream holding a tuple as value to multiple Streams for each tuple element.
  /// Given that `Value` is of type `std::tuple<Value1, Value2, ..., ValueN>`, this returns
  /// `std::tuple< Stream<Value1>, Stream<Value2>, ..., Stream<ValueN>>`
  auto unpack() {
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream does not have a value, there is nothing to unpack().");
    static_assert(is_tuple_v<Value>, "The Value must be a tuple for .unpack()");
    constexpr size_t tuple_sz = std::tuple_size_v<ValueOf<Self>>;
    /// hana::to<> is needed to make a sequence from a range, otherwise we cannot transform it, see
    /// https://stackoverflow.com/a/33181700
    constexpr auto indices = hana::to<hana::tuple_tag>(hana::range_c<std::size_t, 0, tuple_sz>);
    auto hana_tuple_output = hana::transform(indices, [*this](auto I) mutable {
      return then([I](const auto &...args) {  /// Need to take variadic because then()
                                              /// automatically unpacks tuples
        return std::get<I>(std::forward_as_tuple(
            args...));  /// So we need to pack this again in a tuple and get the index.
      });
    });
    /// Now create a std::tuple from the hana::tuple to be able to use structured bindings
    return hana::unpack(hana_tuple_output,
                        [](const auto &...args) { return std::make_tuple(args...); });
  }

  /// Returns a new Stream that cannot have Errors by handling the error
  /// \todo implement more cleanly
  template <class F>
  Stream<Value, Nothing> unwrap_or(F &&f) {
    this->except(std::forward<F>(f));
    auto new_stream = this->template transform_to<Value, Nothing>();
    this->connect_values(new_stream);
    return new_stream;
  }

  /// Outputs the Value only if the given predicate f returns true.
  template <class F>
  Stream<Value, ErrorValue> filter(F &&f) {
    return this->then([f = std::move(f)](auto x) -> std::optional<Value> {
      if (!f(x))
        return {};
      else
        return x;
    });
  }

  /// Buffers N elements
  Buffer<Value> buffer(std::size_t N) const {
    return this->template create_stream<Buffer<Value>>(N, *this);
  }

protected:
  void assert_we_have_context() {
    if (!this->impl()->context.lock())
      throw std::runtime_error("This stream does not have context");
  }

  /// Creates a new stream of type S using the Context. See Context::create_stream
  template <AnyStream S, class... Args>
  S create_stream(Args &&...args) const {
    return this->impl()->context.lock()->template create_stream<S>(std::forward<Args>(args)...);
  }

  template <class NewValue, class NewError>
  Stream<NewValue, NewError> transform_to() const {
    return create_stream<Stream<NewValue, NewError>>();
  }

  /// Pattern-maching factory function that creates a New Self with different value and error types
  /// based on the passed implementation pointer.
  /// (this is only needed for impl::Stream::done, which creates a new stream that always has
  /// Derived stripped off, i.e. set to Nothing.)
  /// We do NOT add the impl to the context since it is already captured by the handler.
  template <class NewVal, class NewErr>
  Stream<NewVal, NewErr> create_from_impl(
      const std::shared_ptr<
        impl::Stream<NewVal, NewErr, WithDefaults<Nothing>, WithDefaults<Nothing>>> &impl) const {
    Stream<NewVal, NewErr> new_stream(impl);
    new_stream.impl()->context = this->impl()->context;
    return new_stream;
  }

  /// The pointer to the undelying implementation (i.e. PIMPL idiom).
  std::shared_ptr<Impl> impl_{impl::create_stream<Impl>()};
};

template <class Value>
struct BufferImpl {
  std::size_t N{1};
  std::shared_ptr<std::vector<Value>> buffer{std::make_shared<std::vector<Value>>()};
};

/// A Buffer is a Stream that holds an array of values. It accumulates a certain amount of values
/// and only then it has itself a value. It does not have errors since it does not make much sense
/// to accumulate errors.
template <class Value>
struct Buffer : public Stream<std::shared_ptr<std::vector<Value>>, Nothing, BufferImpl<Value>> {
  using Base = Stream<std::shared_ptr<std::vector<Value>>, Nothing, BufferImpl<Value>>;
  template <AnyStream Input>
  explicit Buffer(NodeBookkeeping &node, std::size_t N, Input input): Base(node) {
    this->impl()->N = N;
    input.impl()->register_handler([impl = this->impl()](auto x) {
      if (impl->buffer->size() == impl->N) {
        impl->put_value(impl->buffer);
        impl->buffer->clear();
      } else {
        impl->buffer->push_back(x.value());
      }
    });
  }
};

/// What follows are parameters.
/// Traits to recognize valid types for ROS parameters (Reference:
/// https://docs.ros.org/en/jazzy/p/rcl_interfaces/interfaces/msg/ParameterValue.html)
template <class T>
struct is_valid_ros_param_type : std::false_type {};
template <>
struct is_valid_ros_param_type<bool> : std::true_type {};
template <>
struct is_valid_ros_param_type<int64_t> : std::true_type {};

template <>
struct is_valid_ros_param_type<double> : std::true_type {};
template <>
struct is_valid_ros_param_type<std::string> : std::true_type {};
/// Array type
template <class T>
struct is_valid_ros_param_type<std::vector<T>> : is_valid_ros_param_type<T> {};
template <class T, std::size_t N>
struct is_valid_ros_param_type<std::array<T, N>> : is_valid_ros_param_type<T> {};
/// Byte array, extra specialization since byte is not allowed as scalar type, only byte arrays
template <>
struct is_valid_ros_param_type<std::vector<std::byte>> : std::true_type {};
template <std::size_t N>
struct is_valid_ros_param_type<std::array<std::byte, N>> : std::true_type {};

template <class T>
struct t_is_std_array : std::false_type {};

template <class T, std::size_t N>
struct t_is_std_array<std::array<T, N>> : std::true_type {};

template <class T>
constexpr bool is_std_array = t_is_std_array<T>::value;

/// What follows, is an improved parameters API. For API docs, see
/// https://docs.ros.org/en/jazzy/p/rcl_interfaces First, some constraints we can impose on
/// parameters:

/// A closed interval, meaning a value must be greater or equal to a minimum value
/// and less or equal to a maximal value.
template <class Value>
struct Interval {
  static_assert(std::is_arithmetic_v<Value>, "The value type must be a number");
  //Interval(Like<Value> auto _minimum, Like<Value> auto _maximum): minimum(_minimum), maximum(_maximum) {}
  Interval(Value _minimum, Value _maximum): minimum(_minimum), maximum(_maximum) {}
  Value minimum;
  Value maximum;
};

/// A set specified by a given list of values that are in the set.
template <class Value>
struct Set {
  Set(std::vector<Value> l) : set_of_values(l.begin(), l.end()) {}
  // explicit Set(const Value ...values) : set_of_values(values...) {}
  std::unordered_set<Value> set_of_values;
};

/*!
   A parameter validator, validating parameter updates. It is able to constrain parameter values.
   It is essentially a function that returns true if a value is allowed and false otherwise.
   \tparam Value the parameter type (i.e. double)
*/
template <class Value>
struct Validator {
  using ROSValue = std::conditional_t<std::is_unsigned_v<Value>, int, Value>;
  /// The type of the predicate
  using Validate = std::function<std::optional<std::string>(const rclcpp::Parameter &)>;

  /// Allow default-constructed validator, by default allowing all values.
  Validator() {
    if constexpr (std::is_unsigned_v<Value>) {
      descriptor.integer_range.resize(1);
      descriptor.integer_range.front().from_value = 0;
      descriptor.integer_range.front().to_value = std::numeric_limits<int64_t>::max();
      descriptor.integer_range.front().step = 1;
      /// When an interval is set with the descriptor, ROS already validates the parameters, our
      /// custom validator won't even get called.
      validate = get_default_validator();
    } else {
      /// ROS already validates type changes (and disallows them).
      validate = get_default_validator();
    }
  }

  /// Construct explicitly from a validation function (predicate).
  explicit Validator(const Validate &validate) : validate(validate) {}

  /// Allow implicit conversion from some easy sets:
  Validator(const Interval<Value> &interval)  // NOLINT
  {
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range.front().from_value = interval.minimum;
    descriptor.floating_point_range.front().to_value = interval.maximum;
    descriptor.floating_point_range.front().step = 0.1;
    /// When an interval is set with the descriptor, ROS already validates the parameters, our
    /// custom validator won't even be called.
    validate = get_default_validator();
  }

  /// Implicit conversion from a Set of values
  Validator(const Set<Value> &set)  // NOLINT
  {
    validate = [set](const rclcpp::Parameter &new_param) {
      auto new_value = new_param.get_value<ROSValue>();
      std::optional<std::string> result;
      if (!set.set_of_values.count(new_value))
        result = "The given value is not in the set of allowed values";
      return result;
    };
  }

  Validate get_default_validator() const {
    return [](const rclcpp::Parameter &) { return std::optional<std::string>{}; };
  }

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  /// A predicate that indicates whether a given value is feasible.
  /// By default, a validator always returns true since the parameter is unconstrained
  Validate validate;
};

/// An stream for ROS parameters.
template <class _Value>
struct ParameterStream : public Stream<_Value> {
  using Base = Stream<_Value, Nothing>;
  using Value = _Value;
  static_assert(is_valid_ros_param_type<_Value>::value,
                "Type is not an allowed ROS parameter type");

  /// @brief A constructor that should only be used for parameter structs. It does not set the name
  /// of the parameter and therefore leaves this ParameterStream in a not fully initialized state.
  /// Context::declare_parameter_struct later infers the name of this parameter from the name of the
  /// field in the parameter struct and sets it before registering the parameter with ROS.
  /// @param default_value
  /// @param validator the validator implementing constraints
  /// @param description the description written in the ParameterDescriptor
  /// @param read_only if yes, the parameter cannot be modified
  /// @param ignore_override
  ParameterStream(const Value &default_value, const Validator<Value> &validator = {},
                  std::string description = "", bool read_only = false,
                  bool ignore_override = false) {
    this->default_value = default_value;
    this->validator = validator;
    this->description = description;
    this->read_only = read_only;
    this->ignore_override = ignore_override;
  }

  /// @brief The standard constructor used when declaring parameters with
  /// Context::declare_parameter.
  /// @param node
  /// @param parameter_name
  /// @param default_value
  /// @param validator the validator implementing constraints
  /// @param description
  /// @param read_only if yes, the parameter cannot be modified
  /// @param ignore_override
  ParameterStream(NodeBookkeeping &node, const std::string &parameter_name,
                  const Value &default_value, const Validator<Value> &validator = {},
                  std::string description = "", bool read_only = false,
                  bool ignore_override = false)
      : ParameterStream(default_value, validator, description, read_only, ignore_override) {
    
    this->parameter_name = parameter_name;
    this->register_with_ros(node);
  }

  /// Register this paremeter with the ROS node, meaning it actually calls
  /// node->declare_parameter(). After calling this method, this ParameterStream will have a value.
  void register_with_ros(NodeBookkeeping &node) {
    node.add_parameter<Value>(this->parameter_name, this->default_value,
      [impl = this->impl()](const rclcpp::Parameter &new_param) {
        if constexpr (is_std_array<Value>) {
          using Scalar = typename Value::value_type;
          auto new_value = new_param.get_value<std::vector<Scalar>>();
          if (std::declval<Value>().max_size() != new_value.size()) {
            throw std::invalid_argument("Wrong size of array parameter");
          }
          Value new_val_arr{};
          std::copy(new_value.begin(), new_value.end(), new_val_arr.begin());
          impl->put_value(new_val_arr);
        } else {
          Value new_value = new_param.get_value<_Value>();
          impl->put_value(new_value);
        }
      }, this->create_descriptor(),
                              this->validator.validate, this->ignore_override);
    /// Set the default value 
    this->impl()->put_value(this->default_value);
  }

  /// Get the value. Parameters are initialized always at the beginning, so they always have a
  /// value.
  const Value &value() const {
    if (!this->impl()->has_value()) {
      throw std::runtime_error("Parameter '" + this->parameter_name + "' does not have a value");
    }
    return this->impl()->value();
  }

  /// Allow implicit conversion to the stored value type for consistent API between constrained and
  /// non-constrained parameters when using the parameter structs.
  operator Value() const  // NOLINT
  {
    return this->value();
  }
  std::string parameter_name;
protected:
  auto create_descriptor() {
    auto desc = validator.descriptor;
    desc.name = parameter_name;
    desc.description = description;
    desc.read_only = read_only;
    return desc;
  }

  Value default_value;
  Validator<Value> validator;
  std::string description;
  bool read_only = false;
  bool ignore_override = false;
};

/// A class that abstracts a plain value and a ParameterStream so that both are supported. 
/// This is needed for example for timeouts and coordinate system names. 
template<class Value>
struct ValueOrParameter {
  ValueOrParameter() = default;
  /// Convert from something that is like the Value and not a stream, for example "hello" is a const char[5] literal that is like a std::string.
  template<class T>
    requires std::convertible_to<T, Value> && (!AnyStream<T>)
  ValueOrParameter(const T &v) // NOLINT
    : get([value=Value(v)]() { return value; }) {}

  ValueOrParameter(const Value &value) // NOLINT
    : get([value]() { return value; }) {}
  ValueOrParameter(const ParameterStream<Value> &param) // NOLINT
    : get([param_impl = param.impl()]() { return param_impl.lock()->get_value(); }) {}
  /// We use a std::function for the type erasure.
  std::function<Value()> get;
};

/// A stream that represents a regular ROS subscriber. It stores as its value always a shared
/// pointer to the message.
template <class _Message>
struct SubscriptionStream : public Stream<typename _Message::SharedPtr> {
  using Base = Stream<typename _Message::SharedPtr>;
  using Value = typename _Message::SharedPtr;  /// Needed for synchronizer to determine message type
  SubscriptionStream(NodeBookkeeping &node, const std::string &topic_name, const rclcpp::QoS &qos,
                     const rclcpp::SubscriptionOptions &options): Base(node) {
    this->impl()->name = topic_name;
    node.add_subscription<_Message>(
        topic_name,
        [impl = this->impl()](typename _Message::SharedPtr new_value) {
          impl->put_value(new_value);
        },
        qos, options);
  }
};

struct InterpolateableStreamTag {};
template <class _Message, class DerivedImpl>
struct InterpolateableStream
    : public InterpolateableStreamTag,
      public Stream<typename _Message::SharedPtr, std::string, DerivedImpl> {
  using Base = Stream<typename _Message::SharedPtr, std::string, DerivedImpl>;
  using Base::Base;
  
  using MaybeValue = std::optional<typename _Message::SharedPtr>;
  /// Get the measurement at a given time point. Returns nothing if the buffer is empty or
  /// an extrapolation would be required.
  virtual MaybeValue get_at_time(const rclcpp::Time &time) const = 0;
};

/// An interpolatable stream is one that buffers the incoming messages using a circular buffer
/// and allows to query the message at a given point, using interpolation.
template <class T>
constexpr auto hana_is_interpolatable(T) {
  if constexpr (std::is_base_of_v<InterpolateableStreamTag, T>)
    return hana::bool_c<true>;
  else
    return hana::bool_c<false>;
}

struct TransformSubscriptionStreamImpl {
  using Message = geometry_msgs::msg::TransformStamped;
  ValueOrParameter<std::string> source_frame;
  ValueOrParameter<std::string> target_frame;
  /// We allocate a single message that we share with the other streams when notifying them.
  /// Note that we cannot use the base value since it is needed for notifying, i.e. it is cleared
  std::shared_ptr<Message> shared_value{std::make_shared<Message>()};
  /// We do not own the listener, the Book owns it
  std::weak_ptr<TFListener> tf2_listener;
};

/// A Stream that represents a subscription between two coordinate systems. (See TFListener)
/// Values can also be pulled with get_at_time.
struct TransformSubscriptionStream
    : public InterpolateableStream<geometry_msgs::msg::TransformStamped,
                                   TransformSubscriptionStreamImpl> {
  using Base = InterpolateableStream<geometry_msgs::msg::TransformStamped, TransformSubscriptionStreamImpl>;
  using Message = geometry_msgs::msg::TransformStamped;
  using MaybeValue = InterpolateableStream<Message, TransformSubscriptionStreamImpl>::MaybeValue;

  TransformSubscriptionStream(NodeBookkeeping &node, 
                          ValueOrParameter<std::string> target_frame,
                          ValueOrParameter<std::string> source_frame): Base(node) {
    this->impl()->target_frame = target_frame;
    this->impl()->source_frame = source_frame;
    this->impl()->tf2_listener = node.add_tf_subscription(
        target_frame.get,
        source_frame.get,
        [impl = this->impl()](const geometry_msgs::msg::TransformStamped &new_value) {
          *impl->shared_value = new_value;
          impl->put_value(impl->shared_value);
        },
        [impl = this->impl()](const tf2::TransformException &ex) { impl->put_error(ex.what()); });
  }
  /// @brief Look up the transform at the given time point, does not wait but instead only return
  /// something if the transform is already in the buffer.
  /// @param time
  /// @return The transform or nothing if it has not arrived yet.
  MaybeValue get_at_time(const rclcpp::Time &time) const override {
    try {
      // Note that this call does not wait, the transform must already have arrived.
      *this->impl()->shared_value = this->impl()->tf2_listener.lock()->buffer_->lookupTransform(
          this->impl()->target_frame.get(), this->impl()->source_frame.get(), time);
      return this->impl()->shared_value;
    } catch (const tf2::TransformException &e) {
      this->impl()->put_error(e.what());
      return {};
    }
  }
};

struct TimerImpl {
  size_t ticks_counter{0};
  rclcpp::TimerBase::SharedPtr timer;
};

/// A Stream representing a ROS-Timer. It saves the number of ticks as it's value.
struct TimerStream : public Stream<size_t, Nothing, TimerImpl> {
  using Base = Stream<size_t, Nothing, TimerImpl>;
  TimerStream(NodeBookkeeping &node, const Duration &interval, bool is_one_off_timer): Base(node) {
    this->impl()->name = "timer";
    this->impl()->timer = node.add_timer(interval, [impl = this->impl(), is_one_off_timer]() {
      impl->put_value(impl->ticks_counter);
      /// Needed as separate state as it might be resetted in async/await mode
      impl->ticks_counter++;
      if (is_one_off_timer) impl->timer->cancel();
    });
  }
};

template <class _Value>
struct PublisherImpl {
  typename rclcpp::Publisher<remove_shared_ptr_t<_Value>>::SharedPtr publisher;
  void publish(const _Value &message) {
    // We cannot pass over the pointer since publish expects a unique ptr and we got a
    // shared ptr. We cannot just create a unique_ptr from the shared ptr because we cannot ensure
    // the shared_ptr is not referenced somewhere else.
    /// We could check whether use_count is one but this is not a reliable indicator whether the
    /// object not referenced anywhere else.
    // This is because the use_count
    /// can change in a multithreaded program immediatelly after it was retrieved, see:
    /// https://en.cppreference.com/w/cpp/memory/shared_ptr/use_count (Same holds for
    /// shared_ptr::unique, which is defined simply as shared_ptr::unique -> bool: use_count() == 1)
    /// Therefore, we have to copy the message for publishing.
    if constexpr (is_shared_ptr<_Value>)
      publisher->publish(*message);
    else
      publisher->publish(message);
  }
};

/// A Stream representing a ROS-publisher. Value can be either a Message or shared_ptr<Message>
template <class _Value>
struct PublisherStream : public Stream<_Value, Nothing, PublisherImpl<_Value>> {
  using Base = Stream<_Value, Nothing, PublisherImpl<_Value>>;
  using Message = remove_shared_ptr_t<_Value>;
  static_assert(rclcpp::is_ros_compatible_type<Message>::value,
                "A publisher must use a publishable ROS message (no primitive types are possible)");
  template <AnyStream Input = Stream<_Value>>
  PublisherStream(NodeBookkeeping &node, const std::string &topic_name,
                  const rclcpp::QoS qos = rclcpp::SystemDefaultsQoS(),
                  const rclcpp::PublisherOptions publisher_options = {},
                  Input *maybe_input = nullptr) : Base(node){
    this->impl()->name = topic_name;
    this->impl()->publisher = node.add_publisher<Message>(topic_name, qos, publisher_options);
    this->impl()->register_handler(
        [impl = this->impl()](const auto &new_state) { impl->publish(new_state.value()); });
    if (maybe_input) {
      maybe_input->connect_values(*this);
    }
  }
  void publish(const _Value &message) const { this->impl()->publish(message); }
};

// A Stream representing a transform broadcaster that publishes transforms on TF.
struct TransformPublisherStream : public Stream<geometry_msgs::msg::TransformStamped> {
  using Base = Stream<geometry_msgs::msg::TransformStamped>;
  using Value = geometry_msgs::msg::TransformStamped;
  TransformPublisherStream(NodeBookkeeping &node): Base(node) {
    this->impl()->name = "tf_pub";
    auto tf_broadcaster = node.add_tf_broadcaster_if_needed();
    this->impl()->register_handler([tf_broadcaster](const auto &new_state) {
      tf_broadcaster->sendTransform(new_state.value());
    });
  }
};

template <class ServiceT>
struct ServiceStreamImpl {
  std::shared_ptr<rclcpp::Service<ServiceT>> service;
};

using RequestID = std::shared_ptr<rmw_request_id_t>;
/// A Stream representing a ROS service (server). It stores as its value the RequestID and the the Request itself. It supports synchronous as well as asynchronous responding.
///
/// See as a reference:
/// - https://github.com/ros2/rclcpp/pull/1709
/// - https://github.com/ros2/rclcpp/issues/1707
/// - https://github.com/tgroechel/lifecycle_prac/blob/main/src/async_srv.cpp#L10-L69C1
/// - https://github.com/ijnek/nested_services_rclcpp_demo
template <class _ServiceT>
struct ServiceStream : public Stream< std::tuple<RequestID, std::shared_ptr<typename _ServiceT::Request>>, 
  Nothing, ServiceStreamImpl<_ServiceT> > {
  using Base = Stream< std::tuple<RequestID, std::shared_ptr<typename _ServiceT::Request>>, Nothing, ServiceStreamImpl<_ServiceT> >;
  using Request = std::shared_ptr<typename _ServiceT::Request>;
  using Value = Request;
  using Response = std::shared_ptr<typename _ServiceT::Response>;
  
  /// The type of the user callback that can response synchronously (i.e. immediately): It receives the request and returns the response.
  using SyncCallback = std::function<Response(Request)>;
  using AsyncCallback = std::function<Stream<Response>(Request)>;

  ServiceStream(NodeBookkeeping &node, const std::string &service_name,
                         SyncCallback sync_callback = {},
                         const rclcpp::QoS &qos = rclcpp::ServicesQoS()): Base(node) {
    this->impl()->service = node.add_service<_ServiceT>(
        service_name,
        [impl = this->impl(), sync_callback](RequestID request_id, Request request) {
          impl->put_value(std::make_tuple(request_id, request));
          if(sync_callback) {
            auto response = sync_callback(request);
            impl->service->send_response(*request_id, *response);
          }
        },
        qos);
  }
  /// Send the service response to the request identified by the request_id.
  ///  It is RMW implementation defined whether this happens synchronously or asynchronously.
  /// \throws any exception from `rclcpp::exceptions::throw_from_rcl_error()`
  /// See for a detailed documentation:
  /// - The C-API that rclcpp uses more or less directly: [rcl_send_response](http://docs.ros.org/en/jazzy/p/rcl/generated/function_service_8h_1a8631f47c48757228b813d0849d399d81.html#_CPPv417rcl_send_responsePK13rcl_service_tP16rmw_request_id_tPv)
  /// - One leayer below: [rmw_send_response](https://docs.ros.org/en/humble/p/rmw/generated/function_rmw_8h_1abb55ba2b2a957cefb0a77b77ddc5afda.html)
  void respond(RequestID request_id, Response response) {
    this->impl()->service->send_response(*request_id, *response);
  }
};

template <class _ServiceT>
struct ServiceClientImpl {
  using Client = rclcpp::Client<_ServiceT>;
  typename Client::SharedPtr client;
  Duration timeout{};
  std::optional<typename Client::SharedFutureWithRequestAndRequestId> maybe_pending_request;
};

/// A Stream representing a service client. It stores the response as it's value.
template <class _ServiceT>
struct ServiceClient : public Stream<typename _ServiceT::Response::SharedPtr, std::string,
                                     ServiceClientImpl<_ServiceT>> {
  using Base = Stream<typename _ServiceT::Response::SharedPtr, std::string, ServiceClientImpl<_ServiceT>>;
  using Request = typename _ServiceT::Request::SharedPtr;
  using Response = typename _ServiceT::Response::SharedPtr;
  using Client = rclcpp::Client<_ServiceT>;
  using Future = typename Client::SharedFutureWithRequest;

  ServiceClient(NodeBookkeeping &node, const std::string &service_name, const Duration &timeout,
                const rclcpp::QoS &qos = rclcpp::ServicesQoS()): Base(node) {
    this->impl()->name = service_name;
    this->impl()->timeout = timeout;
    this->impl()->client = node.add_client<_ServiceT>(service_name, qos);
  }

  auto &call(Request request) const {
    if (!wait_for_service(this->impl())) return *this;
    this->impl()->maybe_pending_request = this->impl()->client->async_send_request(
        request,
        [impl = this->impl()](Future response_futur) { on_response(impl, response_futur); });
    return *this;
  }

protected:
  static bool wait_for_service(auto impl) {
    if (!impl->client->wait_for_service(impl->timeout)) {
      // Reference:https://github.com/ros2/examples/blob/rolling/rclcpp/services/async_client/main.cpp#L65
      if (!rclcpp::ok()) {
        impl->put_error("INTERRUPTED");
      } else {
        impl->put_error("SERVICE_UNAVAILABLE");
      }
      return false;
    }
    return true;
  }

  static void on_response(auto impl, Future response_futur) {
    if (response_futur.valid()) {
      impl->maybe_pending_request = {};
      impl->put_value(response_futur.get().second);
    } else {
      // Reference:
      // https://github.com/ros2/examples/blob/rolling/rclcpp/services/async_client/main.cpp#L65
      if (!rclcpp::ok()) {
        impl->put_error("rclcpp::FutureReturnCode::INTERRUPTED");
      } else {
        impl->put_error("rclcpp::FutureReturnCode::TIMEOUT");
      }
      /// Now do the weird cleanup thing that the API-user definitely neither does need to care
      /// nor know about:
      impl->client->remove_pending_request(impl->maybe_pending_request.value());
      /// I'm not sure this will still not leak memory smh:
      /// https://github.com/ros2/rclcpp/issues/1697. There is a ROS example that launches a timer
      /// which polls for dead requests and cleans them up. That's why I'll put this assertion here:
      if (size_t num_requests_pruned = impl->client->prune_pending_requests() != 0) {
        throw std::runtime_error(
            "Pruned some more requests even after calling remove_pending_request(), you should buy "
            "a new RPC library.");
      }
      impl->maybe_pending_request = {};
    }
  }
};

/// A filter that detects timeouts, i.e. whether a value was received in a given time window.
/// It simply passes over the value if no timeout occured, and errors otherwise.
/// \tparam _Value the value must be a message that has a header stamp
template <class Value>
struct TimeoutFilter
    : public Stream<Value, std::tuple<rclcpp::Time, rclcpp::Time, rclcpp::Duration>> {
  using Base = Stream<Value, std::tuple<rclcpp::Time, rclcpp::Time, rclcpp::Duration>>;
  /// Construct the filter an connect it to the input.
  /// \tparam Input another Stream that holds as a value a ROS message with a header stamp
  /// \param node the node is needed to know the current time
  /// \param input another Stream which is the input to this filter
  /// \param max_age a maximum age the message is allowed to have.
  /// \param create_extra_timer If set to false, the timeout will only be detected after at least
  /// one message was received. If set to true, an extra timer is created so that timeouts can be
  /// detected even if no message is received
  template <AnyStream Input>
  TimeoutFilter(NodeBookkeeping &node, Input input, const Duration &max_age,
                bool create_extra_timer = true) : Base(node) {
    auto node_clock = node.get_node_clock_interface();
    rclcpp::Duration max_age_ros(max_age);
    auto check_state = [impl = this->impl(), node_clock, max_age_ros](const auto &new_state) {
      if (!new_state.has_value()) return true;
      const auto &message = new_state.value();
      rclcpp::Time time_now = node_clock->get_clock()->now();
      rclcpp::Time time_message = rclcpp::Time(message->header.stamp);
      if ((time_now - time_message) <= max_age_ros) {
        impl->put_value(message);
        return false;
      } else {
        impl->put_error(std::make_tuple(time_now, time_message, max_age_ros));
        return true;
      }
    };
    if (create_extra_timer) {
      auto timer = input.impl()->context.lock()->create_timer(max_age);
      timer.then([timer_impl=timer.impl(), input_impl = input.impl(), check_state](size_t) {
        bool timeout_occured = check_state(input_impl->get_state());
        if (!timeout_occured) timer_impl->timer->reset();
      });
    } else {
      input.impl()->register_handler(check_state);
    }
  }
};

/// Adapts the `message_filters::SimpleFilter` to our
/// `Stream` (which is a similar concept).
/// \note This is essentially the same as what
/// `message_filters::Subscriber` does:
/// https://github.com/ros2/message_filters/blob/humble/include/message_filters/subscriber.h#L349
template <class _Message>
struct SimpleFilterAdapter : public Stream<typename _Message::SharedPtr, Nothing, message_filters::SimpleFilter<_Message> > {
  using Base = Stream<typename _Message::SharedPtr, Nothing, message_filters::SimpleFilter<_Message> >;
  /// Constructs a new instance and connects this Stream to the `message_filters::SimpleFilter` so
  /// that `signalMessage` is called once a value is received.
  SimpleFilterAdapter(NodeBookkeeping &node): Base(node) {
    this->impl()->register_handler([impl=this->impl()](const auto &new_state) {
      using Event = message_filters::MessageEvent<const _Message>;
      impl->signalMessage(Event(new_state.value()));
    });
  }
};

template <class... Messages>
struct SynchronizerStreamImpl {
  using Policy = message_filters::sync_policies::ApproximateTime<Messages...>;
  using Sync = message_filters::Synchronizer<Policy>;
  using Inputs = std::tuple<std::shared_ptr<SimpleFilterAdapter<Messages>>...>;
  const auto &inputs() const { return inputs_; }

  void create_mfl_synchronizer(NodeBookkeeping &node, uint32_t queue_size) {
    queue_size_ = queue_size;
    inputs_ = std::make_tuple(SimpleFilterAdapter<Messages>(node)...);
    auto synchronizer = std::make_shared<Sync>(Policy(queue_size_));
    synchronizer_ = synchronizer;
    /// Connect with the input streams
    std::apply(
        [synchronizer](auto &...input_filters) { synchronizer->connectInput(*input_filters->impl()...); },
        inputs_);
    /// This parameter setting is from the examples
    synchronizer_->setAgePenalty(0.50);
  }
  uint32_t queue_size_{100};

  Inputs inputs_;
  std::shared_ptr<Sync> synchronizer_;
};

/// A Stream representing an approximate time synchronizer.
/// \warning All inputs must have the same QoS accorcding to the documentation of message_filters
template <class... Messages>
class SynchronizerStream : public Stream<std::tuple<typename Messages::SharedPtr...>, std::string,
                                         SynchronizerStreamImpl<Messages...>> {
public: 
  using Base = Stream<std::tuple<typename Messages::SharedPtr...>, std::string, SynchronizerStreamImpl<Messages...>>;
  using Self = SynchronizerStream<Messages...>;
  SynchronizerStream(NodeBookkeeping &node, uint32_t queue_size): Base(node) {
    this->impl()->create_mfl_synchronizer(node, queue_size);
    /// Note that even if this object is copied, this capture of the this-pointer is still valid
    /// because we only access impl in on_messages. Therefore, the referenced impl is always the
    /// same and (since it is ref-counted) always valid.
    this->impl()->synchronizer_->registerCallback(&Self::on_messages, this);
  }

private:
  void on_messages(typename Messages::SharedPtr... msgs) {
    this->impl()->put_value(std::forward_as_tuple(msgs...));
  }
};

template <class Message>
struct TF2MessageFilterImpl {
  SimpleFilterAdapter<Message> input_adapter;
  std::shared_ptr<tf2_ros::MessageFilter<Message>> filter;
};

/// Wrapper for the tf2_ros::MessageFilter.
template <class _Message>
struct TF2MessageFilter
    : public Stream<typename _Message::SharedPtr, std::string, TF2MessageFilterImpl<_Message>> {
  using Base = Stream<typename _Message::SharedPtr, std::string, TF2MessageFilterImpl<_Message>>;
  using Self = TF2MessageFilter<_Message>;
  using Message = _Message;

  TF2MessageFilter(NodeBookkeeping &node, const std::string &target_frame): Base(node) {
    this->impl()->name = "tf_filter";
    this->impl()->filter = std::make_shared<tf2_ros::MessageFilter<Message>>(
        this->impl()->input_adapter, *node.get_tf_buffer(), target_frame, 10,
        node.get_node_logging_interface(), node.get_node_clock_interface());
    this->impl()->filter->registerCallback(&Self::on_message, this);
  }
  void on_message(const typename _Message::SharedPtr &msg) { this->impl()->put_value(msg); }
};

/// The context owns the streams and is what is returned when calling `node->icey()`
/// (NodeWithIceyContext::icey).
class Context : public NodeBookkeeping, public std::enable_shared_from_this<Context>, private boost::noncopyable {
public:
  /// Construct a context using the given Node interface, initializing the `node` field. The
  /// interface must always be present because the Context creates new Streams and Streams need a
  /// the node for construction.
  explicit Context(const NodeInterfaces &node_interface)
      : NodeBookkeeping(node_interface) {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_interface.get_node_base_interface());
  }

  /// Creates a new stream of type S by passing the args to the constructor. It adds the impl to the
  /// list of streams so that it does not go out of scope. It also sets the context.
  template <AnyStream S, class... Args>
  S create_stream(Args &&...args) {
    S stream(static_cast<NodeBookkeeping&>(*this), std::forward<Args>(args)...);
    /// Track (i.e. reference) the Stream impl so that it does not go out of scope.
    stream.impl()->context = this->shared_from_this();
    return stream;
  }

  /// Declares a single parameter to ROS and register for updates. The ParameterDescriptor is
  /// created automatically matching the given Validator.
  /// \sa For more detailed documentation: [rclcpp::Node::declare_parameter](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4I0EN6rclcpp4Node17declare_parameterEDaRKNSt6stringERK10ParameterTRKN14rcl_interfaces3msg19ParameterDescriptorEb)
  template <class ParameterT>
  ParameterStream<ParameterT> declare_parameter(
      const std::string &parameter_name, const ParameterT &default_value = {},
      const Validator<ParameterT> &validator = {}, std::string description = "",
      bool read_only = false, bool ignore_override = false) {
    return create_stream<ParameterStream<ParameterT>>(
        parameter_name, default_value, validator, description, read_only, ignore_override);
  }

  /*!
  \brief Declare a given parameter struct to ROS.
  \tparam ParameterStruct the type of the parameter struct. It must be a struct/class with fields of
  either a primitive type supported by ROS (e.g. `double`) or a `icey::ParameterStream`, or another
  (nested) struct with more such fields.

  \param params The instance of the parameter struct where the values will be written to.
  \param notify_callback The callback that gets called when any field changes
  \param name_prefix Prefix for each parameter. Used by the recursive call to support nested
  structs. \note The passed object `params` must have the same lifetime as the node, best is to
  store it as a member of the node class.

  Example usage:
  \verbatim
    /// Here you declare in a single struct all parameters of the node:
    struct NodeParameters {
      /// We can have regular fields :
      double amplitude{3};

      /// And as well parameters with constraints and a description:
      icey::ParameterStream<double> frequency{10., icey::Interval(0., 25.),
                                          std::string("The frequency of the sine")};

      icey::ParameterStream<std::string> mode{"single", icey::Set<std::string>({"single", "double",
  "pulse"})};

      /// We can also have nested structs with more parameters, they will be named others.max_amp,
  others.cov: struct OtherParams { double max_amp = 6.; std::vector<double> cov; } others;

    };

    auto node = icey::create_node(argc, argv, "node_with_many_parameters");
    // Now create an object of the node-parameters that will be updated:
    NodeParameters params;

    /// Now simply declare the parameter struct and a callback that is called when any field
  updates: node->icey().declare_parameter_struct(params, [&](const std::string &changed_parameter) {
          RCLCPP_INFO_STREAM(node->get_logger(),
                            "Parameter " << changed_parameter << " changed");
        });
  \endverbatim
  */
  template <class ParameterStruct>
  void declare_parameter_struct(
      ParameterStruct &params, const std::function<void(const std::string &)> &notify_callback = {},
      std::string name_prefix = "") {
    field_reflection::for_each_field(params, [this, notify_callback, name_prefix](
                                                 std::string_view field_name, auto &field_value) {
      using Field = std::remove_reference_t<decltype(field_value)>;
      std::string field_name_r = name_prefix + std::string(field_name);
      if constexpr (is_stream<Field>) {

        static_cast<NodeBookkeeping&>(*this).stream_impls_.push_back(field_value.impl_);
        field_value.impl()->context = this->shared_from_this(); /// First, give it the missing context

        field_value.parameter_name = field_name_r;
        if (notify_callback) {
          field_value.impl()->register_handler(
              [field_name_r, notify_callback](const auto &) {
                notify_callback(field_name_r);
              });
        }
        field_value.register_with_ros(*this);
      } else if constexpr (is_valid_ros_param_type<Field>::value) {
        this->declare_parameter<Field>(field_name_r, field_value)
            .impl()
            ->register_handler(
                [&field_value, field_name_r, notify_callback](const auto &new_state) {
                  field_value = new_state.value();
                  if (notify_callback) {
                    notify_callback(field_name_r);
                  }
                });
      } else if constexpr (std::is_aggregate_v<Field>) {
        /// Else recurse for supporting grouped params
        declare_parameter_struct(field_value, notify_callback, field_name_r + ".");
      } else {
        /// static_assert(false) would always trigger, that is why we use this workaround, see
        /// https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2022/p2593r0.html
        static_assert(
            std::is_array_v<int>,
            "Every field of the parameters struct must be of type T or icey::ParameterStream<T> or "
            "a "
            "struct of such, where T is a valid ROS param type (see rcl_interfaces/ParameterType)");
      }
    });
  }

  template <class MessageT>
  SubscriptionStream<MessageT> create_subscription(
      const std::string &name, const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
      const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions()) {
    return create_stream<SubscriptionStream<MessageT>>(name, qos, options);
  }

  /// Create a subscriber that subscribes to a single transform between two frames.
  TransformSubscriptionStream create_transform_subscription(ValueOrParameter<std::string> target_frame,
                                                            ValueOrParameter<std::string> source_frame) {
    return create_stream<TransformSubscriptionStream>(target_frame, source_frame);
  }

  template <class Message>
  PublisherStream<Message> create_publisher(const std::string &topic_name,
                                            const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
                                            const rclcpp::PublisherOptions publisher_options = {}) {
    return create_stream<PublisherStream<Message>>(topic_name, qos, publisher_options);
  }

  template <AnyStream Input>
  PublisherStream<ValueOf<Input>> create_publisher(
      Input input, const std::string &topic_name,
      const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
      const rclcpp::PublisherOptions publisher_options = {}) {
    using Message = ValueOf<Input>;
    return create_stream<PublisherStream<Message>>(topic_name, qos, publisher_options, &input);
  }

  template <AnyStream Input>
  TransformPublisherStream create_transform_publisher(Input input) {
    auto output = create_stream<TransformPublisherStream>();
    input.connect_values(output);
    return output;
  }

  TimerStream create_timer(const Duration &interval, bool is_one_off_timer = false) {
    return create_stream<TimerStream>(interval, is_one_off_timer);
  }

  template <class ServiceT>
  ServiceStream<ServiceT> create_service(const std::string &service_name,
                                         ServiceStream<ServiceT>::SyncCallback sync_callback = {},
                                         const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    return create_stream<ServiceStream<ServiceT>>(service_name, sync_callback, qos);
  }

  /// Add a service client
  template <class ServiceT>
  ServiceClient<ServiceT> create_client(const std::string &service_name, const Duration &timeout,
                                        const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    return create_stream<ServiceClient<ServiceT>>(service_name, timeout, qos);
  }

  /// Add a service client and connect it to the input
  template <class ServiceT, AnyStream Input>
  ServiceClient<ServiceT> create_client(Input input, const std::string &service_name,
                                        const Duration &timeout,
                                        const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    static_assert(std::is_same_v<ValueOf<Input>, typename ServiceT::Request::SharedPtr>,
                  "The input triggering the service must hold a value of type Request::SharedPtr");
    auto service_client = create_client<ServiceT>(service_name, timeout, qos);
    input.then([service_client](auto req) { service_client.call(req); });
    /// Pass the error since service calls are chainable
    if constexpr (!std::is_same_v<ErrorOf<Input>, Nothing>) {
      input.except([service_client](auto err) { service_client.impl()->put_error(err); });
    }
    return service_client;
  }

  /*!
    Synchronizer that given a reference signal at its first argument, ouputs all the other topics
    \warning Errors are currently not passed through
  */
  template <ErrorFreeStream Reference, AnyStream... Interpolatables>
  static auto sync_with_reference(Reference reference, Interpolatables... interpolatables) {
    using namespace message_filters::message_traits;
    using RefMsg = MessageOf<Reference>;
    static_assert(HasHeader<RefMsg>::value,
                  "The ROS message type must have a header with the timestamp to be synchronized");
    auto interpolatables_tuple = std::make_tuple(interpolatables...);
    /// TOOD somehow does not work
    // auto all_are_interpolatables = hana::all_of(inputs_tuple,  [](auto t) { return
    // hana_is_interpolatable(t); }); static_assert(all_are_interpolatables, "All inputs must be
    // interpolatable when using the sync_with_reference");
    return reference.then([interpolatables_tuple](const auto &new_value) {
      auto input_maybe_values = hana::transform(interpolatables_tuple, [&](auto input) {
        return input.get_at_time(rclcpp::Time(new_value->header.stamp));
      });
      return hana::prepend(input_maybe_values, new_value);
    });
  }

  /// Synchronizer that synchronizes streams by approximately matching the header time-stamps (using
  /// the synchronizer from the `message_filters` package)
  ///
  /// \tparam Inputs the input stream types, not necessarily all the same
  /// \param inputs the input streams, not necessarily all of the same type
  /// \note The queue size is 100, it cannot be changed currently
  /// \warning Errors are currently not passed through
  template <ErrorFreeStream... Inputs>
  SynchronizerStream<MessageOf<Inputs>...> synchronize_approx_time(Inputs... inputs) {
    static_assert(sizeof...(Inputs) >= 2, "You need to synchronize at least two inputs.");
    using namespace hana::literals;
    const uint32_t queue_size = 100;
    auto synchronizer = create_stream<SynchronizerStream<MessageOf<Inputs>...>>(queue_size);
    auto zipped = hana::zip(std::forward_as_tuple(inputs...), synchronizer.impl()->inputs());
    hana::for_each(zipped, [](auto &input_output_tuple) {
      auto &input = input_output_tuple[0_c];
      auto &synchronizer_input = input_output_tuple[1_c];
      input.connect_values(synchronizer_input);
    });
    return synchronizer;
  }

  /// Synchronize a variable amount of Streams. Uses a Approx-Time synchronizer (i.e. calls
  /// synchronize_approx_time) if the inputs are not interpolatable or an interpolation-based
  /// synchronizer based on a given (non-interpolatable) reference. Or, a combination of both, this
  /// is decided at compile-time.
  template <AnyStream... Inputs>
  auto synchronize(Inputs... inputs) {
    static_assert(sizeof...(Inputs) >= 2,
                  "You need to have at least two inputs for synchronization.");
    auto inputs_tuple = std::make_tuple(inputs...);
    auto interpolatables =
        hana::remove_if(inputs_tuple, [](auto t) { return not hana_is_interpolatable(t); });
    auto non_interpolatables =
        hana::remove_if(inputs_tuple, [](auto t) { return hana_is_interpolatable(t); });
    constexpr int num_interpolatables = hana::length(interpolatables);
    constexpr int num_non_interpolatables = hana::length(non_interpolatables);
    static_assert(num_interpolatables <= 0 || num_non_interpolatables != 0,
                  "You are trying to synchronize only interpolatable signals. This does not work, "
                  "you need to "
                  "have at least one non-interpolatable signal that is the common time for all the "
                  "interpolatable signals.");
    // This can only either be one or more than one. Precondition is that we synchronize at least
    // two entities. Given the condition above, the statement follows.
    if constexpr (num_non_interpolatables > 1) {
      /// We need the ApproxTime
      auto approx_time_output = hana::unpack(
          non_interpolatables, [](auto... inputs) { return synchronize_approx_time(inputs...); });
      if constexpr (num_interpolatables > 1) {
        /// We have interpolatables and non-interpolatables, so we need to use both synchronizers
        return hana::unpack(
            hana::prepend(interpolatables, approx_time_output),
            [](auto ref, auto... ints) { return sync_with_reference(ref, ints...); });
      } else {
        return approx_time_output;
      }
    } else {
      // Otherwise, we only need a sync with reference
      return hana::unpack(hana::prepend(interpolatables, std::get<0>(non_interpolatables)),
                          [](auto ref, auto... ints) { return sync_with_reference(ref, ints...); });
    }
  }

  /*!
    Outputs the value or error of any of the inputs. All the inputs must have the same Value and
    ErrorValue type.
  */
  template <AnyStream... Inputs>
  auto any(Inputs... inputs) {
    // assert_all_stream_values_are_same<Inputs...>();
    using Input = decltype(std::get<0>(std::forward_as_tuple(inputs...)));
    using InputValue = typename std::remove_reference_t<Input>::Value;
    using InputError = typename std::remove_reference_t<Input>::ErrorValue;
    /// First, create a new stream
    auto output = create_stream<Stream<InputValue, InputError>>();
    /// Now connect each input with the output
    hana::for_each(std::forward_as_tuple(inputs...),
                   [output](auto &input) { input.connect_values(output); });
    return output;
  }

  /// Synchronizes a input stream with a transform: The Streams outputs the input value when the
  /// transform between it's header frame and the target_frame becomes available. It uses for this
  /// the `tf2_ros::MessageFilter`
  template <ErrorFreeStream Input>
  TF2MessageFilter<MessageOf<Input>> synchronize_with_transform(Input input,
                                                                const std::string &target_frame) {
    auto output = create_stream<TF2MessageFilter<MessageOf<Input>>>(target_frame);
    input.connect_values(output);
    return output;
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> &get_executor() { return executor_; }

  /// Spins the ROS executor until the Stream has something (value or error). This is also called a
  /// synchronous wait.
  template <AnyStream Input>
  void spin_executor_until_stream_has_some(Input stream) {
    while (rclcpp::ok() && stream.impl()->has_none()) {
      get_executor()->spin_once();
    }
    if(stream.impl()->has_none()) {
      /// In case rclcpp::ok() returned false, Ctrl+C was pressed while waiting with co_await stream, just terminate the ROS, this is what we would do in a normal ROS node anyway. 
      /// We handle this case here because the Stream does not have a value and we do not want to force the user to do unwrapping only to handle the 
      /// 0.1% percent case that Ctrl+C.
      std::cout << "Exiting Node after ok is false ..." << std::endl;
      rclcpp::shutdown();
      std::exit(0);
    }
  }

  /// Spins the ROS executor until the Stream has something (value or error) or the timeout occurs.
  /// This is also called a synchronous wait. \param timeout The maximum duration to wait. If no
  /// timeout is desired, please use the other overload of this function.
  template <AnyStream Input>
  void spin_executor_until_stream_has_some(Input stream, const Duration &timeout) {
    const auto start = Clock::now();
    while (rclcpp::ok() && stream.impl()->has_none() && (Clock::now() - start) < timeout) {
      get_executor()->spin_once(timeout);
    }
  }

protected:
  /// The executor is needed for async/await because the Streams need to be able to await
  /// themselves. For this, they acces the ROS executor through the Context.
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  
};

/// The ROS node, additionally owning the context that holds the Streams.
/// \tparam NodeType can either be rclcpp::Node or rclcpp_lifecycle::LifecycleNode
template <class NodeType>
class NodeWithIceyContext : public NodeType {
public:
  /// Constructs a new new node and initializes the ICEY context.
  NodeWithIceyContext(std::string node_name,
                      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : NodeType(node_name, node_options) {
    /// Note that here we cannot call shared_from_this() since it requires the object to be already
    /// constructed.
    this->icey_context_ = std::make_shared<Context>(NodeInterfaces(this));
  }

  /// Returns the context that is needed to create ICEY streams
  Context &icey() { return *this->icey_context_; }

protected:
  std::shared_ptr<Context> icey_context_;
};

/// The node type that you will use instead of an `rclcpp::Node`. It derives from the
/// `rclcpp::Node`, so that you can do everything that you can also with an `rclcpp::Node`. See
/// `NodeWithIceyContext` for details.
using Node = NodeWithIceyContext<rclcpp::Node>;
/// The node type that you will use instead of an `rclcpp_lifecycle::LifecycleNode`. It derives from
/// the `rclcpp_lifecycle::LifecycleNode`, so that you can do everything that you can also with an
/// `rclcpp_lifecycle::LifecycleNode`. See `NodeWithIceyContext` for details.
using LifecycleNode = NodeWithIceyContext<rclcpp_lifecycle::LifecycleNode>;

/// Start spinning either a Node or a LifeCycleNode. Calls `rclcpp::shutdown()` at the end so you do
/// not have to do it.
template <class NodeType>
static void spin(NodeType node) {
  /// We use single-threaded executor because the MT one can starve due to a bug
  rclcpp::executors::SingleThreadedExecutor executor;
  if (node->icey().get_executor()) {
    node->icey().get_executor()->remove_node(node->get_node_base_interface());
    node->icey().get_executor().reset();
  }
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
}

static void spin_nodes(const std::vector<std::shared_ptr<Node>> &nodes) {
  rclcpp::executors::SingleThreadedExecutor executor;
  /// This is how nodes should be composed according to ROS guru wjwwood:
  /// https://robotics.stackexchange.com/a/89767. He references
  /// https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp
  for (auto &node : nodes) {
    if (node->icey().get_executor()) {
      node->icey().get_executor()->remove_node(node->get_node_base_interface());
      node->icey().get_executor().reset();
    }
    executor.add_node(node->get_node_base_interface());
  }
  executor.spin();
  rclcpp::shutdown();
}

/// Creates a node by simply calling `std::make_shared`, but it additionally calls `rclcpp::init` if
/// not done already, so that you don't have to do it.
template <class NodeType = Node>
static auto create_node(int argc, char **argv, const std::string &node_name) {
  if (!rclcpp::contexts::get_global_default_context()
           ->is_valid())  /// Create a context if it is the first spawn
    rclcpp::init(argc, argv);
  return std::make_shared<NodeType>(node_name);
}

}  // namespace icey
