#pragma once

#include <any>
#include <boost/hana.hpp>
#include <boost/hana/ext/std/tuple.hpp>  /// Needed so that we do not need the custom hana tuples everywhere: https://stackoverflow.com/a/34318002
#include <boost/type_index.hpp>
#include <functional>
#include <icey/impl/bag_of_metaprogramming_tricks.hpp>
#include <icey/impl/observable.hpp>
#include <iostream>
#include <map>
#include <optional>
#include <tuple>
#include <unordered_map>

/// TF2 support:
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

// Message filters library: (.h so that this works with humble as well)
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
/// Support for lifecycle nodes:
#include <coroutine>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace icey {

namespace hana = boost::hana;
bool icey_debug_print = false;

using Clock = std::chrono::system_clock;
using Time = std::chrono::time_point<Clock>;
using Duration = Clock::duration;

/// A helper to abstract regular rclrpp::Nodes and LifecycleNodes.
/// Similar to the NodeInterfaces class: https://github.com/ros2/rclcpp/pull/2041
/// but it's not coming for Humble. So I did something similar.
/// Mind also that NodeInterfaces is not yet supported by geometry2/TF:
/// https://github.com/ros2/geometry2/issues/698
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
    if constexpr (std::is_same_v<_Node, rclcpp_lifecycle::LifecycleNode>)
      maybe_lifecycle_node = node;
    else
      maybe_regular_node = node;
  }
  /// This getter is needed for ParameterEventHandler. Other functions likely require this interface
  /// too.
  auto get_node_base_interface() { return node_base_; }
  auto get_node_graph_interface() { return node_graph_; }
  auto get_node_clock_interface() { return node_clock_; }
  auto get_node_logging_interface() { return node_logging_; }
  auto get_node_timers_interface() { return node_timers_; }
  auto get_node_topics_interface() { return node_topics_; }
  auto get_node_services_interface() { return node_services_; }
  auto get_node_parameters_interface() { return node_parameters_; }
  auto get_node_time_source_interface() { return node_time_source_; }
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr node_time_source_;
  // rclcpp::node_interfaces::NodeTypeDescriptionsInterface::SharedPtr node_type_descriptions_;
  // rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_;
  /// This is set to either of the two, depending on which node we got.
  /// It has to be a raw pointer since this nodeInterface is needed during node construction
  rclcpp::Node *maybe_regular_node{nullptr};
  rclcpp_lifecycle::LifecycleNode *maybe_lifecycle_node{nullptr};
};

/// A transform listener that allows to subscribe on a single transform between two coordinate
/// systems. It is implemented similarly to the tf2_ros::TransformListener, but without a separate
/// node. The implementation currently checks for relevant transforms (i.e. ones we subscribed)
/// every time a new message is receved on /tf.
/// TODO: To speed up the code a bit (not sure if significantly), we could obtain the path in the TF
/// tree between the source- and target-frame using tf2::BufferCore::_chainAsVector and then only
/// react if any transform was received that is part of this path.
struct TFListener {
  using TransformMsg = geometry_msgs::msg::TransformStamped;
  using OnTransform = std::function<void(const TransformMsg &)>;
  using OnError = std::function<void(const tf2::TransformException &)>;

  explicit TFListener(const NodeInterfaces &node) : node_(node) { init(); }

  /// Add notification for a single transform.
  void add_subscription(std::string target_frame, std::string source_frame,
                        const OnTransform &on_transform, const OnError &on_error) {
    subscribed_transforms_.emplace_back(target_frame, source_frame, on_transform, on_error);
  }

  NodeInterfaces node_;
  /// We take a tf2_ros::Buffer instead of a tf2::BufferImpl only to be able to use ROS-time API
  /// (internally TF2 has it's own timestamps...), not because we need to wait on anything (that's
  /// what tf2_ros::Buffer does in addition to tf2::BufferImpl).
  std::shared_ptr<tf2_ros::Buffer> buffer_;

private:
  using TransformsMsg = tf2_msgs::msg::TFMessage::ConstSharedPtr;

  struct TFSubscriptionInfo {
    TFSubscriptionInfo(std::string target_frame, std::string source_frame,
                       const OnTransform &on_transform, const OnError &on_error)
        : target_frame(target_frame),
          source_frame(source_frame),
          on_transform(on_transform),
          on_error(on_error) {}
    std::string target_frame;
    std::string source_frame;
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
    for (size_t i = 0u; i < msg_in.transforms.size(); i++) {
      try {
        buffer_->setTransform(msg_in.transforms[i], authority, is_static);
      } catch (const tf2::TransformException &ex) {
        // /\todo Use error reporting
        std::string temp = ex.what();
        RCLCPP_ERROR(node_.node_logging_->get_logger(),
                     "Failure to set received transform from %s to %s with error: %s\n",
                     msg_in.transforms[i].child_frame_id.c_str(),
                     msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
      }
    }
  }

  /// This simply looks up the transform in the buffer at the latest stamp and checks if it
  /// changed with respect to the previously received one. If the transform has changed, we know
  /// we have to notify.
  void maybe_notify(TFSubscriptionInfo &info) {
    try {
      /// Lookup the latest transform in the buffer to see if we got something new in the buffer.
      /// Note that this does not wait/thread-sleep etc. This is simply a lookup in a
      /// std::vector/tree.
      geometry_msgs::msg::TransformStamped tf_msg =
          buffer_->lookupTransform(info.target_frame, info.source_frame, tf2::TimePointZero);
      /// Now check if it is the same as the last one, in this case we return nothing since the
      /// transform did not change. (Instead, we received on /tf some other, unrelated
      /// transforms.)
      if (!info.last_received_transform || tf_msg != *info.last_received_transform) {
        info.last_received_transform = tf_msg;
        info.on_transform(tf_msg);
      }
    } catch (const tf2::TransformException &e) {
      /// Simply ignore. Because we are requesting the latest transform in the buffer, the only
      /// exception we can get is that there is no transform available yet.
      info.on_error(e);
    }
  }

  void notify_if_any_relevant_transform_was_received() {
    for (auto &tf_info : subscribed_transforms_) {
      maybe_notify(tf_info);
    }
  }

  void on_tf_message(TransformsMsg msg, bool is_static) {
    store_in_buffer(*msg, is_static);
    notify_if_any_relevant_transform_was_received();
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_;
  std::vector<TFSubscriptionInfo> subscribed_transforms_;
};

/// A level 1 API wrap, implementing the lower level boilerplate-code and the bookkeeping
class NodeBookkeeping {
public:
  /// Do not force the user to do the bookkeeping themselves: Do it instead automatically
  struct IceyBook {
    std::unordered_map<std::string, std::pair<std::shared_ptr<rclcpp::ParameterEventHandler>,
                                              std::shared_ptr<rclcpp::ParameterCallbackHandle>>>
        parameters_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscribers_;
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
    std::unordered_map<std::string, rclcpp::ServiceBase::SharedPtr> services_;
    std::unordered_map<std::string, rclcpp::ClientBase::SharedPtr> services_clients_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
    std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;

    /// TF Support
    std::shared_ptr<TFListener> tf2_listener_;
    /// This is a simple wrapper around a publisher, there is really nothing intereseting under the
    /// hood of tf2_ros::TransformBroadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    /// Some extra baggage we can have for the functional API as an extention point
    std::unordered_map<std::string, std::any> extra_baggage_;
  };

  explicit NodeBookkeeping(const NodeInterfaces &node) : node_(node) {}

  NodeInterfaces node_;
  IceyBook book_;
  /// The internal rclcpp::Node does not consistently call the free function (i.e. rclcpp::create_*)
  /// but instead prepends the sub_namespace. (on humble) This seems to me like a patch, so we have
  /// to apply it here as well. This is unfortunate, but needed to support both lifecycle nodes and
  /// regular nodes without templates.
  inline std::string extend_name_with_sub_namespace(const std::string &name,
                                                    const std::string &sub_namespace) {
    std::string name_with_sub_namespace(name);
    if (sub_namespace != "" && name.front() != '/' && name.front() != '~') {
      name_with_sub_namespace = sub_namespace + "/" + name;
    }
    return name_with_sub_namespace;
  }

  template <class ParameterT, class CallbackT>
  auto add_parameter(const std::string &name, const std::optional<ParameterT> &default_value,
                     CallbackT &&update_callback,
                     const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
                         rcl_interfaces::msg::ParameterDescriptor(),
                     bool ignore_override = false) {
    rclcpp::ParameterValue v =
        default_value ? rclcpp::ParameterValue(*default_value) : rclcpp::ParameterValue();
    auto param =
        node_.node_parameters_->declare_parameter(name, v, parameter_descriptor, ignore_override);
    auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(node_);
    auto cb_handle = param_subscriber->add_parameter_callback(name, std::move(update_callback));
    book_.parameters_.emplace(name, std::make_pair(param_subscriber, cb_handle));
    return param;
  }

  template <class Msg, class F>
  void add_subscription(const std::string &topic, F &&cb, const rclcpp::QoS &qos,
                        const rclcpp::SubscriptionOptions &options) {
    /// TODO extend_name_with_sub_namespace(topic_name, this->get_sub_namespace())
    book_.subscribers_[topic] =
        rclcpp::create_subscription<Msg>(node_.node_topics_, topic, qos, cb, options);
  }

  template <class Msg>
  auto add_publisher(const std::string &topic, const rclcpp::QoS &qos) {
    auto publisher = rclcpp::create_publisher<Msg>(node_.node_topics_, topic, qos);
    book_.publishers_.emplace(topic, publisher);
    return publisher;
  }

  template <class CallbackT>
  auto add_timer(const Duration &time_interval, bool use_wall_time, CallbackT &&callback,
                 rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    /// TODO no normal timer in humble, also no autostart flag was present in Humble
    rclcpp::TimerBase::SharedPtr timer =
        rclcpp::create_wall_timer(time_interval, std::move(callback), group, node_.node_base_.get(),
                                  node_.node_timers_.get());
    book_.timers_.push_back(timer);
    return timer;
  }

  template <class ServiceT, class CallbackT>
  void add_service(const std::string &service_name, CallbackT &&callback,
                   const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                   rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    auto service =
        rclcpp::create_service<ServiceT>(node_.node_base_, node_.node_services_, service_name,
                                         std::move(callback), qos.get_rmw_qos_profile(), group);
    book_.services_.emplace(service_name, service);
  }

  template <class Service>
  auto add_client(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                  rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    /// TODO extend_name_with_sub_namespace(service_name, this->get_sub_namespace()),
    auto client = rclcpp::create_client<Service>(
        node_.node_base_, node_.node_graph_, node_.node_services_, service_name,
        rclcpp::ServicesQoS().get_rmw_qos_profile(), group);
    book_.services_clients_.emplace(service_name, client);
    return client;
  }

  /// Subscribe to a transform on tf between two frames
  template <class OnTransform, class OnError>
  auto add_tf_subscription(std::string target_frame, std::string source_frame,
                           OnTransform &&on_transform, OnError &&on_error) {
    add_tf_listener_if_needed();
    book_.tf2_listener_->add_subscription(target_frame, source_frame, on_transform, on_error);
    return book_.tf2_listener_;
  }

  auto add_tf_broadcaster_if_needed() {
    if (!book_.tf_broadcaster_)
      book_.tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_.node_topics_);
    return book_.tf_broadcaster_;
  }

private:
  void add_tf_listener_if_needed() {
    if (book_.tf2_listener_)  /// We need only one subscription on /tf, but can have multiple
                              /// transforms on which we listen
      return;
    book_.tf2_listener_ = std::make_shared<TFListener>(node_);
  }
};

/// Everything that can be "attached" to a ROS node, publishers, subscribers, clients, TF subscriber
/// etc.
class NodeAttachable {
public:
  void attach_to_node(NodeBookkeeping &node_handle) {
    if (this->was_attached_) throw std::invalid_argument("NodeAttachable was already attached");
    // if (icey_debug_print)
    // std::cout << "Attaching " << class_name << ", " << name << " ..." << std::endl;
    attach_(node_handle);
    was_attached_ = true;
  }

  /// Needed to initialize parameters first
  bool is_parameter() const { return is_parameter_; }
  bool is_parameter_{false};
  bool was_attached_{false};
  std::function<void(NodeBookkeeping &)> attach_;
};

struct ObservableTag {};  /// A tag to be able to recognize the type "Observeble" using traits
template <typename... Args>
struct observable_traits {
  /*static_assert(
      (is_shared_ptr<Args> && ...),
      "The arguments must be a shared_ptr< icey::Observable >, but it is not a shared_ptr");*/
  static_assert((std::is_base_of_v<ObservableTag, remove_shared_ptr_t<Args>> && ...),
                "The arguments must be an icey::Observable");
};

template <class T>
constexpr void assert_observable_holds_tuple() {
  static_assert(is_tuple_v<obs_msg<T>>,
                "The Observable must hold a tuple as a value for unpacking.");
}

// Assert that all Observables types hold the same value
template <typename First, typename... Rest>
constexpr void assert_all_observable_values_are_same() {
  observable_traits<First, Rest...>{};  /// Only Observables are expected to have ::Value
  // Static assert that each T::Value is the same as First::Value
  static_assert((std::is_same_v<obs_msg<First>, obs_msg<Rest>> && ...),
                "The values of all the observables must be the same");
}

/// Adds some things we want to put in inside the impl::Observable by default.
/// Handy to not force the user to declare this, i.e. to not leak implementation details
template <class Derived>
struct DerivedWithDefaults : public Derived, public NodeAttachable {};

/// An observable. Similar to a promise in JavaScript.
template <typename _Value, typename _ErrorValue = Nothing, typename Derived = NodeAttachable>
class Observable : public ObservableTag {
public:
  using Impl = impl::Observable<_Value, _ErrorValue, Derived>;
  using Value = typename Impl::Value;
  using MaybeValue = typename Impl::MaybeValue;
  using ErrorValue = typename Impl::ErrorValue;
  using Self = Observable<_Value, _ErrorValue, Derived>;  // DerivedWithDefaults<Derived>

  auto impl() const { return impl_; }

  void assert_we_have_context() {  // Cpp is great, but Java still has a NullPtrException more...
    if (!this->impl()->context.lock())
      throw std::runtime_error(
          "This observable does not have context, we cannot do stuff with it that depends on the "
          "context.");
  }

  /// Creates a new Observable that changes it's value to y every time the value x of the parent
  /// observable changes, where y = f(x).
  template <typename F>
  auto then(F &&f) {
    auto child = Self::create_from_impl(impl()->then(f));
    child.impl()->context = this->impl()->context;
    return child;
  }

  template <typename F>
  auto except(F &&f) {
    static_assert(not std::is_same_v<ErrorValue, Nothing>,
                  "This observable cannot have errors, so you cannot register except() on it.");
    auto child = Self::create_from_impl(impl()->except(f));
    child.impl()->context = this->impl()->context;
    return child;
  }

  /// Create a ROS publisher by creating a new observable of type T and connecting it to this
  /// observable.
  template <class T, typename... Args>
  void publish(Args &&...args) {
    assert_we_have_context();
    static_assert(
        not std::is_same_v<Value, Nothing>,
        "This observable does not have a value, there is nothing to publish, so you cannot "
        "call publish() on it.");
    /// We create this through the context to register it for attachment to the ROS node
    auto child = this->impl()->context.lock()->template create_observable<T>(args...);
    this->impl()->then([child](const auto &x) { child.impl()->resolve(x); });
  }

  /// Create a ROS normal publisher.
  void publish(const std::string &topic_name,
               const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS()) {
    assert_we_have_context();
    static_assert(
        not std::is_same_v<Value, Nothing>,
        "This observable does not have a value, there is nothing to publish, so you cannot "
        "call publish() on it.");
    return this->impl()->context.lock()->create_publisher(*this, topic_name, qos);
  }

  void publish_transform() {
    assert_we_have_context();
    static_assert(std::is_same_v<obs_msg<Self>, geometry_msgs::msg::TransformStamped>,
                  "The observable must hold a Value of type "
                  "geometry_msgs::msg::TransformStamped[::SharedPtr] to be able to call "
                  "publish_transform() on it.");
    return this->impl()->context.lock()->create_transform_publisher(*this);
  }

  /// Create a new ServiceClient observable, this observable holds the request.
  template <class ServiceT>
  auto call_service(const std::string &service_name, const Duration &timeout,
                    const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    assert_we_have_context();
    static_assert(not std::is_same_v<Value, Nothing>,
                  "This observable does not have a value, there is nothing to publish, you cannot "
                  "call publish() on it.");
    return this->impl()->context.lock()->template create_client<ServiceT>(*this, service_name,
                                                                          timeout, qos);
  }

  /// Unpacks an Observable holding a tuple as value to multiple Observables for each tuple element.
  auto unpack() {
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This observable does not have a value, there is nothing to unpack().");
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This observable does not have a value, there is nothing to unpack().");
    // TODO assert_observable_holds_tuple<Parent>();
    constexpr size_t tuple_sz = std::tuple_size_v<obs_val<Self>>;
    /// hana::to<> is needed to make a sequence from a range, otherwise we cannot transform it, see
    /// https://stackoverflow.com/a/33181700
    constexpr auto indices = hana::to<hana::tuple_tag>(hana::range_c<std::size_t, 0, tuple_sz>);
    auto hana_tuple_output = hana::transform(indices, [impl = this->impl()](auto I) {
      return impl->then([I](const auto &...args) {  /// Need to take variadic because then()
                                                    /// automatically unpacks tuples
        return std::get<I>(std::forward_as_tuple(
            args...));  /// So we need to pack this again in a tuple and get the index.
      });
    });
    /// Now create a std::tuple from the hana::tuple to be able to use structured bindings
    /// TODO perfect FW
    return hana::unpack(hana_tuple_output, [](auto... args) { return std::make_tuple(args...); });
  }

  

  ///// Everything that follows is to satisfy the interface for C++20's coroutines.

  using promise_type = Self;  /// We are a promise
  Self get_return_object() { return *this; }
  /// We never already got something, we always first need to spin the ROS executor to get a message
  std::suspend_never initial_suspend() { return {}; }
  Value return_value() { return this->impl()->value(); }
  template <class ReturnType>
  ReturnType return_value(ReturnType x) {
    return x;
  }
  void unhandled_exception() {}
  std::suspend_always final_suspend() noexcept {
    std::cout << "final_suspend  " << std::endl;
    return {};
  }

  /// Promisify a value if needed, meaning if it is not already a promise
  template <class ReturnType>
  auto await_transform(ReturnType x) {
    if constexpr (std::is_base_of_v<ObservableTag, ReturnType>)
      return x;
    else
      return Observable<ReturnType, Nothing>{};
  }

  /// Run the ROS executor untils this Promise is fulfilled. This function can be called normally
  auto await() {
    spin_executor();
    return await_resume();
  }

  /// Spin the event loop, (the ROS executor) until this Promise is fulfilled
  void spin_executor() {
    auto promise = this->get_promise();
    while (!(promise->has_value() || promise->has_error())) {
      /// Note that spinning once might not be enough, for example if we synchronize three topics
      /// and await the synchronizer output, we would need to spin at least three times.
      this->impl()->context.lock()->executor_->spin_once();
    }
  }

  //// Now the Awaiter interface for C++20 coroutines:
  bool await_ready() { return !this->get_promise()->has_none(); }
  bool await_suspend(auto coroutine_handle) {
    this->spin_executor();
    return false;  /// Resume the current coroutine, see
                   /// https://en.cppreference.com/w/cpp/language/coroutines
  }
  auto await_resume() {
    auto promise = this->get_promise();
    /// Return the value if there can be no error
    if constexpr (std::is_same_v<ErrorValue, Nothing>) {
      auto result = promise->value();
      /// Reset the state since we consumed this value
      promise->set_none();
      return result;
    } else {
      auto result = promise->get_state();
      /// Reset the state since we consumed this value
      promise->set_none();
      return result;
    }
  }
  ///////////////////// End coroutine support interface

  // protected:
  /// Pattern-maching factory function that creates a New Self with different value and error types
  /// based on the passed implementation pointer.
  template <class NewVal, class NewErr, class NewDerived>
  static Observable<NewVal, NewErr, NewDerived> create_from_impl(
      std::shared_ptr<impl::Observable<NewVal, NewErr, NewDerived>> impl) {
    Observable<NewVal, NewErr, NewDerived> new_obs;
    new_obs.impl_ = impl;
    return new_obs;
  }
  std::shared_ptr<Impl> impl_{impl::create_observable<Impl>()};
};

/// An observable for ROS parameters. Fires initially an event if a default_value set
template <typename _Value>
struct ParameterObservable : public Observable<_Value> {
  ParameterObservable(const std::string &parameter_name, const std::optional<_Value> &default_value,
                      const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
                          rcl_interfaces::msg::ParameterDescriptor(),
                      bool ignore_override = false) {
    this->impl()->is_parameter_ = true;
    this->impl()->name = parameter_name;
    this->impl()->attach_ = [impl = this->impl(), parameter_name, default_value, parameter_descriptor, ignore_override](NodeBookkeeping &node) {
      node.add_parameter<_Value>(
          parameter_name, default_value,
          [impl](const rclcpp::Parameter &new_param) {
            _Value new_value = new_param.get_value<_Value>();
            impl->resolve(new_value);
          },
          parameter_descriptor, ignore_override);
      /// Set default value if there is one
      if (default_value) {
        // Value initial_value;
        /// TODO I think I added this to crash in case no default is there, think regarding default
        /// values of params
        // node.get_parameter_or(parameter_name, initial_value, *default_value);
        impl->resolve(*default_value);
      }
    };
  }
  /// Parameters are initialized always at the beginning, so we can provide a getter for the value
  /// so that they can be used conveniently in callbacks.
  const _Value &value() const {
    if (!this->impl()->has_value()) {
      throw std::runtime_error(
          "Parameter '" + this->impl()->name +
          "' cannot be accessed before spawning the node. You can only access parameters "
          "inside callbacks (which are triggered after calling icey::spawn())");
    }
    return this->impl()->value();
  }
};

/// A subscriber observable, always stores a shared pointer to the message as it's value
template <typename _Message>
struct SubscriptionObservable : public Observable<typename _Message::SharedPtr> {
  using Value = typename _Message::SharedPtr;  /// Needed for synchronizer to determine message type
                                               /// TODO use base, upcast, then decltype or smth
  SubscriptionObservable(const std::string &topic_name, const rclcpp::QoS &qos,
                         const rclcpp::SubscriptionOptions &options) {
    this->impl()->name = topic_name;
    this->impl()->attach_ = [impl=this->impl(), topic_name, qos, options](NodeBookkeeping &node) {
      node.add_subscription<_Message>(
          topic_name,
          [impl](typename _Message::SharedPtr new_value) { impl->resolve(new_value); }, qos,
          options);
    };
  }
};

/// TODO rem RTTI, recognize differently, use concepts for iface def for example.
struct InterpolateableObservableTag {};
template <typename _Message, typename DerivedImpl>
struct InterpolateableObservable
    : public InterpolateableObservableTag,
      public Observable<typename _Message::SharedPtr, std::string, DerivedImpl> {
  using MaybeValue = std::optional<typename _Message::SharedPtr>;
  /// Get the measurement at a given time point. Returns nothing if the buffer is empty or
  /// an extrapolation would be required.
  virtual MaybeValue get_at_time(const rclcpp::Time &time) const = 0;
};

/// An interpolatable observable is one that buffers the incoming messages using a circular buffer
/// and allows to query the message at a given point, using interpolation.
template <typename T>
constexpr auto hana_is_interpolatable(T) {
  if constexpr (std::is_base_of_v<InterpolateableObservableTag, T>)
    return hana::bool_c<true>;
  else
    return hana::bool_c<false>;
}

/// A subscription for single transforms. It implements InterpolateableObservable but by using
/// lookupTransform, not an own buffer
struct TransformSubscriptionObservableImpl : public NodeAttachable {
  using Message = geometry_msgs::msg::TransformStamped;
  std::string target_frame_;
  std::string source_frame_;
  /// We allocate a single message that we share with the other observables when notifying them.
  /// Note that we cannot use the base value since it is needed for notifying, i.e. it is cleared
  std::shared_ptr<Message> shared_value_{std::make_shared<Message>()};
  /// We do not own the listener, the Book owns it
  std::weak_ptr<TFListener> tf2_listener_;
};
struct TransformSubscriptionObservable
    : public InterpolateableObservable<geometry_msgs::msg::TransformStamped,
                                       TransformSubscriptionObservableImpl> {
  using Message = geometry_msgs::msg::TransformStamped;
  using MaybeValue =
      InterpolateableObservable<Message, TransformSubscriptionObservableImpl>::MaybeValue;

  TransformSubscriptionObservable(const std::string &target_frame,
                                  const std::string &source_frame) {
    this->impl()->target_frame_ = target_frame;
    this->impl()->source_frame_ = source_frame;
    this->impl()->name = "source_frame: " + source_frame + ", target_frame: " + target_frame;
    this->impl()->attach_ = [impl = this->impl()](NodeBookkeeping &node) {
      impl->tf2_listener_ = node.add_tf_subscription(
          impl->target_frame_, impl->source_frame_,
          [impl](const geometry_msgs::msg::TransformStamped &new_value) {
            *impl->shared_value_ = new_value;
            impl->resolve(impl->shared_value_);
          },
          [impl](const tf2::TransformException &ex) { impl->reject(ex.what()); });
    };
  }
  MaybeValue get_at_time(const rclcpp::Time &time) const override {
    try {
      // Note that this call does not wait, the transform must already have arrived.
      *this->impl()->shared_value_ = this->impl()->tf2_listener_.lock()->buffer_->lookupTransform(
          this->impl()->target_frame_, this->impl()->source_frame_, time);
      return this->impl()->shared_value_;
    } catch (const tf2::TransformException &e) {
      this->impl()->reject(e.what());
      return {};
    }
  }
};

/// Timer signal, saves the number of ticks as the value and also passes the timerobject as well to
/// the callback
struct TimerImpl : public NodeAttachable {
  size_t ticks_counter{0};
  rclcpp::TimerBase::SharedPtr timer;
};
struct TimerObservable : public Observable<size_t, Nothing, TimerImpl> {
  TimerObservable(const Duration &interval, bool use_wall_time, bool is_one_off_timer) {
    this->impl()->name = "timer";
    this->impl()->attach_ = [impl = this->impl(), interval, use_wall_time,
                             is_one_off_timer](NodeBookkeeping &node) {
      impl->timer = node.add_timer(interval, use_wall_time, [impl, is_one_off_timer]() {
        impl->resolve(impl->ticks_counter);
        /// Needed as separate state as it might be resetted in async/await mode
        impl->ticks_counter++;
        if (is_one_off_timer) impl->timer->cancel();
      });
    };
  }
};

/// A publishabe state, read-only. Value can be either a Message or shared_ptr<Message>
template <typename _Value>
struct PublisherImpl : public NodeAttachable {
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
template <typename _Value>
struct PublisherObservable : public Observable<_Value, Nothing, PublisherImpl<_Value>> {
  using Message = remove_shared_ptr_t<_Value>;
  static_assert(rclcpp::is_ros_compatible_type<Message>::value,
                "A publisher must use a publishable ROS message (no primitive types are possible)");
  PublisherObservable(const std::string &topic_name,
                      const rclcpp::QoS qos = rclcpp::SystemDefaultsQoS()) {
    this->impl()->name = topic_name;
    this->impl()->attach_ = [impl = this->impl(), topic_name, qos](NodeBookkeeping &node) {
      impl->publisher = node.add_publisher<Message>(topic_name, qos);
      impl->register_handler([impl]() {
        const auto &message = impl->value();
        impl->publish(message);
      });
    };
  }
};

/// Wrap the message_filters official ROS package. In the following, "MFL" refers to the
/// message_filters package. An adapter, adapting the message_filters::SimpleFilter to our
/// Observable (two different implementations of almost the same concept). Does nothing else than
/// what message_filters::Subscriber does:
/// https://github.com/ros2/message_filters/blob/humble/include/message_filters/subscriber.h#L349
// We neeed the base to be able to recognize interpolatable nodes for example
template <typename _Message, class _Base = Observable<typename _Message::SharedPtr>>
struct SimpleFilterAdapter : public _Base, public message_filters::SimpleFilter<_Message> {
  SimpleFilterAdapter() {
    this->impl()->register_handler([this]() {
      using Event = message_filters::MessageEvent<const _Message>;
      const auto &new_value = this->impl()->value();  /// There can be no error
      this->signalMessage(Event(new_value));
    });
  }
};

/// TODO this needs to check whether all inputs have the same QoS, so we will have do a walk
/// TODO adapt queue size automatically if we detect very different frequencies so that
/// synchronization still works. I would guess currently it works only if the lowest frequency topic
/// has a frequency of at least 1/queue_size, given the highest frequency topic has a frequency of
/// one.
template <typename... Messages>
struct SynchronizerObservableImpl {
  /// Approx time will work as exact time if the stamps are exactly the same, so I wonder why the
  /// `TImeSynchronizer` uses by default ExactTime
  using Policy = message_filters::sync_policies::ApproximateTime<Messages...>;
  using Sync = message_filters::Synchronizer<Policy>;
  using Inputs = std::tuple<std::shared_ptr<SimpleFilterAdapter<Messages>>...>;
  const auto &inputs() const { return inputs_; }

  void create_mfl_synchronizer(uint32_t queue_size) {
    queue_size_ = queue_size;
    /// They are dynamically allocated as is every other Observable
    // TODO is this still valid
    inputs_ = std::make_tuple(std::make_shared<SimpleFilterAdapter<Messages>>()...);
    auto synchronizer = std::make_shared<Sync>(Policy(queue_size_));
    synchronizer_ = synchronizer;
    /// Connect with the input observables
    std::apply(
        [synchronizer](auto &...input_filters) { synchronizer->connectInput(*input_filters...); },
        inputs_);
    /// TODO not sure why this is needed, present in example code
    synchronizer_->setAgePenalty(0.50);
  }
  /// The input filters
  uint32_t queue_size_{10};
  Inputs inputs_;
  std::shared_ptr<Sync> synchronizer_;
};

template <typename... Messages>
class SynchronizerObservable
    : public Observable<std::tuple<typename Messages::SharedPtr...>, std::string,
                        SynchronizerObservableImpl<Messages...>> {
public:
  using Self = SynchronizerObservable<Messages...>;
  SynchronizerObservable(uint32_t queue_size) {
    this->create_mfl_synchronizer(queue_size);
    /// Note that even if this object is copied, this capture of the this-pointer is still valid
    /// because we only access impl in on_messages
    this->impl()->synchronizer_->registerCallback(&Self::on_messages, this);
  }

private:
  void on_messages(typename Messages::SharedPtr... msgs) {
    this->impl()->resolve(std::forward_as_tuple(msgs...));
  }
};

// A transform broadcaster observable
class TransformPublisherObservable : public Observable<geometry_msgs::msg::TransformStamped> {
  friend class Context;

public:
  using Value = geometry_msgs::msg::TransformStamped;
  TransformPublisherObservable() {
    this->impl()->name = "tf_pub";
    this->impl()->attach_ = [impl = this->impl()](NodeBookkeeping &node) {
      auto tf_broadcaster = node.add_tf_broadcaster_if_needed();
      impl->register_handler([impl, tf_broadcaster]() {
        const auto &new_value = impl->value();  /// There can be no error
        tf_broadcaster->sendTransform(new_value);
      });
    };
  }
};

/// A service observable, storing it's request and response
template <typename _ServiceT>
struct ServiceObservable
    : public Observable<std::pair<std::shared_ptr<typename _ServiceT::Request>,
                                  std::shared_ptr<typename _ServiceT::Response>>> {
  using Request = std::shared_ptr<typename _ServiceT::Request>;
  using Response = std::shared_ptr<typename _ServiceT::Response>;
  using Value = std::pair<Request, Response>;

  ServiceObservable(const std::string &service_name,
                    const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    this->impl()->attach_ = [impl = this->impl(), service_name, qos](NodeBookkeeping &node) {
      node.add_service<_ServiceT>(
          service_name,
          [impl](Request request, Response response) {
            impl->resolve(std::make_pair(request, response));
          },
          qos);
    };
  }
};

/// A service client is a remote procedure call (RPC). It is a computation, and therefore an edge in
/// the DFG
template <typename _ServiceT>
struct ServiceClientImpl : public NodeAttachable {
  using Client = rclcpp::Client<_ServiceT>;
  typename Client::SharedPtr client;
  Duration timeout;
  std::optional<typename Client::SharedFutureWithRequestAndRequestId> maybe_pending_request;
};
template <typename _ServiceT>
struct ServiceClient : public Observable<typename _ServiceT::Response::SharedPtr, std::string,
                                         ServiceClientImpl<_ServiceT>> {
  using Request = typename _ServiceT::Request::SharedPtr;
  using Response = typename _ServiceT::Response::SharedPtr;
  using Client = rclcpp::Client<_ServiceT>;
  using Future = typename Client::SharedFutureWithRequest;

  ServiceClient(const std::string &service_name, const Duration &timeout) {
    this->impl()->name = service_name;
    this->impl()->timeout = timeout;
    this->impl()->attach_ = [impl = this->impl(), service_name](NodeBookkeeping &node) {
      impl->client = node.add_client<_ServiceT>(service_name);
    };
  }

  auto &call(Request request) const {
    if (!wait_for_service()) return *this;
    this->impl()->maybe_pending_request = this->impl()->client->async_send_request(
        request, [this](Future response_futur) { on_response(response_futur); });
    return *this;
  }

protected:
  bool wait_for_service() const {
    if (!this->impl()->client->wait_for_service(this->impl()->timeout)) {
      // Reference:https://github.com/ros2/examples/blob/rolling/rclcpp/services/async_client/main.cpp#L65
      if (!rclcpp::ok()) {
        this->impl()->reject("INTERRUPTED");
      } else {
        this->impl()->reject("SERVICE_UNAVAILABLE");
      }
      return false;
    }
    return true;
  }

  void on_response(Future response_futur) const {
    if (response_futur.valid()) {
      this->impl()->maybe_pending_request = {};
      this->impl()->resolve(response_futur.get().second);
    } else {
      /// The FutureReturnCode enum has SUCCESS, INTERRUPTED, TIMEOUT as possible values.
      /// Since the async_send_request waits on the executor, we cannot only interrupt it with
      /// Ctrl-C
      // Reference:
      // https://github.com/ros2/examples/blob/rolling/rclcpp/services/async_client/main.cpp#L65
      if (!rclcpp::ok()) {
        this->impl()->reject("rclcpp::FutureReturnCode::INTERRUPTED");
      } else {
        this->impl()->reject("rclcpp::FutureReturnCode::TIMEOUT");
      }
      /// Now do the weird cleanup thing that the API-user definitely neither does need to care
      /// nor know about:
      this->impl()->client->remove_pending_request(this->impl()->maybe_pending_request.value());
      /// TODO I'm not sure this will still not leak memory smh:
      /// https://github.com/ros2/rclcpp/issues/1697 Some ROS examples use stuff like timers that
      /// poll for dead request and clean them up. That's why I'll put this assertion here:
      if (size_t num_requests_pruned = this->impl()->client->prune_pending_requests() != 0) {
        throw std::runtime_error(
            "Pruned some more requests even after calling remove_pending_request(), you should buy "
            "a new RPC library.");
      }
      this->impl()->maybe_pending_request = {};
    }
  }
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

  void initialize(NodeBookkeeping &node) {
    if (attachables_.empty()) {
      std::cout << "WARNING: Nothing to spawn, try first to create some Observables" << std::endl;
      return;
    }
    attach_everything_to_node(node);
    was_initialized_ = true;
  }

  void attach_everything_to_node(NodeBookkeeping &node) {
    /// Now attach everything to the ROS-Node, this creates the parameters, publishers etc.
    /// Now, allow for attaching additional nodes after we got the parameters. After attaching,
    /// parameters immediatelly have their values.
    if (icey_debug_print) std::cout << "[icey::Context] Attaching parameters ..." << std::endl;

    for (auto &attachable : attachables_) {
      if (attachable.is_parameter()) attachable.attach_to_node(node);
    }
    if (icey_debug_print)
      std::cout << "[icey::Context] Attaching parameters finished." << std::endl;

    if (after_parameter_initialization_cb_) after_parameter_initialization_cb_();
    if (icey_debug_print)
      std::cout << "[icey::Context] Attaching everything else  ... " << std::endl;
    for (auto &attachable : attachables_) {
      if (!attachable.is_parameter()) attachable.attach_to_node(node);
    }
    if (icey_debug_print) std::cout << "[icey::Context] Attaching finished. " << std::endl;
  }

  /// Creates a new observable of type O by passing the args to the constructor. It also adds it as
  /// a vertex to the graph.
  template <class O, typename... Args>
  auto create_observable(Args &&...args) {
    assert_icey_was_not_initialized();
    O observable{args...};
    NodeAttachable att = *observable.impl();
    attachables_.push_back(att);  /// Register
    observable.impl()->context = shared_from_this();
    observable.impl()->class_name = boost::typeindex::type_id_runtime(observable).pretty_name();
    return observable;
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
      const std::string &name, const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
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
                        const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS()) {
    observable_traits<Parent>{};
    auto child = create_observable<PublisherObservable<obs_val<Parent>>>(topic_name, qos);
    parent.impl()->then([child](const auto &x) { child.impl()->resolve(x); });
  }

  template <class Parent>
  void create_transform_publisher(Parent parent) {
    observable_traits<Parent>{};
    auto child = create_observable<TransformPublisherObservable>();
    parent.impl()->then([child](const auto &x) { child.impl()->resolve(x); });
  }

  auto create_timer(const Duration &interval, bool use_wall_time = false,
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
  auto create_client(Parent parent, const std::string &service_name, const Duration &timeout,
                     const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    observable_traits<Parent>{};  // obs_val since the Request must be a shared_ptr
    static_assert(std::is_same_v<obs_val<Parent>, typename ServiceT::Request::SharedPtr>,
                  "The parent triggering the service must hold a value of type Request::SharedPtr");
    auto service_client = create_observable<ServiceClient<ServiceT>>(service_name, timeout);
    /// TODO maybe solve better. We would need to implement then(Promise<A>, F<A,
    /// Promise<B>>)->Promise<B>, where the service call behaves already like F<A, Promise<B>>: Just
    /// return self. We would also need to accumulate all kinds of errors, i.e. ErrorValue must
    /// become a sum type of all the possible ErrorValues of the chain.
    parent.then([service_client](auto req) { service_client.call(req); });
    /// Pass the error since service calls are chainable
    if constexpr (not std::is_same_v<obs_err<Parent>, Nothing>) {
      parent.except([service_client](auto err) { service_client.impl()->reject(err); });
    }
    return service_client;
  }

  /// Synchronizer that given a reference signal at its first argument, ouputs all the other topics
  // interpolated
  // TODO specialize when reference is a tuple of messages. In this case, we compute the arithmetic
  // TODO impl receive time. For this, this synchronizer must be an attachable because it needs to
  // know the node's clock (that may be simulated time, i.e sub on /clock)
  template <class Reference, class... Interpolatables>
  static auto sync_with_reference(Reference reference, Interpolatables... interpolatables) {
    observable_traits<Reference>{};
    observable_traits<Interpolatables...>{};
    using namespace message_filters::message_traits;
    using RefMsg = obs_msg<Reference>;
    static_assert(HasHeader<RefMsg>::value,
                  "The ROS message type must have a header with the timestamp to be synchronized");
    auto interpolatables_tuple = std::make_tuple(interpolatables...);
    /// TOOD somehow does not work
    // auto all_are_interpolatables = hana::all_of(parents_tuple,  [](auto t) { return
    // hana_is_interpolatable(t); }); static_assert(all_are_interpolatables, "All inputs must be
    // interpolatable when using the sync_with_reference");
    return reference.then([interpolatables_tuple](const auto &new_value) {
      auto parent_maybe_values = hana::transform(interpolatables_tuple, [&](auto parent) {
        return parent.get_at_time(rclcpp::Time(new_value->header.stamp));
      });
      /// TODO If not hana::all(parent_maybe_values, have value) -> get error from parent and reject
      return hana::prepend(parent_maybe_values, new_value);
    });
  }

  /// Synchronizer that synchronizes non-interpolatable signals by matching the time-stamps
  /// approximately
  template <typename... Parents>
  auto synchronize_approx_time(Parents... parents) {
    observable_traits<Parents...>{};
    static_assert(sizeof...(Parents), "You need to synchronize at least two inputs.");
    using namespace hana::literals;
    uint32_t queue_size = 10;
    auto synchronizer = create_observable<SynchronizerObservable<obs_msg<Parents>...>>(queue_size);
    auto zipped = hana::zip(std::forward_as_tuple(parents...), synchronizer->inputs());
    hana::for_each(zipped, [](auto &input_output_tuple) {
      auto &parent = input_output_tuple[0_c];
      auto &synchronizer_input = input_output_tuple[1_c];
      parent.then(
          [synchronizer_input](const auto &x) { synchronizer_input->impl()->resolve(x); });
    });
    return synchronizer;
  }

  /// Synchronize a variable amount of Observables. Uses a Approx-Time synchronizer if the inputs
  /// are not interpolatable or an interpolation-based synchronizer based on a given
  /// (non-interpolatable) reference. Or, a combination of both, this is decided at compile-time.
  /// TODO consider rafactoring using auto template arg to achieve constexpr passing
  template <typename... Parents>
  auto synchronize(Parents... parents) {
    observable_traits<Parents...>{};
    static_assert(sizeof...(Parents) >= 2,
                  "You need to have at least two inputs for synchronization.");
    auto parents_tuple = std::make_tuple(parents...);
    auto interpolatables =
        hana::remove_if(parents_tuple, [](auto t) { return not hana_is_interpolatable(t); });
    auto non_interpolatables =
        hana::remove_if(parents_tuple, [](auto t) { return hana_is_interpolatable(t); });
    constexpr int num_interpolatables = hana::length(interpolatables);
    constexpr int num_non_interpolatables = hana::length(non_interpolatables);
    static_assert(not(num_interpolatables > 0 && num_non_interpolatables == 0),
                  "You are trying to synchronize only interpolatable signals. This does not work, "
                  "you need to "
                  "have at least one non-interpolatable signal that is the common time for all the "
                  "interpolatable signals.");
    // This can only either be one or more than one. Precondition is that we synchronize at least
    // two entities. Given the condition above, the statement follows.
    if constexpr (num_non_interpolatables > 1) {
      /// We need the ApproxTime
      auto approx_time_output = hana::unpack(non_interpolatables, [this](auto... parents) {
        return synchronize_approx_time(parents...);
      });
      if constexpr (num_interpolatables > 1) {
        /// We have interpolatables and non-interpolatables, so we need to use both synchronizers
        return hana::unpack(
            hana::prepend(interpolatables, approx_time_output),
            [this](auto ref, auto... ints) { return sync_with_reference(ref, ints...); });
      } else {
        return approx_time_output;
      }
    } else {
      // Otherwise, we only need a sync with reference
      return hana::unpack(hana::prepend(interpolatables, std::get<0>(non_interpolatables)),
                          [](auto ref, auto... ints) { return sync_with_reference(ref, ints...); });
    }
  }

  /// Serialize, pipe arbitrary number of parents of the same type into one. Needed for the
  /// control-flow where the same publisher can be called from multiple callbacks
  template <typename... Parents>
  auto serialize(Parents... parents) {
    observable_traits<Parents...>{};
    // TODO assert_all_observable_values_are_same<Parents...>();
    /// TODO use obs_val, for this get first of variadic pack with hana
    using Parent = decltype(std::get<0>(std::forward_as_tuple(parents...)));
    using ParentValue = typename std::remove_reference_t<Parent>::Value;
    /// First, create a new observable
    /// TODO error handling
    auto child = create_observable<Observable<ParentValue>>();
    /// Now connect each parent with the child with the identity function
    hana::for_each(std::forward_as_tuple(parents...), [child](auto &parent) {
      parent.then([child](const auto &x) { child.impl()->resolve(x); });
    });
    return child;
  }

  /// Then's on timeout. Creates a new timer.
  // timeout
  // Crates a new timer that sync_with_reference, matching exactly an output frequency. Input must
  // be interpolatable. throttle

  bool empty() const { return attachables_.empty(); }
  void clear() { attachables_.clear(); }

  void assert_icey_was_not_initialized() {
    if (was_initialized_)
      throw std::invalid_argument(
          "You are not allowed to add signals after ICEY was initialized. The graph must be "
          "static");
  }

  bool was_initialized_{false};

  std::vector<NodeAttachable> attachables_;

  std::function<void()> after_parameter_initialization_cb_;
  std::function<void()> on_node_destruction_cb_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// TODO simplify, I do not believe we need this
struct ICEYNodeBase {
  /// This attaches all the ICEY signals to the node, meaning it creates the subcribes etc. It
  /// initializes everything in a pre-defined order.
  virtual void icey_initialize() = 0;
  /// Returns the context that is needed to create ICEY observables
  Context &icey() { return *this->icey_context_; }
  std::shared_ptr<NodeBookkeeping> book_;
  std::shared_ptr<Context> icey_context_{std::make_shared<Context>()};
};

/// The ROS node, additionally owning the context that contains the observables.
/// The template argument is used to support lifecycle_nodes
template <class NodeType>
class NodeWithIceyContext : public NodeType, public ICEYNodeBase {
public:
  using Base = NodeType;
  using Base::Base;  // Take over all base class constructors

  void icey_initialize() override {
    NodeInterfaces node_interfaces(
        this);  /// do not call shared_from_this, since in the class-based API we call
                /// icey_initialize in the ctor, meaning the ctor is not finished yet.
    this->book_ = std::make_shared<NodeBookkeeping>(node_interfaces);
    icey().initialize(*this->book_);
  }

  /// TODO hacky, circular ref, mem leak !
  void create_executor_in_context() {
    this->icey_context_->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->icey_context_->executor_->add_node(this->shared_from_this());
  }
};

/// Await until a promise has a value by spinning the ROS executor for as long as needed. Returns
/// the state of the promise, i.e. of type Result<V, E>
template <class Obs>
auto await(Obs obs) {
  return obs->await();
}

/// Blocking spawn of an existing node.
template <class Node>
void spawn(Node node) {
  /// We use single-threaded executor because the MT one can starve due to a bug
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}

/// Public API aliases:
using Node = NodeWithIceyContext<rclcpp::Node>;
using LifecycleNode = NodeWithIceyContext<rclcpp_lifecycle::LifecycleNode>;

void spin_nodes(const std::vector<std::shared_ptr<Node>> &nodes) {
  rclcpp::executors::SingleThreadedExecutor executor;
  /// This is how nodes should be composed according to ROS guru wjwwood:
  /// https://robotics.stackexchange.com/a/89767. He references
  /// https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp
  for (const auto &node : nodes) executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}

template <class T>
using Parameter = ParameterObservable<T>;

}  // namespace icey

#include <icey/functional_api.hpp>
#define ICEY_ROS2_WAS_INCLUDED
