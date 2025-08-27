/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#pragma once

#include <coroutine>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <tuple>
#include <unordered_map>

/// Support for lifecycle nodes:
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

/// TF2 support:
#include <icey/impl/promise.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace icey {

inline bool icey_debug_print = false;

using Clock = std::chrono::system_clock;
using Time = std::chrono::time_point<Clock>;
using Duration = Clock::duration;

static rclcpp::Time rclcpp_from_chrono(const Time &time_point) {
  return rclcpp::Time(std::chrono::time_point_cast<std::chrono::nanoseconds>(time_point)
                          .time_since_epoch()
                          .count());
}

static Time rclcpp_to_chrono(const rclcpp::Time &time_point) {
  return Time(std::chrono::nanoseconds(time_point.nanoseconds()));
}

/// A helper to abstract regular rclcpp::Nodes and LifecycleNodes.
/// Similar to the NodeInterfaces class: https://github.com/ros2/rclcpp/pull/2041
/// which doesn't look like it's going to come for Humble:
/// https://github.com/ros2/rclcpp/issues/2309
struct NodeBase {
  template <class _Node>
  explicit NodeBase(_Node *node)
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
                    "NodeBase must be constructed either from a rclcpp::Node or a "
                    "rclcpp_lifecycle::LifecycleNode");
  }

  /// These getters are needed at some places like for the ParameterEventHandler. Other functions
  /// likely require this interface too.
  auto get_node_base_interface() const { return node_base_; }
  auto get_node_graph_interface() const { return node_graph_; }
  auto get_node_clock_interface() const { return node_clock_; }
  auto get_node_logging_interface() const { return node_logging_; }
  auto get_node_timers_interface() const { return node_timers_; }
  auto get_node_topics_interface() const { return node_topics_; }
  auto get_node_services_interface() const { return node_services_; }
  auto get_node_parameters_interface() const { return node_parameters_; }
  auto get_node_time_source_interface() const { return node_time_source_; }

  template <class T>
  auto get_parameter(const std::string &name) {
    return node_parameters_->get_parameter(name).get_value<T>();
  }

  template <class Msg, class F>
  auto create_subscription(const std::string &topic, F &&cb, const rclcpp::QoS &qos,
                           const rclcpp::SubscriptionOptions &options = {}) {
    return rclcpp::create_subscription<Msg>(node_topics_, topic, qos, cb, options);
  }

  template <class Msg>
  auto create_publisher(const std::string &topic, const rclcpp::QoS &qos,
                        const rclcpp::PublisherOptions publisher_options = {}) {
    return rclcpp::create_publisher<Msg>(node_topics_, topic, qos, publisher_options);
  }

  template <class CallbackT>
  auto create_wall_timer(const Duration &time_interval, CallbackT &&callback,
                         rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    /// We have not no normal timer in Humble, this is why only wall_timer is supported
    return rclcpp::create_wall_timer(time_interval, std::forward<CallbackT>(callback), group,
                                     node_base_.get(), node_timers_.get());
  }

  template <class ServiceT, class CallbackT>
  auto create_service(const std::string &service_name, CallbackT &&callback,
                      const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                      rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    return rclcpp::create_service<ServiceT>(node_base_, node_services_, service_name,
                                            std::forward<CallbackT>(callback),
                                            qos.get_rmw_qos_profile(), group);
  }

  template <class Service>
  auto create_client(const std::string &service_name,
                     const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                     rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    return rclcpp::create_client<Service>(node_base_, node_graph_, node_services_, service_name,
                                          qos.get_rmw_qos_profile(), group);
  }

  bool is_regular_node() { return maybe_regular_node; }
  bool is_lifecycle_node() { return maybe_lifecycle_node; }

  /// Get the underlying (regular) rclcpp::Node from which this NodeBase was constructed
  rclcpp::Node &as_node() {
    if (!maybe_regular_node)
      throw std::invalid_argument("This NodeBase does hold a regular node but a lifecycle node");
    return *maybe_regular_node;
  }

  /// Get the underlying rclcpp_lifecycle::LifecycleNode from which this NodeBase was constructed
  rclcpp_lifecycle::LifecycleNode &as_lifecycle_node() {
    if (!maybe_lifecycle_node)
      throw std::invalid_argument("This NodeBase does hold a lifecycle node but a regular node");
    return *maybe_lifecycle_node;
  }

protected:
  /// This is set to either of the two, depending on which node we got (TODO sum types are a thing).
  /// It has to be a raw pointer since this nodeInterface is needed during node construction where
  /// we cannot call shared_ptr::shared_from_this
  rclcpp::Node *maybe_regular_node{nullptr};
  rclcpp_lifecycle::LifecycleNode *maybe_lifecycle_node{nullptr};

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr node_time_source_;
};

/// A weak pointer that supports operator->.
template <class T>
struct Weak {
  Weak() = default;
  Weak(std::shared_ptr<T> p) : p_(p) {}
  T *operator->() const {
    if (!p_.lock()) throw std::bad_weak_ptr();
    return p_.lock().get();
  }
  T &operator*() const {
    if (!p_.lock()) throw std::bad_weak_ptr();
    return *p_.lock().get();
  }
  T *get() const {
    if (!p_.lock()) throw std::bad_weak_ptr();
    return p_.lock().get();
  }
  auto lock() const { return p_.lock(); }
  std::weak_ptr<T> p_;
};

/// A subscription + buffer for transforms that allows for asynchronous lookups and to subscribe on
/// a single transform between two coordinate systems. It is otherwise implemented similarly to the
/// tf2_ros::TransformListener but offering a well-developed asynchronous API.
///  It works like this: Every time a new message is received on /tf, we check whether a
/// It subscribes on the topic /tf and listens for relevant transforms (i.e. ones we subscribed to
/// or the ones we requested a lookup). If any relevant transform was received, it notifies via a
/// callback. It is therefore an asynchronous interface to TF.
///
/// Why not using `tf2_ros::AsyncBuffer` ? Due to multiple issues with it. (1) It uses
/// the `std::future/std::promise` primitives that are effectively useless. (2)
/// tf2_ros::AsyncBuffer::waitFroTransform has a bug: We cannot make another lookup in the callback
/// of a lookup (it holds a (non-reentrant) mutex locked while calling the user callback), making it
/// impossible to chain asynchronous operations.
///
/// @sa This class is used to implement the `TransformSubscriptionStream` and the `TransformBuffer`.
struct TFListener {
  using TransformMsg = geometry_msgs::msg::TransformStamped;
  using TransformsMsg = tf2_msgs::msg::TFMessage::ConstSharedPtr;

  using OnTransform = std::function<void(const TransformMsg &)>;
  using OnError = std::function<void(const tf2::TransformException &)>;

  using GetFrame = std::function<std::string()>;

  /// A transform request stores a request for looking up a transform between two coordinate
  /// systems. Either (1) at a particular time with a timeout, or (2) as a subscription. When
  /// "subscribing" to a transform, we yield a transform each time it changes.
  struct TransformRequest {
    TransformRequest(const GetFrame &target_frame, const GetFrame &source_frame,
                     OnTransform on_transform, OnError on_error,
                     std::optional<Time> _maybe_time = {})
        : target_frame(target_frame),
          source_frame(source_frame),
          on_transform(on_transform),
          on_error(on_error),
          maybe_time(_maybe_time) {}
    GetFrame target_frame;
    GetFrame source_frame;
    std::optional<TransformMsg> last_received_transform;
    OnTransform on_transform;
    OnError on_error;
    std::optional<Time> maybe_time;
  };

  /// A request for a transform lookup: It represents effectively a single call to the async_lookup
  /// function. Even if you call async_lookup with the exact same arguments (same frames and time),
  /// this will count as two separate requests.
  using RequestHandle = std::shared_ptr<TransformRequest>;

  explicit TFListener(NodeBase &node) : node_(node) { init(); }

  /// @brief Subscribe to a single transform between two coordinate systems. Every time this
  /// transform changes, the `on_transform` callback function is called. More precisely, every time
  /// a message arrives on the `/tf` or `/tf_static` topic, a lookup is attempted. If the lookup
  /// fails, `on_error` is called. If the lookup succeeds and if the looked up transform is
  // different to the previously emitted one, the `on_transform` callback is called.
  void add_subscription(const GetFrame &target_frame, const GetFrame &source_frame,
                        const OnTransform &on_transform, const OnError &on_error) {
    requests_.emplace(
        std::make_shared<TransformRequest>(target_frame, source_frame, on_transform, on_error));
  }

  /// @brief Queries the TF buffer for a transform at the given time between the given frames. It
  /// does not wait but instead only returns something if the transform is already in the buffer.
  ///
  /// @param target_frame
  /// @param source_frame
  /// @param time
  /// @return The transform or the TF error if the transform is not in the buffer.
  Result<geometry_msgs::msg::TransformStamped, std::string> get_from_buffer(
      std::string target_frame, std::string source_frame, const Time &time) const {
    try {
      const tf2::TimePoint legacy_timepoint = tf2_ros::fromRclcpp(rclcpp_from_chrono(time));
      // Note that this call does not wait, the transform must already have arrived.
      auto tf = buffer_->lookupTransform(target_frame, source_frame, legacy_timepoint);
      return Result<geometry_msgs::msg::TransformStamped, std::string>::Ok(
          tf);  /// my Result-type is not the best, but it's also only 25 lines :D
    } catch (const tf2::TransformException &e) {
      return Result<geometry_msgs::msg::TransformStamped, std::string>::Err(e.what());
    }
  }

  /// @brief Makes an asynchronous lookup for a for a transform at the given time between the given
  /// frames.
  ///
  /// @param target_frame
  /// @param source_frame
  /// @param time
  /// @param timeout
  /// @param on_transform The callback that is called with the requested transform after it becomes
  /// available
  /// @param on_error The callback that is called if a timeout occurs.
  /// @return The "handle", identifying this request. You can use this handle to call
  /// `cancel_request` if you want to cancel the request.
  /// @warning Currently the timeout is measured with wall-time, i.e. does not consider sim-time due
  /// to the limitation of ROS 2 Humble only offering wall-timers
  RequestHandle async_lookup(const std::string &target_frame, const std::string &source_frame,
                             Time time, const Duration &timeout, OnTransform on_transform,
                             OnError on_error) {
    /// Clean up the cancelled timers, i.e. collect the rclcpp::TimerBase objects for timers that
    /// were cancelled since the last call to async_lookup:
    // We need to do this kind of deferred cleanup because we would likely get a deadlock if
    // we tried to clean them up in their own callback. ("Likely" means that whether a deadlock
    // occurs is currently an unspecified behavior, and therefore likely to change in the future.)
    cancelled_timers_.clear();
    auto request{std::make_shared<TransformRequest>([target_frame]() { return target_frame; },
                                                    [source_frame]() { return source_frame; },
                                                    on_transform, on_error, time)};
    requests_.emplace(request);
    /// TODO use non-wall timer so that this works with rosbags, it is not available in Humble
    active_timers_.emplace(
        request,
        rclcpp::create_wall_timer(
            timeout,
            [this, on_error, request = Weak(request)]() {
              active_timers_.at(request.lock())
                  ->cancel();  /// cancel this timer, (it still lives in the active_timers_)
              cancelled_timers_.emplace(active_timers_.at(
                  request.lock()));  /// Copy the timer over to the cancelled ones so that we know
                                     /// we need to clean it up next time
              active_timers_.erase(request.lock());  // Erase this timer from active_timers_
              requests_.erase(request.lock());       // Destroy the request
              on_error(tf2::TimeoutException{"Timed out waiting for transform"});
            },
            nullptr, node_.get_node_base_interface().get(),
            node_.get_node_timers_interface().get()));
    return request;
  }

  /// Cancel a transform request: This means that the registered callbacks will no longer be called.
  /// If the given request does not exist, this function does nothing.
  bool cancel_request(RequestHandle request) {
    requests_.erase(request);
    return active_timers_.erase(request);
  }

  /// We take a tf2_ros::Buffer instead of a tf2::BufferImpl only to be able to use ROS-time API
  /// (internally TF2 has it's own timestamps...), not because we need to wait on anything (that's
  /// what tf2_ros::Buffer does in addition to tf2::BufferImpl).
  std::shared_ptr<tf2_ros::Buffer> buffer_;

protected:
  void init() {
    init_tf_buffer();
    message_subscription_tf_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", [this](TransformsMsg msg) { on_tf_message(msg, false); },
        tf2_ros::DynamicListenerQoS());
    message_subscription_tf_static_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", [this](TransformsMsg msg) { on_tf_message(msg, true); },
        tf2_ros::StaticListenerQoS());
  }

  void init_tf_buffer() {
    /// Mind that the official example regarding the piece of code below is incomplete:
    /// ([official](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html))
    /// . I'm following this example instead:
    /// https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/src/imu_transformer.cpp#L16
    /// See also the following discussions on why this code needs to be this way:
    /// https://answers.ros.org/question/372608/?sort=votes
    /// https://github.com/ros-navigation/navigation2/issues/1182
    /// https://github.com/ros2/geometry2/issues/446
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_node_clock_interface()->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        node_.get_node_base_interface(), node_.get_node_timers_interface());
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
        RCLCPP_ERROR(node_.get_node_logging_interface()->get_logger(),
                     "Failure to set received transform from %s to %s with error: %s\n",
                     transform.child_frame_id.c_str(), transform.header.frame_id.c_str(),
                     temp.c_str());
      }
    }
  }

  /// This simply looks up the transform in the buffer at the latest stamp and checks if it
  /// changed with respect to the previously received one. If the transform has changed, we know
  /// we have to notify. Returns true when it notified about a changed transform (called
  /// on_transform) and false otherwise.
  bool maybe_notify(TransformRequest &info) {
    try {
      /// Note that this does not wait/thread-sleep etc. This is simply a lookup in a
      /// std::vector/tree.
      geometry_msgs::msg::TransformStamped tf_msg =
          buffer_->lookupTransform(info.target_frame(), info.source_frame(), tf2::TimePointZero);
      if (!info.last_received_transform || tf_msg != *info.last_received_transform) {
        info.last_received_transform = tf_msg;
        info.on_transform(tf_msg);
        return true;
      }
    } catch (const tf2::TransformException &e) {
      info.on_error(e);
    }
    return false;
  }

  bool maybe_notify_specific_time(RequestHandle request) {
    try {
      /// Note that this does not wait/thread-sleep etc. This is simply a lookup in a
      /// std::vector/tree.
      geometry_msgs::msg::TransformStamped tf_msg = buffer_->lookupTransform(
          request->target_frame(), request->source_frame(), request->maybe_time.value());
      request->on_transform(tf_msg);
      /// If the request is destroyed gracefully(because the lookup succeeded), destroy (and
      /// therefore cancel) the associated timer:
      active_timers_.erase(request);
      return true;
    } /* catch (tf2::ExtrapolationException e) { // TODO BackwardExtrapolationException (),
       /// If we tried to extrapolate back into the past, we cannot fulfill the promise anymore, so
     we reject it: info.on_error(e); return true;
     }*/
    catch (const tf2::TransformException &e) {
      /// For any other error, we continue waiting
    }
    return false;
  }

  void notify_if_any_relevant_transform_was_received() {
    /// Iterate all requests, notify and maybe erase them if they are requests for a specific time
    std::erase_if(requests_, [this](auto req) {
      if (req->maybe_time) {
        return maybe_notify_specific_time(req);
      } else {
        // If it is a regular subscription, is is persistend and never erased
        maybe_notify(*req);
        return false;
      }
    });
  }

  void on_tf_message(const TransformsMsg &msg, bool is_static) {
    store_in_buffer(*msg, is_static);
    notify_if_any_relevant_transform_was_received();
  }

  NodeBase &node_;  /// Hold weak reference to the NodeBase because the Context owns the NodeBase as
                    /// well, so we avoid circular reference

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_;

  std::unordered_set<RequestHandle> requests_;
  /// The timeout timers for every lookup transform request: These are only the active timers, i.e.
  /// the timeout timers for pending requests.
  std::unordered_map<RequestHandle, std::shared_ptr<rclcpp::TimerBase>> active_timers_;
  /// A separate hashset for cancelled timers so that we know immediately which are cancelled.
  std::unordered_set<std::shared_ptr<rclcpp::TimerBase>> cancelled_timers_;
};

/*! A service client offering an async/await API and per-request timeouts. Service calls happen
asynchronously and return a promise that can be awaited using co_await. This allows synchronization
with other operations, enabling you to effectively perform synchronous service calls. We do not
offer a function to wait until the service is available for two reasons: (1) there is no
asynchronous function available at the rclcpp layer (re-implementation using asynchronous graph
change notification would be required), and (2) ROS does not seem to optimize RPC calls by keeping
connections open; therefore, it does not seem beneficial to differentiate whether the server is
available or not.
*/
template <class ServiceT>
struct ServiceClient {
  using Request = typename ServiceT::Request::SharedPtr;
  using Response = typename ServiceT::Response::SharedPtr;
  /// The RequestID in rclcpp is currently of type int64_t.
  using RequestID = decltype(rclcpp::Client<ServiceT>::FutureAndRequestId::request_id);
  using Client = rclcpp::Client<ServiceT>;

  /// Constructs the service client. A node has to be provided because it is needed to create
  /// timeout timers for every service call.
  ServiceClient(Weak<NodeBase> node, const std::string &service_name,
                const rclcpp::QoS &qos = rclcpp::ServicesQoS())
      : node_(node), client(node_->create_client<ServiceT>(service_name, qos)) {}

  // clang-format off
  /*! Make an asynchronous call to the service with a timeout. Two callbacks may be provided: One for the response and one in case of error (timeout or service unavailable).  Requests can never hang forever but will eventually time out. Also you don't need to clean up pending requests -- they will be cleaned up automatically. So this function will never cause any memory leaks.
  \param request the request
  \param timeout The timeout for the service call, both for service discovery and the actual call.
  \returns The request id using which this request can be cancelled.
  */
  RequestID call(Request request, const Duration &timeout,
                 std::function<void(Response)> on_response,
                 std::function<void(const std::string &)> on_error) {
    /// Clean up the cancelled timers, i.e. collect the rclcpp::TimerBase objects for timers that
    /// were cancelled since the last call:
    // We need to do this kind of deferred cleanup because we would likely get a deadlock if
    // we tried to clean them up in their own callback. ("Likely" means that whether a deadlock
    // occurs is currently an unspecified behavior, and therefore likely to change in the future.)
    cancelled_timers_.clear();
    auto req_id = std::make_shared<RequestID>();
    auto future_and_req_id = client->async_send_request(
        request, [this, on_response, on_error, req_id](typename Client::SharedFuture result) {
          if (!result.valid()) {
            on_error(rclcpp::ok() ? "TIMEOUT" : "INTERRUPTED");
          } else {
            /// Cancel and erase the timeout timer since we got a response
            active_timers_.erase(*req_id);
            on_response(result.get());
          }
        });
    *req_id = future_and_req_id.request_id;

    active_timers_.emplace(
        future_and_req_id.request_id,
        node_->create_wall_timer(timeout,
                                [this, on_error, request_id = future_and_req_id.request_id] {
                                  client->remove_pending_request(request_id);
                                  active_timers_.at(request_id)->cancel();
                                  cancelled_timers_.emplace(active_timers_.at(request_id));
                                  active_timers_.erase(request_id);
                                  on_error("TIMEOUT");
                                }));
    return future_and_req_id.request_id;
  }

  // clang-format off
  /*! Make an asynchronous call to the service. Returns a Promise that can be awaited using `co_await`.
  Requests can never hang forever but will eventually time out. Also you don't need to clean up pending requests -- they will be cleaned up automatically. So this function will never cause any memory leaks.
  \param request the request
  \param timeout The timeout for the service call, both for service discovery and the actual call.
  \returns A promise that can be awaited to obtain the response or an error. Possible errors are "TIMEOUT", "SERVICE_UNAVAILABLE" or "INTERRUPTED".
  
  Example usage:
  \verbatim
    auto client = node->icey().create_client<ExampleService>("set_bool_service1");
    auto request = std::make_shared<ExampleService::Request>();
    icey::Result<ExampleService::Response::SharedPtr, std::string> result = co_await client.call(request, 1s); 
  \endverbatim
  */
  // clang-format on
  Promise<Response, std::string> call(Request request, const Duration &timeout) {
    using Cancel = typename Promise<Response, std::string>::Cancel;
    return Promise<Response, std::string>{[this, request, timeout](auto &promise) {
      auto request_id = this->call(
          request, timeout, [&](const auto &x) { promise.resolve(x); },
          [&](const auto &x) { promise.reject(x); });
      return Cancel{[this, request_id](auto &) { cancel_request(request_id); }};
    }};
  }

  /// Cancel the request so that callbacks will not be called anymore.
  bool cancel_request(RequestID request_id) {
    client->remove_pending_request(request_id);
    return active_timers_.erase(request_id);
  }

  /// The underlying rclcpp service client
  std::shared_ptr<rclcpp::Client<ServiceT>> client;

protected:
  Weak<NodeBase> node_;
  /// The timeout timers for every lookup transform request: These are only the active timers, i.e.
  /// the timeout timers for pending requests.
  std::unordered_map<RequestID, std::shared_ptr<rclcpp::TimerBase>> active_timers_;
  /// A separate hashset for cancelled timers so that we know immediately which are cancelled.
  std::unordered_set<std::shared_ptr<rclcpp::TimerBase>> cancelled_timers_;
};

/// The context providing an Node-like API but with async/await compatible entities.
class Context : public NodeBase, public std::enable_shared_from_this<Context> {
public:
  /// Constructs the Context from the given node pointer. Supports both rclcpp::Node as well as a
  /// lifecycle node.
  /// @param node the node
  /// @tparam NodeT rclcpp::Node or rclcpp_lifecycle::LifecycleNode
  template <class NodeT>
  explicit Context(NodeT *node) : NodeBase(node) {}

  /// Create a subscription and registers the given callback. The callback can be either
  /// synchronous or asynchronous. Works otherwise the same as [rclcpp::Node::create_subscription].
  template <class MessageT, class Callback>
  std::shared_ptr<rclcpp::Subscription<MessageT>> create_subscription(
      const std::string &topic_name, Callback &&callback,
      const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
      const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions()) {
    auto subscription = node_base().create_subscription<MessageT>(
        topic_name,
        [callback](typename MessageT::SharedPtr msg) {
          using ReturnType = decltype(callback(msg));
          if constexpr (has_promise_type_v<ReturnType>) {
            const auto continuation = [](auto msg, auto &&callback) -> Promise<void> {
              co_await callback(msg);
              co_return;
            };
            continuation(msg, std::forward<Callback>(callback));
          } else {
            callback(msg);
          }
        },
        qos, options);
    return subscription;
  }

  /// Create a timer that accepts asynchronous callbacks (i.e. coroutines)
  /// \param period the period time
  /// \param callback the callback, may be synchronous or asynchronous
  /// \tparam Callback () -> void or () -> Promise<void>
  /// \note This function creates a wall-clock timer.
  /// \note A callback signature that accepts a TimerInfo argument is not implemented yet
  /// Works otherwise the same as [rclcpp::Node::create_timer].
  template <class Callback>
  std::shared_ptr<rclcpp::TimerBase> create_timer(const Duration &period, Callback &&callback) {
    return node_base().create_wall_timer(period, [callback]() {
      using ReturnType = decltype(callback());
      if constexpr (has_promise_type_v<ReturnType>) {
        const auto continuation = [](auto &&callback) -> Promise<void> {
          co_await callback();
          co_return;
        };
        continuation(std::forward<Callback>(callback));
      } else {
        callback();
      }
    });
  }

  /// Create a service server supporting asynchronous callbacks (i.e. coroutines). One a request is
  /// received, the provided callback will be called. This callback receives the request and returns
  /// a shared pointer to the response. If it returns a nullptr, then no response is made. The
  /// callback can be either synchronous (a regular function) or asynchronous, i.e. a coroutine. The
  /// callbacks returns the response. Works otherwise the same as [rclcpp::Node::create_service].
  template <class ServiceT, class Callback>
  std::shared_ptr<rclcpp::Service<ServiceT>> create_service(
      const std::string &service_name, Callback &&callback,
      const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    using Request = std::shared_ptr<typename ServiceT::Request>;
    using RequestID = std::shared_ptr<rmw_request_id_t>;
    using Response = std::shared_ptr<typename ServiceT::Response>;
    /// The type of the user callback that can response synchronously (i.e. immediately): It
    /// receives the request and returns the response.
    using SyncCallback = std::function<Response(Request)>;
    /// The type of the user callback that responds asynchronously, i.e. is a coroutine.
    using AsyncCallback = std::function<Promise<Response>(Request)>;
    /// For devs: For "documentation", see
    /// - https://github.com/ros2/rclcpp/pull/1709
    /// - https://github.com/ros2/rclcpp/issues/1707
    /// -
    /// [rcl_send_response](http://docs.ros.org/en/jazzy/p/rcl/generated/function_service_8h_1a8631f47c48757228b813d0849d399d81.html#_CPPv417rcl_send_responsePK13rcl_service_tP16rmw_request_id_tPv)
    return node_base().create_service<ServiceT>(
        service_name,
        [callback](std::shared_ptr<rclcpp::Service<ServiceT>> server, RequestID request_id,
                   Request request) {
          using ReturnType = decltype(callback(std::declval<Request>()));
          if constexpr (!has_promise_type_v<ReturnType>) {
            auto response = callback(request);
            if (response)  /// If we got nullptr, this means we do not respond.
              server->send_response(*request_id, *response);
          } else {
            const auto continuation = [](auto server, const auto &async_callback,
                                         RequestID request_id, Request request) -> Promise<void> {
              auto response = co_await async_callback(request);
              if (response)  /// If we got nullptr, this means we do not respond.
                server->send_response(*request_id, *response);
              co_return;
            };
            continuation(server, callback, request_id, request);
          } // TODO more strict type checking 
          /*else {
            static_assert(std::is_array_v<int>,
                          "Service server callbacks must have either the signature "
                          "(std::shared_ptr<Request>) -> std::shared_ptr<Response> (synchronous "
                          "callback) or (std::shared_ptr<Request>) -> "
                          "icey::Promise<std::shared_ptr<Response>> (asynchronous callback).");
          }*/
        },
        qos);
  }

  /// Create a service client that supports async/await.
  /// Works otherwise the same as [rclcpp::Node::create_client].
  template <class ServiceT>
  ServiceClient<ServiceT> create_client(const std::string &service_name,
                                        const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    return ServiceClient<ServiceT>(std::dynamic_pointer_cast<NodeBase>(this->shared_from_this()),
                                   service_name, qos);
  }

  /// Get the NodeBase, i.e. the ROS node using which this Context was created.
  NodeBase &node_base() { return static_cast<NodeBase &>(*this); }
};

}  // namespace icey
