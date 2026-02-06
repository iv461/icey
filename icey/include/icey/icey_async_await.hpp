/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This header defines an async/await-based interface for services and TF.
/// If you only want async/await and nothing else (i.e. no reactive programming using streams), you
/// can include this header only and get faster compile times.
#pragma once

#include <functional>
#include <icey/impl/promise.hpp>
#include <optional>
#include <thread>  /// for ID
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/create_timer_ros.hpp"
#include "tf2_ros/qos.hpp"

namespace icey {

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
        node_time_source_(node->get_node_time_source_interface()),
        node_waitables_(node->get_node_waitables_interface()) {
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
  auto get_node_waitables_interface() const { return node_waitables_; }

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

  /// Add a one-off task bound to a key. This key can be used to cancel the task. We need these
  /// tasks for the timeout detection for TF, services and actions. Currently, they are implemented
  /// using ordinary rclcpp timers, that need to be cancelled and cleaned up in a deferred manner.
  /// This is of course fugly and slow, but there is no public API to create tasks (aka Waitables)
  /// in the executor, so this is the best we can do.
  template <class CallbackT>
  void add_task_for(uint64_t id, const Duration &timeout, CallbackT on_timeout,
                    rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    oneoff_cancelled_timers_.clear();
    oneoff_active_timers_.emplace(id, create_wall_timer(
                                          timeout,
                                          [this, id, on_timeout]() {
                                            cancel_task_for(id);
                                            on_timeout();
                                          },
                                          group));
  }

  /// Cancel a previously scheduled task by key (no-op if not present)
  bool cancel_task_for(uint64_t id) {
    auto it = oneoff_active_timers_.find(id);
    if (it == oneoff_active_timers_.end()) return false;
    auto timer = it->second;
    timer->cancel();
    oneoff_cancelled_timers_.emplace(timer);
    oneoff_active_timers_.erase(it);
    return true;
  }

  template <class ServiceT, class CallbackT>
  auto create_service(const std::string &service_name, CallbackT &&callback,
                      const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                      rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    return rclcpp::create_service<ServiceT>(node_base_, node_services_, service_name,
                                            std::forward<CallbackT>(callback),
#if RCLCPP_VERSION_MAJOR >= \
    29  // Not removed yet like create_client, but likely will be in the near future
                                            qos,
#else
                                            qos.get_rmw_qos_profile(),
#endif
                                            group);
  }

  template <class Service>
  auto create_client(const std::string &service_name,
                     const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                     rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    return rclcpp::create_client<Service>(node_base_, node_graph_, node_services_, service_name,
#if RCLCPP_VERSION_MAJOR >= 29  // The function overload taking the C QoS type was removed in
                                // https://github.com/ros2/rclcpp/pull/2575
                                          qos,
#else
                                          qos.get_rmw_qos_profile(),
#endif
                                          group);
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
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_;
  /// Deferred cleanup store for one-off timers cancelled inside callbacks
  std::unordered_set<std::shared_ptr<rclcpp::TimerBase>> oneoff_cancelled_timers_;
  /// Active one-off tasks keyed by a stable uintptr_t key
  std::unordered_map<uint64_t, std::shared_ptr<rclcpp::TimerBase>> oneoff_active_timers_;

public:
  // Test helpers (introspection)
  std::size_t oneoff_active_task_count() const { return oneoff_active_timers_.size(); }
  std::size_t oneoff_cancelled_task_count() const { return oneoff_cancelled_timers_.size(); }
};

/// A subscription + buffer for transforms that allows for asynchronous lookups and to subscribe on
/// a single transform between two coordinate systems. It is otherwise implemented similarly to the
/// tf2_ros::TransformListener but offering a well-developed asynchronous API.
/// It subscribes on the topic /tf and listens for relevant transforms (i.e. ones we subscribed to
/// or the ones we requested a lookup). If any relevant transform was received, it notifies via a
/// callback.
///
/// Why not using `tf2_ros::AsyncBuffer` ? Due to multiple issues with it: (1) It uses
/// the `std::future/std::promise` primitives that are effectively useless. (2)
/// tf2_ros::AsyncBuffer::waitForTransform has a bug: We cannot make another lookup in the callback
/// of a lookup (it holds a (non-reentrant) mutex locked while calling the user callback), making it
/// impossible to chain asynchronous operations.
///
/// @sa This class is used to implement the `TransformSubscriptionStream` and the `TransformBuffer`.
struct TransformBufferImpl {
  using TransformMsg = geometry_msgs::msg::TransformStamped;
  using TransformsMsg = tf2_msgs::msg::TFMessage::ConstSharedPtr;
  using OnTransform = std::function<void(const TransformMsg &)>;
  using OnError = std::function<void(const tf2::TransformException &)>;
  using GetFrame = std::function<std::string()>;

  /// We make this class non-copyable since it captures the this-pointer in clojures.
  TransformBufferImpl(const TransformBufferImpl &) = delete;
  TransformBufferImpl(TransformBufferImpl &&) = delete;
  TransformBufferImpl &operator=(const TransformBufferImpl &) = delete;
  TransformBufferImpl &operator=(TransformBufferImpl &&) = delete;

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

  explicit TransformBufferImpl(NodeBase &node) : node_(node) { init(); }

  /// @brief Subscribe to a single transform between two coordinate systems. Every time this
  /// transform changes, the `on_transform` callback function is called. More precisely, every time
  /// a message arrives on the `/tf` or `/tf_static` topic, a lookup is attempted. If the lookup
  /// fails, `on_error` is called. If the lookup succeeds and if the looked up transform is
  // different to the previously emitted one, the `on_transform` callback is called.
  void add_subscription(const GetFrame &target_frame, const GetFrame &source_frame,
                        const OnTransform &on_transform, const OnError &on_error) {
    std::lock_guard<std::recursive_mutex> lock{mutex_};
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
      return Ok(tf);
    } catch (const tf2::TransformException &e) {
      return Err(std::string(e.what()));
    }
  }

  /// @brief Makes an asynchronous lookup for a for a transform at the given time between the given
  /// frames. Callback-based API.
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
  RequestHandle lookup(const std::string &target_frame, const std::string &source_frame, Time time,
                       const Duration &timeout, OnTransform on_transform, OnError on_error) {
    auto request{std::make_shared<TransformRequest>([target_frame]() { return target_frame; },
                                                    [source_frame]() { return source_frame; },
                                                    on_transform, on_error, time)};

    auto weak_request = std::weak_ptr<TransformRequest>(request);
    requests_.emplace(request);
    node_.add_task_for(uint64_t(request.get()), timeout, [this, on_error, request = weak_request]() {
      if (auto req = request.lock()) {
        requests_.erase(req);  // Destroy the request
        on_error(tf2::TimeoutException{"Timed out waiting for transform"});
      }
    });
    return request;
  }

  /// @brief Does an asynchronous lookup for a single transform that can be awaited using `co_await`
  ///
  /// @param target_frame
  /// @param source_frame
  /// @param time At which time to get the transform
  /// @param timeout How long to wait for the transform
  /// @return A future that resolves with the transform or with an error if a timeout occurs
  impl::Promise<geometry_msgs::msg::TransformStamped, std::string> lookup(
      const std::string &target_frame, const std::string &source_frame, const Time &time,
      const Duration &timeout) {
    return impl::Promise<geometry_msgs::msg::TransformStamped, std::string>(
        [this, target_frame, source_frame, time, timeout](auto &promise) {
          auto request_handle = this->lookup(
              target_frame, source_frame, time, timeout,
              [&promise](const geometry_msgs::msg::TransformStamped &tf) { promise.resolve(tf); },
              [&promise](const tf2::TransformException &ex) { promise.reject(ex.what()); });
          promise.set_cancel(
              [this, request_handle](auto &) { this->cancel_request(request_handle); });
        });
  }

  /// @brief Same as `lookup`, but accepts a ROS time point
  ///
  /// @param target_frame
  /// @param source_frame
  /// @param time At which time to get the transform
  /// @param timeout How long to wait for the transform
  /// @return A future that resolves with the transform or with an error if a timeout occurs
  impl::Promise<geometry_msgs::msg::TransformStamped, std::string> lookup(
      const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &time,
      const Duration &timeout) {
    return lookup(target_frame, source_frame, icey::rclcpp_to_chrono(time), timeout);
  }

  /// Cancel a transform request: This means that the registered callbacks will no longer be called.
  /// If the given request does not exist, this function does nothing.
  bool cancel_request(RequestHandle request) {
    std::lock_guard<std::recursive_mutex> lock{mutex_};
    requests_.erase(request);
    return node_.cancel_task_for(uint64_t(request.get()));
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
      /// If the request is destroyed gracefully (lookup succeeded), cancel the associated task
      /// timer
      node_.cancel_task_for(uint64_t(request.get()));
      return true;
    } catch (
        const tf2::TransformException &e) {  /// TODO we could catch for extrapolation here as well
      /// For any other error, we continue waiting
    }
    return false;
  }

  void notify_if_any_relevant_transform_was_received() {
    /// Iterate all requests, notify and maybe erase them if they are requests for a specific time
    std::vector<RequestHandle> requests_to_delete;
    mutex_.lock();
    auto requests = requests_;
    mutex_.unlock();

    for (auto req : requests) {
      if (req->maybe_time) {
        if (maybe_notify_specific_time(req)) {
          requests_to_delete.push_back(req);
        }
      } else {
        // If it is a regular subscription, is is persistend and never erased
        maybe_notify(*req);
      }
    }
    mutex_.lock();
    for (auto k : requests_to_delete) requests_.erase(k);
    mutex_.unlock();
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
};

/// A TransformBuffer offers a modern, asynchronous interface for looking up transforms at a given
/// point in time. This class is lightweight pointer to underlying implementation and can therefore
/// be copied and passed around by value (PIMPL idiom)
struct TransformBuffer {
  using RequestHandle = TransformBufferImpl::RequestHandle;
  using OnTransform = std::function<void(const geometry_msgs::msg::TransformStamped &)>;
  using OnError = std::function<void(const tf2::TransformException &)>;

  TransformBuffer() = default;
  TransformBuffer(std::shared_ptr<TransformBufferImpl> impl) : impl_(impl) {}

  /// @brief Makes an asynchronous lookup for a for a transform at the given time between the given
  /// frames. Callback-based API.
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
  RequestHandle lookup(const std::string &target_frame, const std::string &source_frame, Time time,
                       const Duration &timeout, OnTransform on_transform, OnError on_error) {
    return impl_.lock()->lookup(target_frame, source_frame, time, timeout, on_transform, on_error);
  }

  /// Cancel a transform request: This means that the registered callbacks will no longer be called.
  /// If the given request does not exist, this function does nothing, in this case it returns
  /// false. If a request was cleaned up, this function returns true.
  bool cancel_request(RequestHandle request_handle) {
    return impl_.lock()->cancel_request(request_handle);
  }

  /// @brief Does an asynchronous lookup for a single transform that can be awaited using `co_await`
  ///
  /// @param target_frame
  /// @param source_frame
  /// @param time At which time to get the transform
  /// @param timeout How long to wait for the transform
  /// @return A future that resolves with the transform or with an error if a timeout occurs
  impl::Promise<geometry_msgs::msg::TransformStamped, std::string> lookup(
      const std::string &target_frame, const std::string &source_frame, const Time &time,
      const Duration &timeout) {
    return impl_.lock()->lookup(target_frame, source_frame, time, timeout);
  }

  /// @brief Same as `lookup`, but accepts a ROS time point
  ///
  /// @param target_frame
  /// @param source_frame
  /// @param time At which time to get the transform
  /// @param timeout How long to wait for the transform
  /// @return A future that resolves with the transform or with an error if a timeout occurs
  impl::Promise<geometry_msgs::msg::TransformStamped, std::string> lookup(
      const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &time,
      const Duration &timeout) {
    return impl_.lock()->lookup(target_frame, source_frame, time, timeout);
  }

private:
  std::weak_ptr<TransformBufferImpl> impl_;
};

template <class ServiceT>
struct ServiceClientImpl {
  using Request = typename ServiceT::Request::SharedPtr;
  using Response = typename ServiceT::Response::SharedPtr;
  /// The RequestID in rclcpp is currently of type int64_t.
  using RequestID = decltype(rclcpp::Client<ServiceT>::FutureAndRequestId::request_id);
  using Client = rclcpp::Client<ServiceT>;
  /// We make this class non-copyable since it captures the this pointer in clojures.
  ServiceClientImpl(const ServiceClientImpl &) = delete;
  ServiceClientImpl(ServiceClientImpl &&) = delete;
  ServiceClientImpl &operator=(const ServiceClientImpl &) = delete;
  ServiceClientImpl &operator=(ServiceClientImpl &&) = delete;

  ServiceClientImpl(NodeBase &node, const std::string &service_name,
                    const rclcpp::QoS &qos = rclcpp::ServicesQoS())
      : node_(node), client(node.create_client<ServiceT>(service_name, qos)) {}

  RequestID call(Request request, const Duration &timeout,
                 std::function<void(Response)> on_response,
                 std::function<void(const std::string &)> on_error) {
    /// We have to count the requests ourselves since we need it inside the callback to cancel the
    /// timeout timer but there is no way with the current rclcpp API to obtain the id in the
    /// callback
    auto request_id = request_counter_++;
    auto future_and_req_id = client->async_send_request(
        request, [this, on_response, on_error, request_id](typename Client::SharedFuture result) {
          /// Cancel the timeout task since we got a response
          node_.cancel_task_for(request_id);
          if (!result.valid()) {
            on_error(rclcpp::ok() ? "TIMEOUT" : "INTERRUPTED");
          } else {
            on_response(result.get());
          }
        });
    our_to_real_req_id_.emplace(request_id, future_and_req_id.request_id);
    node_.add_task_for(request_id, timeout, [this, on_error, request_id] {
      auto it = our_to_real_req_id_.find(request_id);
      if (it != our_to_real_req_id_.end()) {
        client->remove_pending_request(it->second);
        our_to_real_req_id_.erase(it);
      }
      on_error("TIMEOUT");
    });
    return request_id;
  }

  impl::Promise<Response, std::string> call(Request request, const Duration &timeout) {
    return impl::Promise<Response, std::string>([this, request, timeout](auto &promise) {
      auto request_id = this->call(
          request, timeout, [&promise](const auto &x) { promise.resolve(x); },
          [&promise](const auto &x) { promise.reject(x); });
      promise.set_cancel([this, request_id](auto &) { cancel_request(request_id); });
    });
  }

  /// Cancel the request so that callbacks will not be called anymore.
  bool cancel_request(RequestID request_id) {
    auto it = our_to_real_req_id_.find(request_id);
    if (it == our_to_real_req_id_.end()) return false;
    client->remove_pending_request(it->second);
    our_to_real_req_id_.erase(it);
    node_.cancel_task_for(request_id);
    return true;
  }

protected:
  NodeBase &node_;
  RequestID request_counter_{
      0};  /// We have to count the requests ourselves, since we cannot access
  /// And a map in case rcl starts to create the request IDs differently compared to how  we are
  /// doing it (otherwise we would depend on an implementation detail of rcl/RMW)
  std::unordered_map<RequestID, RequestID> our_to_real_req_id_;

public:
  /// The underlying rclcpp service client
  std::shared_ptr<rclcpp::Client<ServiceT>> client;
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

  /// Constructs the service client. A node has to be provided because it is needed to create
  /// timeout timers for every service call.
  ServiceClient(NodeBase &node, const std::string &service_name,
                const rclcpp::QoS &qos = rclcpp::ServicesQoS())
      : impl_(std::make_shared<ServiceClientImpl<ServiceT>>(node, service_name, qos)) {}

  /*! Make an asynchronous call to the service with a timeout. Two callbacks may be provided: One
  for the response and one in case of error (timeout or service unavailable).  Requests can never
  hang forever but will eventually time out. Also you don't need to clean up pending requests --
  they will be cleaned up automatically. So this function will never cause any memory leaks. \param
  request the request.
  \param timeout The timeout for the service call, both for service discovery
  and the actual call. \returns The request id using which this request can be cancelled.
  */
  RequestID call(Request request, const Duration &timeout,
                 std::function<void(Response)> on_response,
                 std::function<void(const std::string &)> on_error) {
    return impl_->call(request, timeout, on_response, on_error);
  }

  // clang-format off
  /*! Make an asynchronous call to the service. Returns a impl::Promise that can be awaited using `co_await`.
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
  impl::Promise<Response, std::string> call(Request request, const Duration &timeout) const {
    return impl_->call(request, timeout);
  }

  /// Cancel the request so that callbacks will not be called anymore.
  bool cancel_request(RequestID request_id) { return impl_->cancel_request(request_id); }

  /// Returns the underlying rclcpp service client.
  std::shared_ptr<rclcpp::Client<ServiceT>> client() const { return impl_->client; }

protected:
  /// We own the impl since having a client without a handle to it is not useful for anything.
  /// Still, we do do not want users to have to litter their code with shared pointers. So we still
  /// use PIMPL, but compared to TF we own the impl.
  std::shared_ptr<ServiceClientImpl<ServiceT>> impl_;
};

/// An AsyncGoalHandle is created once a requested goal was accepted by the action server.
/// It provides an async/await based API for requesting the result and cancellation.
/// WARNING: On Humble, there seems to be a memory leak bug in the underlying send_goal API.
template <class ActionT>
struct AsyncGoalHandle {
  using Goal = typename ActionT::Goal;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using Result = typename GoalHandle::WrappedResult;
  using CancelResponse = typename rclcpp_action::Client<ActionT>::CancelResponse::SharedPtr;
  using Feedback = typename rclcpp_action::Client<ActionT>::Feedback;

  using ResultPromise = impl::Promise<Result, std::string>;

  AsyncGoalHandle(NodeBase &node, std::shared_ptr<rclcpp_action::Client<ActionT>> client,
                  const std::shared_ptr<GoalHandle> &goal_handle)
      : node_(node), client_(client), goal_handle_(goal_handle) {}

  ~AsyncGoalHandle() {
    /// This is a cleanup function that removes entries from a hashtable called "goal_handles_"
    /// inside the rclcpp_action::Client, so we have to call it to prevent memory leaks apparently.
#if RCLCPP_VERSION_MAJOR > 16  /// This function does not exist on Humble, source for the rclcpp
                               /// version: https://index.ros.org/p/rclcpp/#humble
    client_.lock()->stop_callbacks(goal_handle_);
#endif
  }

  /// Get the goal UUID.
  const rclcpp_action::GoalUUID &get_goal_id() const { return goal_handle_->get_goal_id(); }
  /// Get the time when the goal was accepted.
  rclcpp::Time get_goal_stamp() const { return goal_handle_->get_goal_stamp(); }
  /// Get the goal status code.
  rclcpp_action::ResultCode get_status() { return goal_handle_->get_status; }

  /// Obtain the result asynchronously.
  /// TODO check in gdb whether it is a problem that this promise might resolve synchronously, i.e.
  /// do we get a stack overflow ?
  ResultPromise &result(const Duration &timeout) {
    node_.add_task_for(uint64_t(&result_promise_), timeout, [this] {
#if RCLCPP_VERSION_MAJOR > 16  /// This function does not exist on Humble, source for the rclcpp
                               /// version: https://index.ros.org/p/rclcpp/#humble
      client_.lock()->stop_callbacks(goal_handle_);
#endif
      result_promise_.reject("RESULT TIMEOUT");
    });
    return result_promise_;
  }

  /// Cancel this goal. Warning: Not co_awaiting this promise leads to use-after free !
  impl::Promise<CancelResponse, std::string> cancel(const Duration &timeout) const {
    return impl::Promise<AsyncGoalHandle, std::string>([this, timeout](auto &promise) {
      client_->async_cancel_goal(goal_handle_, [this, &promise]() {
        node_.cancel_task_for(uint64_t(&promise));
        promise.resolve();
      });
      /// Add timeout task
      node_.add_task_for(uint64_t(&promise), timeout, [this, &promise] {
        client_->stop_callbacks(goal_handle_);
        promise.reject("RESULT TIMEOUT");
      });
      /// We can't cancel the cancellation :(, destroying the promise before the
      /// cancellation is complete leads to UAF.
    });
  }

  /// Returns the held rclcpp_action::GoalHandle.
  std::shared_ptr<GoalHandle> get_goal_handle() const { return goal_handle_; }

  ResultPromise result_promise_;

private:
  NodeBase &node_;
  std::weak_ptr<rclcpp_action::Client<ActionT>> client_;
  std::shared_ptr<GoalHandle> goal_handle_;
};

/*! A action client offering an async/await API and per-request timeouts. Everything happens
asynchronously and returns a promise that can be awaited using co_await.
*/
template <class ActionT>
struct ActionClient {
  using Goal = typename ActionT::Goal;
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using FeedbackCallback = typename GoalHandle::FeedbackCallback;
  using Result = typename GoalHandle::WrappedResult;
  using RequestID = int64_t;

  using AsyncGoalHandleT = std::shared_ptr<AsyncGoalHandle<ActionT>>;
  ActionClient(NodeBase &node, const std::string &action_name) : node_(node) {
    client_ = rclcpp_action::create_client<ActionT>(
        node.get_node_base_interface(), node.get_node_graph_interface(),
        node.get_node_logging_interface(), node.get_node_waitables_interface(), action_name);
  }

  /// Send asynchronously an action goal: if it is accepted, then you get a goal handle.
  template <class F>
  impl::Promise<AsyncGoalHandleT, std::string> send_goal(const Goal &goal, const Duration &timeout,
                                                         F &&feedback_callback) const {
    return impl::Promise<AsyncGoalHandleT, std::string>([this, goal, timeout,
                                                         feedback_callback](auto &promise) {
      typename Client::SendGoalOptions options;
      // Acceptance timeout: reject if no acceptance within timeout
      node_.add_task_for(uint64_t(&promise), timeout,
                         [this, &promise]() { promise.reject("TIMEOUT"); });

      options.goal_response_callback = [this, &promise](auto goal_handle) {
        // Cancel acceptance timeout
        node_.cancel_task_for(uint64_t(&promise));
        if (goal_handle == nullptr) {
          promise.reject("GOAL REJECTED");  /// TODO error type
        } else {
          promise.resolve(std::make_shared<AsyncGoalHandle<ActionT>>(node_, client_, goal_handle));
        }
      };
      /// HINT:  Wrap inside a lambda to support feedback_callback being a coroutine
      options.feedback_callback = [feedback_callback](auto goal_handle, auto feedback) {
        feedback_callback(goal_handle, feedback);
      };
      /// Citing the documentation of this function "[...] WARNING this method has inconsistent
      /// behaviour and a memory leak bug. If you set the result callback in @param options, the
      /// handle will be self referencing, and you will receive
      /// * callbacks even though you do not hold a reference to the shared pointer. In this
      /// case, the self reference will
      /// * be deleted if the result callback was received. If there is no result callback,
      /// there will be a memory leak.[...]".
      // So apparently, we need to set this callback. Internally,
      /// this leads the goal to be "result aware" (???).
      options.result_callback = [this, &promise](const Result &result) {
        if (!promise.has_value()) {
          /// this is a state error, bug in ros
          throw std::invalid_argument(
              "Action result callback was called before goal handle was received");
        } else {
          auto &goal_handle = promise.get_state().value();
          node_.cancel_task_for(uint64_t(&goal_handle->result_promise_));
          goal_handle->result_promise_.resolve(result);
        }
      };
      client_->async_send_goal(goal, options);
    });
  }

protected:
  NodeBase &node_;
  std::shared_ptr<rclcpp_action::Client<ActionT>> client_;
};

/// A context that provides only async/await related entities.
class ContextAsyncAwait : public NodeBase {
public:
  ContextAsyncAwait(const ContextAsyncAwait &) = delete;
  ContextAsyncAwait(ContextAsyncAwait &&) = delete;
  ContextAsyncAwait &operator=(const ContextAsyncAwait &) = delete;
  ContextAsyncAwait &operator=(ContextAsyncAwait &&) = delete;

  /// Constructs the Context from the given node pointer. Supports both rclcpp::Node as well as a
  /// lifecycle node.
  /// @param node the node
  /// @tparam NodeT rclcpp::Node or rclcpp_lifecycle::LifecycleNode
  template <class NodeT>
  explicit ContextAsyncAwait(NodeT *node) : NodeBase(node) {}

  /// Create a subscription and registers the given callback. The callback can be either
  /// synchronous or asynchronous. Works otherwise the same as [rclcpp::Node::create_subscription].
  template <class MessageT, class Callback>
  std::shared_ptr<rclcpp::Subscription<MessageT>> create_subscription_async(
      const std::string &topic_name, Callback &&callback,
      const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
      const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions()) {
    auto subscription = node_base().create_subscription<MessageT>(
        topic_name, [callback](typename MessageT::SharedPtr msg) { callback(msg); }, qos, options);
    std::lock_guard<std::recursive_mutex> lock{bookkeeping_mutex_};
    subscriptions_.push_back(std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(subscription));
    return subscription;
  }

  /// Create a timer that accepts asynchronous callbacks (i.e. coroutines)
  /// \param period the period time
  /// \param callback the callback, may be synchronous or asynchronous
  /// \tparam Callback () -> void or () -> impl::Promise<void>
  /// \note This function creates a wall-clock timer.
  /// \note A callback signature that accepts a TimerInfo argument is not implemented yet
  /// Works otherwise the same as [rclcpp::Node::create_timer].
  template <class Callback>
  std::shared_ptr<rclcpp::TimerBase> create_timer_async(const Duration &period, Callback callback) {
    auto timer = node_base().create_wall_timer(period, [callback]() { callback(std::size_t{}); });
    std::lock_guard<std::recursive_mutex> lock{bookkeeping_mutex_};
    timers_.push_back(std::dynamic_pointer_cast<rclcpp::TimerBase>(timer));
    return timer;
  }

  /// Create a service server supporting asynchronous callbacks (i.e. coroutines). One a request is
  /// received, the provided callback will be called. This callback receives the request and returns
  /// a shared pointer to the response. If it returns a nullptr, then no response is made. The
  /// callback can be either synchronous (a regular function) or asynchronous, i.e. a coroutine. The
  /// callbacks returns the response. The context additionally provides bookkeeping for this
  /// service, this means you do not have to store service in the node class. Works otherwise the
  /// same as [rclcpp::Node::create_service]. \param service_name the name of the service \param
  /// callback the callback \param qos quality of service \tparam Callback Either
  /// (std::shared_ptr<ServiceT::Request>) -> std::shared_ptr<ServiceT::Response> or
  /// (std::shared_ptr<ServiceT::Request>) ->
  /// icey::impl::Promise<std::shared_ptr<ServiceT::Response>>
  template <class ServiceT, class Callback>
  std::shared_ptr<rclcpp::Service<ServiceT>> create_service(
      const std::string &service_name, Callback callback,
      const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    using Request = std::shared_ptr<typename ServiceT::Request>;
    using RequestID = std::shared_ptr<rmw_request_id_t>;
    /// The type of the user callback that can response synchronously (i.e. immediately): It
    /// receives the request and returns the response.
    /// For devs: For "documentation", see
    /// - https://github.com/ros2/rclcpp/pull/1709
    /// - https://github.com/ros2/rclcpp/issues/1707
    /// -
    /// [rcl_send_response](http://docs.ros.org/en/jazzy/p/rcl/generated/function_service_8h_1a8631f47c48757228b813d0849d399d81.html#_CPPv417rcl_send_responsePK13rcl_service_tP16rmw_request_id_tPv)
    auto service = node_base().create_service<ServiceT>(
        service_name,
        [callback](std::shared_ptr<rclcpp::Service<ServiceT>> server, RequestID request_id,
                   Request request) {
          using ReturnType = decltype(callback(std::declval<Request>()));
          if constexpr (!impl::has_promise_type_v<ReturnType>) {
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
          }
        },
        qos);
    std::lock_guard<std::recursive_mutex> lock{bookkeeping_mutex_};
    services_.push_back(std::dynamic_pointer_cast<rclcpp::ServiceBase>(service));
    return service;
  }

  /// Create a service client that supports async/await.
  /// Works otherwise the same as [rclcpp::Node::create_client].
  template <class ServiceT>
  ServiceClient<ServiceT> create_client(const std::string &service_name,
                                        const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    return ServiceClient<ServiceT>(node_base(), service_name, qos);
  }

  /// Create an action server with a synchronous or asynchronous execute callback.
  /// The goal and cancel callbacks default to ACCEPT/ACCEPT behavior.
  template <class ActionT, class ExecuteCallbackT,
            class GoalCallbackT = std::function<rclcpp_action::GoalResponse(
                const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>,
            class CancelCallbackT = std::function<rclcpp_action::CancelResponse(
                std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>>
  std::shared_ptr<rclcpp_action::Server<ActionT>> create_action_server(
      const std::string &action_name, ExecuteCallbackT &&execute_callback,
      GoalCallbackT goal_callback =
          [](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          },
      CancelCallbackT cancel_callback =
          [](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>) {
            return rclcpp_action::CancelResponse::ACCEPT;
          }) {
    using ServerGoalHandleT = rclcpp_action::ServerGoalHandle<ActionT>;

    auto server = rclcpp_action::create_server<ActionT>(
        get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
        get_node_waitables_interface(), action_name, goal_callback, cancel_callback,
        [execute_callback = std::forward<ExecuteCallbackT>(execute_callback)](
            std::shared_ptr<ServerGoalHandleT> goal_handle) {
          using ReturnType = decltype(execute_callback(goal_handle));
          if constexpr (!impl::has_promise_type_v<ReturnType>) {
            execute_callback(goal_handle);
          } else {
            const auto continuation = [](auto exec_cb,
                                         std::shared_ptr<ServerGoalHandleT> gh) -> Promise<void> {
              co_await exec_cb(gh);
              co_return;
            };
            continuation(execute_callback, goal_handle);
          }
        });

    action_servers_.push_back(std::dynamic_pointer_cast<rclcpp_action::ServerBase>(server));
    return server;
  }

  /// Create an action client that supports async/await send_goal
  template <class ActionT>
  ActionClient<ActionT> create_action_client(const std::string &action_name) {
    return ActionClient<ActionT>(node_base(), action_name);
  }

  /// Creates a transform buffer that works like the usual combination of a tf2_ros::Buffer and a
  /// tf2_ros::TransformListener. It is used to `lookup()` transforms asynchronously at a specific
  /// point in time.
  TransformBuffer create_transform_buffer() { return TransformBuffer{add_tf_listener_if_needed()}; }

  /// Get the NodeBase, i.e. the ROS node using which this Context was created.
  NodeBase &node_base() { return static_cast<NodeBase &>(*this); }

  std::shared_ptr<TransformBufferImpl> add_tf_listener_if_needed() {
    std::call_once(tf_buffer_init_flag_, [this]() {
      /// We need only one subscription on /tf, but we can have multiple transforms on which we
      /// listen to
      tf_buffer_impl_ = std::make_shared<TransformBufferImpl>(this->node_base());
    });
    return tf_buffer_impl_;
  }

  /// The TF async interface impl
  std::shared_ptr<TransformBufferImpl> tf_buffer_impl_;
  /// We need bookkeeping for the service servers.
protected:
  std::once_flag
      tf_buffer_init_flag_;  /// Needed for atomic, i.e. thread-safe initialization of tf buffer.
  std::recursive_mutex bookkeeping_mutex_;
  std::vector<std::shared_ptr<rclcpp::TimerBase>> timers_;
  std::vector<std::shared_ptr<rclcpp::ServiceBase>> services_;
  std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;
  std::vector<std::shared_ptr<rclcpp_action::ServerBase>> action_servers_;
};

}  // namespace icey
