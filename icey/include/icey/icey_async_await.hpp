/// Copyright © 2025 Technische Hochschule Augsburg
/// Copyright 2018 Open Source Robotics Foundation, Inc.
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This header defines an async/await-based interface for services and TF.
/// If you only want async/await and nothing else (i.e. no reactive programming using streams), you
/// can include this header only and get faster compile times.
#pragma once

#include <atomic>
#include <functional>
#include <icey/action/action.hpp>
#include <icey/impl/promise.hpp>
#include <mutex>
#include <optional>
#include <thread>  /// for ID
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.hpp"

namespace icey {

#ifndef ICEY_ASYNC_AWAIT_THREAD_SAFE
#define ICEY_ASYNC_AWAIT_THREAD_SAFE 1
#endif

using Clock = std::chrono::system_clock;
using Time = std::chrono::time_point<Clock>;
using Duration = Clock::duration;

[[maybe_unused]] static rclcpp::Time rclcpp_from_chrono(const Time &time_point) {
  return rclcpp::Time(std::chrono::time_point_cast<std::chrono::nanoseconds>(time_point)
                          .time_since_epoch()
                          .count());
}

[[maybe_unused]] static Time rclcpp_to_chrono(const rclcpp::Time &time_point) {
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
#if ICEY_ASYNC_AWAIT_THREAD_SAFE
    std::lock_guard<std::mutex> lock{oneoff_tasks_mutex_};
#endif
    oneoff_cancelled_timers_.clear();
    oneoff_active_timers_.emplace(id, create_wall_timer(
                                          timeout,
                                          [this, id, on_timeout]() {
                                            if (cancel_task_for(id)) {
                                              on_timeout();
                                            }
                                          },
                                          group));
  }

  /// Cancel a previously scheduled task by key (no-op if not present)
  bool cancel_task_for(uint64_t id) {
#if ICEY_ASYNC_AWAIT_THREAD_SAFE
    std::lock_guard<std::mutex> lock{oneoff_tasks_mutex_};
#endif
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
#if ICEY_ASYNC_AWAIT_THREAD_SAFE
  mutable std::mutex oneoff_tasks_mutex_;
#endif

public:
  // Test helpers (introspection)
  std::size_t oneoff_active_task_count() const {
#if ICEY_ASYNC_AWAIT_THREAD_SAFE
    std::lock_guard<std::mutex> lock{oneoff_tasks_mutex_};
#endif
    return oneoff_active_timers_.size();
  }
  std::size_t oneoff_cancelled_task_count() const {
#if ICEY_ASYNC_AWAIT_THREAD_SAFE
    std::lock_guard<std::mutex> lock{oneoff_tasks_mutex_};
#endif
    return oneoff_cancelled_timers_.size();
  }
};

struct TransformBufferImpl;
/// A TransformBuffer offers an async/await interface for looking up transforms at a given
/// point in time. This class is lightweight pointer to underlying implementation and can therefore
/// be copied and passed around by value (PIMPL idiom)
struct TransformBuffer {
  using RequestHandle = uint64_t;
  using OnTransform = std::function<void(const geometry_msgs::msg::TransformStamped &)>;
  using OnError = std::function<void(const tf2::TransformException &)>;
  using GetFrame = std::function<std::string()>;

  /// Initializes the transform buffer by starting the subscriptions on TF. Returns *this;
  /// Does nothing if already initialized.
  TransformBuffer &init(std::weak_ptr<NodeBase> node_base);

  /// Return the underlying tf2_ros::Buffer.
  std::shared_ptr<tf2_ros::Buffer> get_buffer() const;

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
  /// @return The request id, identifying this request. You can use this handle to call
  /// `cancel_request` if you want to cancel the request.
  /// @warning Currently the timeout is measured with wall-time, i.e. does not consider sim-time due
  /// to the limitation of ROS 2 Humble only offering wall-timers
  uint64_t lookup(const std::string &target_frame, const std::string &source_frame, Time time,
                  const Duration &timeout, OnTransform on_transform, OnError on_error);

  /// @brief Subscribe to a single transform between two coordinate systems. Every time this
  /// transform changes, the `on_transform` callback function is called. More precisely, every time
  /// a message arrives on the `/tf` or `/tf_static` topic, a lookup is attempted. If the lookup
  /// fails, `on_error` is called. If the lookup succeeds and if the looked up transform is
  // different to the previously emitted one, the `on_transform` callback is called.
  void add_subscription(const GetFrame &target_frame, const GetFrame &source_frame,
                        const OnTransform &on_transform, const OnError &on_error);

  /// Cancel a transform request: This means that the registered callbacks will no longer be called.
  /// If the given request does not exist, this function does nothing, in this case it returns
  /// false. If a request was cleaned up, this function returns true.
  bool cancel_request(uint64_t request_id);

  /// @brief Does an asynchronous lookup for a single transform that can be awaited using `co_await`
  ///
  /// @param target_frame
  /// @param source_frame
  /// @param time At which time to get the transform
  /// @param timeout How long to wait for the transform
  /// @return A future that resolves with the transform or with an error if a timeout occurs
  impl::Promise<geometry_msgs::msg::TransformStamped, std::string> lookup(
      const std::string &target_frame, const std::string &source_frame, const Time &time,
      const Duration &timeout);

  /// @brief Same as `lookup`, but accepts a ROS time point
  ///
  /// @param target_frame
  /// @param source_frame
  /// @param time At which time to get the transform
  /// @param timeout How long to wait for the transform
  /// @return A future that resolves with the transform or with an error if a timeout occurs
  impl::Promise<geometry_msgs::msg::TransformStamped, std::string> lookup(
      const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &time,
      const Duration &timeout);

private:
  std::shared_ptr<TransformBufferImpl> impl_;
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

  ServiceClientImpl(std::weak_ptr<NodeBase> node, const std::string &service_name,
                    const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                    rclcpp::CallbackGroup::SharedPtr group = nullptr)
      : node_(node), client(node.lock()->create_client<ServiceT>(service_name, qos, group)) {}

  impl::Promise<Response, std::string> call(Request request, const Duration &timeout) {
    return impl::Promise<Response, std::string>([this, request, timeout](auto &promise) {
      auto future_and_req_id = client->async_send_request(
          request, [this, &promise](typename Client::SharedFuture result) {
            /// Cancel the timeout task since we got a response
            if (node_.lock()->cancel_task_for(uint64_t(&promise))) {
              if (!result.valid()) {
                promise.reject(rclcpp::ok() ? "TIMEOUT" : "INTERRUPTED");
              } else {
                promise.resolve(result.get());
              }
            }
          });
      auto request_id = future_and_req_id.request_id;
      node_.lock()->add_task_for(uint64_t(&promise), timeout, [this, &promise, request_id] {
        client->remove_pending_request(request_id);
        promise.reject("TIMEOUT");
      });
    });
  }

protected:
  std::weak_ptr<NodeBase> node_;

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
  ServiceClient(std::weak_ptr<NodeBase> node, const std::string &service_name,
                const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                rclcpp::CallbackGroup::SharedPtr group = nullptr)
      : impl_(std::make_shared<ServiceClientImpl<ServiceT>>(node, service_name, qos, group)) {}

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
  using GoalHandle = icey::rclcpp_action::ClientGoalHandle<ActionT>;
  using Result = typename GoalHandle::WrappedResult;
  using Client = icey::rclcpp_action::Client<ActionT>;
  using CancelResponse = typename Client::CancelResponse::SharedPtr;
  using Feedback = typename Client::Feedback;

  using ResultPromise = impl::Promise<Result, std::string>;

  AsyncGoalHandle(std::weak_ptr<NodeBase> node, std::shared_ptr<Client> client,
                  const std::shared_ptr<GoalHandle> &goal_handle)
      : node_(node), client_(client), goal_handle_(goal_handle) {}

  /// Get the goal UUID.
  const rclcpp_action::GoalUUID &get_goal_id() const { return goal_handle_->get_goal_id(); }
  /// Get the time when the goal was accepted.
  rclcpp::Time get_goal_stamp() const { return goal_handle_->get_goal_stamp(); }
  /// Get the goal status code. See
  /// https://docs.ros.org/en/ros2_packages/rolling/api/action_msgs/msg/GoalStatus.html for possible
  /// values
  int8_t get_status() { return goal_handle_->get_status(); }

  /// Obtain the result asynchronously.
  ResultPromise result(const Duration &timeout) const {
    return ResultPromise{[this, timeout](auto &promise) {
      try {
        auto request_id =
            client_.lock()->async_get_result(goal_handle_, [this, &promise](const Result &result) {
              if (node_.lock()->cancel_task_for(uint64_t(&promise))) {
                promise.resolve(result);
              }
            });
        if (!request_id) {  /// This can only happen in case the rcl/rmw function fails
          promise.reject("FAILED");
        } else {
          node_.lock()->add_task_for(uint64_t(&promise), timeout,
                                     [this, &promise, request_id = request_id.value()] {
                                       client_.lock()->cancel_result_request(request_id);
                                       promise.reject("RESULT TIMEOUT");
                                     });
        }
      } catch (const ::rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
        promise.reject(ex.what());
      }
    }};
  }

  /// Cancel this goal. Warning: Not co_awaiting this promise leads to use-after free !
  // The promise is rejected in case the goal was already reached.
  impl::Promise<CancelResponse, std::string> cancel(const Duration &timeout) const {
    return impl::Promise<CancelResponse, std::string>([this, timeout](auto &promise) {
      try {
        auto request_id = client_.lock()->async_cancel_goal(
            goal_handle_, [this, &promise](CancelResponse cancel_response) {
              if (node_.lock()->cancel_task_for(uint64_t(&promise))) {
                promise.resolve(cancel_response);
              }
            });
        /// Add timeout task
        node_.lock()->add_task_for(uint64_t(&promise), timeout, [this, &promise, request_id] {
          client_.lock()->cancel_cancellation_request(request_id);
          client_.lock()->stop_callbacks(goal_handle_);
          promise.reject("RESULT TIMEOUT");
        });

      } catch (const ::rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
        /// According to the documentation, an exception can we thrown if we try to cancel, but the
        /// goal was already reached
        promise.reject(ex.what());
      }
    });
  }

  /// Returns the held rclcpp_action::GoalHandle.
  std::shared_ptr<GoalHandle> get_goal_handle() const { return goal_handle_; }

private:
  std::weak_ptr<NodeBase> node_;
  std::weak_ptr<Client> client_;
  std::shared_ptr<GoalHandle> goal_handle_;
};

/// A action client offering an async/await API and per-request timeouts. Everything happens
/// asynchronously and returns a promise that can be awaited using co_await.
template <class ActionT>
struct ActionClient {
  using Goal = typename ActionT::Goal;
  using Client = icey::rclcpp_action::Client<ActionT>;
  using GoalHandle = icey::rclcpp_action::ClientGoalHandle<ActionT>;
  using FeedbackCallback = typename GoalHandle::FeedbackCallback;
  using Result = typename GoalHandle::WrappedResult;
  using RequestID = int64_t;
  using AsyncGoalHandleT = AsyncGoalHandle<ActionT>;

  /// Construct an async action client wrapper.
  /// @param node Node base used to create the underlying rclcpp_action client.
  /// @param action_name Name of the action topic (e.g. "/fibonacci").
  /// @param group Optional callback group for the action client waitable. If nullptr, the node's
  /// default callback group is used.
  /// @param options rcl_action client options. Defaults to
  /// rcl_action_client_get_default_options().
  ActionClient(std::weak_ptr<NodeBase> node, const std::string &action_name,
               rclcpp::CallbackGroup::SharedPtr group = nullptr,
               const rcl_action_client_options_t &options = rcl_action_client_get_default_options())
      : node_(node) {
    client_ = icey::rclcpp_action::create_client<ActionT>(
        node.lock()->get_node_base_interface(), node.lock()->get_node_graph_interface(),
        node.lock()->get_node_logging_interface(), node.lock()->get_node_waitables_interface(),
        action_name, group, options);
  }

  /// Send asynchronously an action goal: if it is accepted, then you get a goal handle.
  template <class F>
  impl::Promise<AsyncGoalHandleT, std::string> send_goal(const Goal &goal, const Duration &timeout,
                                                         F &&feedback_callback) const {
    return impl::Promise<AsyncGoalHandleT, std::string>(
        [this, goal, timeout, feedback_callback](auto &promise) {
          typename Client::SendGoalOptions options;
          options.goal_response_callback = [this, &promise](auto goal_handle) {
            if (node_.lock()->cancel_task_for(uint64_t(&promise))) {
              if (goal_handle == nullptr) {
                promise.reject("GOAL REJECTED");  /// TODO error type
              } else {
                promise.resolve(AsyncGoalHandle<ActionT>(node_, client_, goal_handle));
              }
            }
          };
          /// HINT:  Wrap inside a lambda to support feedback_callback being a coroutine
          options.feedback_callback = [feedback_callback](auto goal_handle, auto feedback) {
            feedback_callback(goal_handle, feedback);
          };
          auto request_id = client_->async_send_goal(goal, options);
          // Acceptance timeout: reject if no acceptance within timeout
          node_.lock()->add_task_for(uint64_t(&promise), timeout, [this, &promise, request_id]() {
            client_->cancel_goal_request(request_id);
            promise.reject("TIMEOUT");
          });
        });
  }

protected:
  std::weak_ptr<NodeBase> node_;
  std::shared_ptr<Client> client_;
};

/// A context that provides only async/await related entities.
class ContextAsyncAwait {
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
  explicit ContextAsyncAwait(NodeT *node) : node_base_(std::make_shared<NodeBase>(node)) {}

  /// Create a subscription and registers the given callback. The callback can be either
  /// synchronous or asynchronous. Works otherwise the same as [rclcpp::Node::create_subscription].
  template <class MessageT, class Callback>
  std::shared_ptr<rclcpp::Subscription<MessageT>> create_subscription_async(
      const std::string &topic_name, Callback &&callback,
      const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
      const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions()) {
    auto callback_wrapper = [callback = std::forward<Callback>(callback)](auto... args) {
      using ReturnType = decltype(callback(args...));
      if constexpr (!impl::HasPromiseType<ReturnType>) {
        callback(args...);
      } else {
        callback(args...).detach();
      }
    };
    auto subscription = node_base()->create_subscription<MessageT>(
        topic_name, std::move(callback_wrapper), qos, options);
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
  std::shared_ptr<rclcpp::TimerBase> create_timer_async(
      const Duration &period, Callback &&callback,
      rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    auto timer = node_base()->create_wall_timer(
        period,
        [callback = std::forward<Callback>(callback)]() {
          static_assert(std::is_invocable_v<const std::decay_t<Callback> &>,
                        "create_timer_async callback must be invocable as ()");
          using ReturnType = decltype(callback());
          if constexpr (!impl::HasPromiseType<ReturnType>) {
            callback();
          } else {
            callback().detach();
          }
        },
        group);
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
  /// icey::Promise<std::shared_ptr<ServiceT::Response>>
  template <class ServiceT, class Callback>
  std::shared_ptr<rclcpp::Service<ServiceT>> create_service(
      const std::string &service_name, Callback &&callback,
      const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
      rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    using Request = std::shared_ptr<typename ServiceT::Request>;
    using RequestID = std::shared_ptr<rmw_request_id_t>;
    /// The type of the user callback that can response synchronously (i.e. immediately): It
    /// receives the request and returns the response.
    /// For devs: For "documentation", see
    /// - https://github.com/ros2/rclcpp/pull/1709
    /// - https://github.com/ros2/rclcpp/issues/1707
    /// -
    /// [rcl_send_response](http://docs.ros.org/en/jazzy/p/rcl/generated/function_service_8h_1a8631f47c48757228b813d0849d399d81.html#_CPPv417rcl_send_responsePK13rcl_service_tP16rmw_request_id_tPv)
    auto service = node_base()->create_service<ServiceT>(
        service_name,
        [callback = std::forward<Callback>(callback)](
            std::shared_ptr<rclcpp::Service<ServiceT>> server, RequestID request_id,
            Request request) {
          using ReturnType = decltype(callback(request));
          if constexpr (!impl::HasPromiseType<ReturnType>) {
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
            continuation(server, callback, request_id, request).detach();
          }
        },
        qos, group);
    std::lock_guard<std::recursive_mutex> lock{bookkeeping_mutex_};
    services_.push_back(std::dynamic_pointer_cast<rclcpp::ServiceBase>(service));
    return service;
  }

  /// Create a service client that supports async/await.
  /// Works otherwise the same as [rclcpp::Node::create_client].
  template <class ServiceT>
  ServiceClient<ServiceT> create_client(const std::string &service_name,
                                        const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                                        rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    return ServiceClient<ServiceT>(node_base(), service_name, qos, group);
  }

  /// Create an action server with a synchronous or asynchronous callbacks.
  template <class ActionT, class GoalCallback, class CancelCallback, class AcceptedCallback>
  std::shared_ptr<icey::rclcpp_action::Server<ActionT>> create_action_server(
      const std::string &name, const GoalCallback &handle_goal, const CancelCallback &handle_cancel,
      const AcceptedCallback &handle_accepted,
      const rcl_action_server_options_t &options = rcl_action_server_get_default_options(),
      rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    using ServerGoalHandleT = icey::rclcpp_action::ServerGoalHandle<ActionT>;
    using Server = icey::rclcpp_action::Server<ActionT>;
    auto server = icey::rclcpp_action::create_server<ActionT>(
        node_base()->get_node_base_interface(), node_base()->get_node_clock_interface(),
        node_base()->get_node_logging_interface(), node_base()->get_node_waitables_interface(),
        name,
        [handle_goal](std::shared_ptr<Server> server,
                      const icey::rclcpp_action::GoalRequest<ActionT> &goal_request) {
          using ReturnType = decltype(handle_goal(goal_request.uuid, goal_request.goal));
          if constexpr (!impl::HasPromiseType<ReturnType>) {
            server->send_goal_response(goal_request,
                                       handle_goal(goal_request.uuid, goal_request.goal));
          } else {
            const auto continuation =
                [](auto server, auto handle_goal,
                   icey::rclcpp_action::GoalRequest<ActionT> goal_request) -> Promise<void> {
              server->send_goal_response(
                  goal_request, co_await handle_goal(goal_request.uuid, goal_request.goal));
              co_return;
            };
            continuation(server, handle_goal, goal_request).detach();
          }
        },
        [handle_cancel](std::shared_ptr<Server> server,
                        std::shared_ptr<ServerGoalHandleT> goal_handle,
                        const icey::rclcpp_action::CancelRequest &cancel_request) {
          using ReturnType = decltype(handle_cancel(goal_handle));
          if constexpr (!impl::HasPromiseType<ReturnType>) {
            auto response = handle_cancel(goal_handle);
            server->send_cancel_response(cancel_request, response);
          } else {
            const auto continuation = [](auto server, auto handle_cancel,
                                         icey::rclcpp_action::CancelRequest cancel_request,
                                         auto goal_handle) -> Promise<void> {
              auto response = co_await handle_cancel(goal_handle);
              server->send_cancel_response(cancel_request, response);
              co_return;
            };
            continuation(server, handle_cancel, cancel_request, goal_handle).detach();
          }
        },
        [handle_accepted](std::shared_ptr<ServerGoalHandleT> goal_handle) -> void {
          using ReturnType = decltype(handle_accepted(goal_handle));
          if constexpr (!impl::HasPromiseType<ReturnType>) {
            handle_accepted(goal_handle);
          } else {
            handle_accepted(goal_handle).detach();
          }
        },
        options, group);
    action_servers_.push_back(std::dynamic_pointer_cast<icey::rclcpp_action::ServerBase>(server));
    return server;
  }

  /// Create an action client that supports async/await send_goal.
  /// @param action_name Name of the action topic (e.g. "/fibonacci").
  /// @param group Optional callback group for the action client waitable. If nullptr, the node's
  /// default callback group is used.
  /// @param options rcl_action client options. Defaults to
  /// rcl_action_client_get_default_options().
  template <class ActionT>
  ActionClient<ActionT> create_action_client(
      const std::string &action_name, rclcpp::CallbackGroup::SharedPtr group = nullptr,
      const rcl_action_client_options_t &options = rcl_action_client_get_default_options()) {
    return ActionClient<ActionT>(node_base(), action_name, group, options);
  }

  /// Creates a transform buffer that works like the usual combination of a tf2_ros::Buffer and a
  /// tf2_ros::TransformListener. It is used to `lookup()` transforms asynchronously at a specific
  /// point in time.
  TransformBuffer create_transform_buffer() { return tf_buffer_.init(this->node_base()); }

  /// Get the NodeBase, i.e. the ROS node using which this Context was created.
  std::shared_ptr<NodeBase> node_base() { return node_base_; }

protected:
  /// The TF async interface impl
  TransformBuffer tf_buffer_;
  std::shared_ptr<NodeBase> node_base_;
  std::recursive_mutex bookkeeping_mutex_;
  std::vector<std::shared_ptr<rclcpp::TimerBase>> timers_;
  std::vector<std::shared_ptr<rclcpp::ServiceBase>> services_;
  std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;
  std::vector<std::shared_ptr<icey::rclcpp_action::ServerBase>> action_servers_;
};

}  // namespace icey
