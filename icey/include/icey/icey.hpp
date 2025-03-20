#pragma once
#include <any>
#include <boost/hana.hpp>
#include <boost/hana/ext/std/tuple.hpp>  /// Needed so that we can use std::tuple instead of custom hana tuples everywhere: https://stackoverflow.com/a/34318002
#include <boost/noncopyable.hpp>
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

#ifdef ICEY_DEGUG_RECORD_THREAD_IDS
extern std::unordered_set<std::thread::id> g_used_thread_ids;
#define ICEY_DEGUG_TRACE_THIS_THREAD_ID g_used_thread_ids.emplace(std::this_thread::get_id());
#else
#define ICEY_DEGUG_TRACE_THIS_THREAD_ID
#endif

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

/// Returns a string that represents the type of the given value: i.e. "icey::Stream<int>"
template <class T>
static std::string get_type(T &t) {
  std::stringstream ss;
  auto this_class = boost::typeindex::type_id_runtime(t).pretty_name();
  ss << "[" << this_class << " @ 0x" << std::hex << size_t(&t) << "]";
  return ss.str();
}

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

  /// These getters are needed at some places like for the ParameterEventHandler. Other functions likely require this interface
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
  auto create_client(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                  rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    return rclcpp::create_client<Service>(node_base_, node_graph_, node_services_, service_name,
                                          qos.get_rmw_qos_profile(), group);
  }

  bool is_regular_node() { return maybe_regular_node; }
  bool is_lifecycle_node() { return maybe_lifecycle_node; }
  
  /// Get the underlying (regular) rclcpp::Node from which this NodeBase was constructed
  rclcpp::Node &as_node() { 
    if(!maybe_regular_node)
      throw std::invalid_argument("This NodeBase does hold a regular node but a lifecycle node");
    return *maybe_regular_node;
  }

  /// Get the underlying rclcpp_lifecycle::LifecycleNode from which this NodeBase was constructed
  rclcpp_lifecycle::LifecycleNode &as_lifecycle_node() { 
    if(!maybe_lifecycle_node)
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

/// A subscriber + buffer for transforms that allows for asynchronous lookups and to subscribe on a
/// single transform between two coordinate systems. It is otherwise implemented similarly to the
/// tf2_ros::TransformListener but offering a well-developed asynchronous API. 
//  It works like this: Every time a new message is received on /tf, we check whether a
/// relevant transforms (i.e. ones we subscribed or ones we need to look up) was received. If yes, we notify via a callback. 
/// It is therefore an asynchronous interface to TF, similar to the tf2_ros::AsyncBuffer. However, we do not use the
/// tf2_ros::AsyncBuffer::waitFroTransform because it has a bug that we cannot make another lookup
/// in the callback of a lookup (it holds a mutex locked while calling the user callback, which is simply a wrong thing to do)  
/// This class is used to implement the TransformSubscriptionStream and the TransformBuffer.
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

  /// A request for a transform lookup: It represents effectively a single call to the async_lookup function. 
  /// Even if you call async_lookup with the exact same arguments (same frames and time), this will count as two separate requests.
  using RequestHandle = std::shared_ptr<TransformRequest>;

  explicit TFListener(NodeBase &node) : node_(node) { init(); }

  /// @brief Subscribe to a single transform between two coordinate systems target_frame and source_frame: Every time a message arrives on /tf or /tf_static, either the 
  /// the on_transform or on_error callback is called, depending on whether the lookup succeeds. 
  // The on_transform callback is called only if the transform between these two coordinate systems changed.
  void add_subscription(const GetFrame &target_frame, const GetFrame &source_frame,
                        const OnTransform &on_transform, const OnError &on_error) {
    requests_.emplace(
        std::make_shared<TransformRequest>(target_frame, source_frame, on_transform, on_error));
  }

  /// @brief Queries the TF buffer for a transform at the given time between the given frames. It does
  /// not wait but instead only returns something if the transform is already in the buffer.
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
  
  /// @brief Makes an asynchronous lookup for a for a transform at the given time between the given frames. 
  ///
  /// @param target_frame
  /// @param source_frame
  /// @param time
  /// @param timeout
  /// @param on_transform The callback that is called with the requested transform after it becomes available
  /// @param on_error The callback that is called if a timeout occurs.
  /// @return The "handle", identifying this request. You can use this handle to call `cancel_request` if you do not want that the given callbacks are called anymore.
  /// @warning Currently the timeout is measured with wall-time, i.e. does not consider sim-time due to the limitation of ROS 2 Humble only offering wall-timers
  RequestHandle async_lookup(const std::string &target_frame, const std::string &source_frame,
                             Time time, const Duration &timeout, OnTransform on_transform,
                             OnError on_error) {
    /// Clean up the cancelled timers, i.e. collect the rclcpp::TimerBase objects for timers that were
    /// cancelled since the last call to async_lookup: 
    // We need to do this kind of deferred cleanup because we would likely get a deadlock if
    // we tried to clean them up in their own callback. ("Likely" means that whether a deadlock occurs is currently an unspecified behavior, and therefore likely to change in the future.)
    cancelled_timers_.clear();
    auto request{std::make_shared<TransformRequest>(
        [target_frame]() { return target_frame; }, [source_frame]() { return source_frame; },
        on_transform, on_error, time)};
    requests_.emplace(request);
    /// TODO use non-wall timer so that this works with rosbags, it is not available in Humble
    active_timers_.emplace(request, rclcpp::create_wall_timer(
        timeout,
        [this, on_error, request = Weak(request)]() {
          active_timers_.at(request.lock())->cancel();  /// cancel this timer, (it still lives in the active_timers_)
          cancelled_timers_.emplace(active_timers_.at(request.lock())); /// Copy the timer over to the cancelled ones so that we know we need to clean it up next time
          active_timers_.erase(request.lock());  // Erase this timer from active_timers_ 
          requests_.erase(request.lock());  // Destroy the request
          on_error(tf2::TimeoutException{"Timed out waiting for transform"});
        },
        nullptr, node_.get_node_base_interface().get(), node_.get_node_timers_interface().get()));
    return request;
  }

  /// Cancel a transform request: This means that the registered callbacks will no longer be called. If the given request does not exist, this function
  /// does nothing. 
  bool cancel_request(RequestHandle request) { return requests_.erase(request); }

  /// We take a tf2_ros::Buffer instead of a tf2::BufferImpl only to be able to use ROS-time API
  /// (internally TF2 has it's own timestamps...), not because we need to wait on anything (that's
  /// what tf2_ros::Buffer does in addition to tf2::BufferImpl).
  std::shared_ptr<tf2_ros::Buffer> buffer_;
protected:
  void init() {
    init_tf_buffer();
    message_subscription_tf_ = node_.create_subscription<tf2_msgs::msg::TFMessage>("/tf", 
        [this](TransformsMsg msg) { on_tf_message(msg, false); }, tf2_ros::DynamicListenerQoS());
    message_subscription_tf_static_ = node_.create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
        [this](TransformsMsg msg) { on_tf_message(msg, true); }, tf2_ros::StaticListenerQoS());
  }

  void init_tf_buffer() {
    /// This code is a bit tricky. It's about asynchronous programming essentially. The official
    /// example is rather incomplete
    /// ([official](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html))
    /// . I'm following this example instead:
    /// https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/src/imu_transformer.cpp#L16
    /// See also the following discussions:
    /// https://answers.ros.org/question/372608/?sort=votes
    /// https://github.com/ros-navigation/navigation2/issues/1182
    /// https://github.com/ros2/geometry2/issues/446
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_node_clock_interface()->get_clock());
    auto timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(node_.get_node_base_interface(), node_.get_node_timers_interface());
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
      /// If the request is destroyed gracefully(because the lookup succeeded), destroy (and therefore cancel)
      /// the associated timer:
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
  
  NodeBase &node_;  /// Hold weak reference to the NodeBase because the Context owns the NodeBase as well, so we avoid circular reference

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_;

  std::unordered_set<RequestHandle> requests_;
  /// The timeout timers for every lookup transform request: These are only the active timers, i.e. the timeout timers for pending requests.
  std::unordered_map<RequestHandle, std::shared_ptr<rclcpp::TimerBase>> active_timers_;
  /// A separate hashset for cancelled timers so that we know immediatelly which are cancelled.
  std::unordered_set<std::shared_ptr<rclcpp::TimerBase>> cancelled_timers_;
};

/// An improved server client that implements per-request timeouts, something currently missing in rclcpp.
template <class ServiceT>
struct ServiceClientImpl {
  using Request = typename ServiceT::Request::SharedPtr;
  using Response = typename ServiceT::Response::SharedPtr;
  
  /// The RequestID is currently of type int64_t in rclcpp.
  using RequestID = decltype(rclcpp::Client<ServiceT>::FutureAndRequestId::request_id);
  using Client = rclcpp::Client<ServiceT>;

  ServiceClientImpl(NodeBase &node, 
    const std::string &service_name,
    const rclcpp::QoS &qos = rclcpp::ServicesQoS()) : node_(node),
    client(node_.create_client<ServiceT>(service_name, qos)) {}

  /// Make an asynchronous call to the service
  RequestID call(Request request, const Duration &timeout, auto on_response, auto on_error) {
    /// Clean up the cancelled timers, i.e. collect the rclcpp::TimerBase objects for timers that were
    /// cancelled since the last call: 
    // We need to do this kind of deferred cleanup because we would likely get a deadlock if
    // we tried to clean them up in their own callback. ("Likely" means that whether a deadlock occurs is currently an unspecified behavior, and therefore likely to change in the future.)
    cancelled_timers_.clear();
    /// TODO we cannot wait for the service synchronously because this will call the callback and
    /// therefore the coroutine continuation directly (i.e. synchronously) instead in the event
    /// loop, eventually leading to a stack overflow
    /*if(!client->wait_for_service(timeout)) {
      if (!rclcpp::ok()) {
        future.put_error("INTERRUPTED");
      } else {
        future.put_error("SERVICE_UNAVAILABLE");
      }
      return Cancel{};
    } else {*/
    auto req_id = std::make_shared<RequestID>();
    auto future_and_req_id =
          client->async_send_request(request, [this, on_response, on_error, req_id](typename Client::SharedFuture result) {
            if (!result.valid()) {
              if (!rclcpp::ok()) {
                on_error("INTERRUPTED");
              } else {
                on_error("TIMEOUT");
              }
            } else {
              /// Cancel and erase the timeout timer since we got a response
              active_timers_.erase(*req_id);
              on_response(result.get());
            }
        });
    *req_id = future_and_req_id.request_id;

    active_timers_.emplace(future_and_req_id.request_id,
        node_.create_wall_timer(timeout, [this, on_error, request_id=future_and_req_id.request_id] {
            client->remove_pending_request(request_id);
            active_timers_.at(request_id)->cancel();
            cancelled_timers_.emplace(active_timers_.at(request_id));
            active_timers_.erase(request_id);
            if (!rclcpp::ok()) {
              on_error("INTERRUPTED");
            } else {
              on_error("TIMEOUT");
            }
      }));
    return future_and_req_id.request_id;
  }

  bool cancel_request(RequestID request_id) {
    client->remove_pending_request(request_id);
    return active_timers_.erase(request_id);
  }
  
  NodeBase &node_;
  std::shared_ptr<rclcpp::Client<ServiceT>> client;
protected:

  /// The rclcpp API does not give us a request id inside the callback, so we need to map the request to the ID to be able to cancel the timeout timers
  std::unordered_map<Request, RequestID> request_to_id_;
  std::unordered_map<RequestID, Request> request_id_to_request_;
  /// The timeout timers for every lookup transform request: These are only the active timers, i.e. the timeout timers for pending requests.
  std::unordered_map<RequestID, std::shared_ptr<rclcpp::TimerBase>> active_timers_;
  /// A separate hashset for cancelled timers so that we know immediatelly which are cancelled.
  std::unordered_set<std::shared_ptr<rclcpp::TimerBase>> cancelled_timers_;
};

/// A tag to be able to recognize the type "Stream", all types deriving from StreamTag satisfy the
/// `AnyStream` concept. \sa AnyStream
struct StreamTag {};

template <class T>
constexpr bool is_stream = std::is_base_of_v<StreamTag, T>;

/// A stream type with any error or value type. \sa StreamTag
template <class T>
concept AnyStream = std::is_base_of_v<StreamTag, T>;

/// A stream type is error-free, meaning its Error is Nothing. It's value should not be Nothing
/// because this does not make any sense.
template <class T>
concept ErrorFreeStream =
    AnyStream<T> && std::is_same_v<ErrorOf<T>, Nothing> && !std::is_same_v<ValueOf<T>, Nothing>;

template <class V>
struct Validator;

template <class V>
struct ParameterStream;

template <class Value>
struct ValueOrParameter;

template <class V>
struct SubscriptionStream;
struct TimerStream;
template <class V>
struct PublisherStream;

template <class V>
struct ServiceStream;

template <class V>
struct ServiceClient;
template <class V>
struct TimeoutFilter;
struct TransformBuffer;
struct TransformSubscriptionStream;
template <class V>
struct Buffer;
template <class V>
struct TransformSynchronizer;
struct TransformPublisherStream;

/// The context is what is returned when calling `node->icey()` (NodeWithIceyContext::icey). 
/// It provides an new node-like API for creating ROS entities, these entities are however streams. 
//  It also provides bookkeeping i.e. holding the shared pointers to subscribers/timers/publishers etc., so that you do not have to do it.
class Context : public NodeBase,
                public std::enable_shared_from_this<Context>,
                private boost::noncopyable {
public:
  /// The parameter validation function that allows the parameter update if the returned string is
  /// empty (i.e. "") and otherwise rejects with the error message.
  using FValidate = std::function<std::string(const rclcpp::Parameter &)>;
  ///
  template <class V>
  using GetValue = std::function<V()>;

  /// Constructs a Context from NodeBase so that both a rclcpp::Node as well as a
  /// lifecycle node are supported.
  explicit Context(const NodeBase &node_base) : NodeBase(node_base) {}

  /// Creates a new stream of type S by passing the args to the constructor. It adds the impl to the
  /// list of streams so that it does not go out of scope. It also sets the context.
  template <AnyStream S, class... Args>
  S create_stream(Args &&...args) {
    S stream(*this, std::forward<Args>(args)...);
    /// Track (i.e. reference) the Stream impl so that it does not go out of scope.
    stream.impl()->context = this->shared_from_this();
    return stream;
  }

  /// Creates a new stream impl and adds it to the list of stream impls so that it does not go out of scope.
  template <class StreamImpl>
  Weak<StreamImpl> create_stream_impl() {
    auto impl = impl::create_stream<StreamImpl>();
    stream_impls_.push_back(impl);
    return impl;
  }
  
  /// Get the NodeBase, i.e. the ROS node using which this Context was created.
  NodeBase &node_base() { return static_cast<NodeBase&>(*this); }

  /// Declares a single parameter to ROS and register for updates. The ParameterDescriptor is
  /// created automatically matching the given Validator.
  /// \sa For more detailed documentation:
  /// [rclcpp::Node::declare_parameter](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4I0EN6rclcpp4Node17declare_parameterEDaRKNSt6stringERK10ParameterTRKN14rcl_interfaces3msg19ParameterDescriptorEb)
  template <class ParameterT>
  ParameterStream<ParameterT> declare_parameter(const std::string &parameter_name,
                                                const ParameterT &default_value,
                                                const Validator<ParameterT> &validator = {},
                                                std::string description = "",
                                                bool read_only = false,
                                                bool ignore_override = false);

   // clang-format off
  /*!
  \brief Declare a given parameter struct to ROS.
  \tparam ParameterStruct the type of the parameter struct. It must be a struct/class with fields of
  either a primitive type supported by ROS (e.g. `double`) or a `icey::ParameterStream`, or another
  (nested) struct with more such fields.

  \param params The instance of the parameter struct where the values will be written to.
  \param notify_callback The callback that gets called when any field changes
  \param name_prefix Prefix for each parameter. Used by the recursive call to support nested structs. 
  \note The passed object `params` must have the same lifetime as the node, best is to
  store it as a member of the node class.

  Example usage:
  \verbatim
    /// Here you declare in a single struct all parameters of the node:
    struct NodeParameters {
      /// We can have regular fields :
      double amplitude{3};

      /// And as well parameters with constraints and a description:
      icey::Parameter<double> frequency{10., icey::Interval(0., 25.),
                                          std::string("The frequency of the sine")};

      icey::Parameter<std::string> mode{"single", icey::Set<std::string>({"single", "double", "pulse"})};

      /// We can also have nested structs with more parameters, they will be named others.max_amp, others.cov: 
      struct OtherParams { 
        double max_amp = 6.; 
        std::vector<double> cov; 
      } others;
    };

    class MyNode : public icey::Node {
      MyNode(std::string name): icey::Node(name) {
          /// Now simply declare the parameter struct and a callback that is called when any field updates: 
          this->icey().declare_parameter_struct(params_, [&](const std::string &changed_parameter) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Parameter " << changed_parameter << " changed");
          });
      }
      // Hold the parameter struct inside the class:
      NodeParameters params_;
    };
  \endverbatim
  */
  // clang-format on
  template <class ParameterStruct>
  void declare_parameter_struct(
      ParameterStruct &params, const std::function<void(const std::string &)> &notify_callback = {},
      std::string name_prefix = "");

  /// Create a subscription stream.
  /// Works otherwise the same as [rclcpp::Node::create_subscription].
  template <class MessageT>
  SubscriptionStream<MessageT> create_subscription(
      const std::string &name, const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
      const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions());

  /// Create a subscriber that subscribes to a single transform between two frames.
  /// This stream will emit a value every time the transform between these two frames changes, i.e.
  /// it is a stream. If you need to lookup transforms at a specific point in time, look instead at
  /// `Context::create_transform_buffer`.
  TransformSubscriptionStream create_transform_subscription(
      ValueOrParameter<std::string> target_frame, ValueOrParameter<std::string> source_frame);

  /// Creates a transform buffer that works like the usual combination of a tf2_ros::Buffer and a
  /// tf2_ros::TransformListener. It is used to `lookup()` transforms asynchronously at a specific
  /// point in time.
  TransformBuffer create_transform_buffer();

  /// Create a publisher stream.
  /// Works otherwise the same as [rclcpp::Node::create_publisher].
  template <class Message>
  PublisherStream<Message> create_publisher(const std::string &topic_name,
                                            const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
                                            const rclcpp::PublisherOptions publisher_options = {});

  /// Create a publisher to publish transforms on the `/tf` topic. Works otherwise the same as
  /// [tf2_ros::TransformBroadcaster].
  TransformPublisherStream create_transform_publisher();

  /// Create a timer stream. It yields as a value the total number of ticks.
  /// \param period the period time
  /// \param is_one_off_timer if set to true, this timer will execute only once.
  /// Works otherwise the same as [rclcpp::Node::create_timer].
  TimerStream create_timer(const Duration &period, bool is_one_off_timer = false);

  /// Create a service server stream. One a request is received, the provided callback will be
  /// called. The callback can be either synchronous (a regular function) or asynchronous, i.e. a
  /// coroutine. The callbacks returns the response. Works otherwise the same as
  /// [rclcpp::Node::create_service].
  template <class ServiceT, class Callback>
  ServiceStream<ServiceT> create_service(const std::string &service_name, Callback &&callback,
                                         const rclcpp::QoS &qos = rclcpp::ServicesQoS());

  /// Create a service server stream. No callback is provided, therefore requests must be awaited
  /// and then a response be send manually by calling respond on the service. Works otherwise the
  /// same as [rclcpp::Node::create_service].
  template <class ServiceT>
  ServiceStream<ServiceT> create_service(const std::string &service_name,
                                         const rclcpp::QoS &qos = rclcpp::ServicesQoS());

  /// Create a service client.
  /// Works otherwise the same as [rclcpp::Node::create_client].
  template <class ServiceT>
  ServiceClient<ServiceT> create_client(const std::string &service_name,
                                        const rclcpp::QoS &qos = rclcpp::ServicesQoS());

  /// Declares a parameter and registers a validator callback and a callback that will get called
  /// when the parameters updates.
  template <class ParameterT, class CallbackT>
  auto add_parameter(const std::string &name, const ParameterT &default_value,
                     CallbackT &&update_callback,
                     const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor = {},
                     FValidate f_validate = {}, bool ignore_override = false) {
    parameter_validators_.emplace(name, f_validate);
    add_parameter_validator_if_needed();
    auto param = node_parameters_->declare_parameter(name, rclcpp::ParameterValue(default_value),
                                                     parameter_descriptor, ignore_override);
    auto param_subscriber =
        std::make_shared<rclcpp::ParameterEventHandler>(static_cast<NodeBase &>(*this));
    auto cb_handle =
        param_subscriber->add_parameter_callback(name, std::forward<CallbackT>(update_callback));
    parameters_.emplace(name, std::make_pair(param_subscriber, cb_handle));
    return param;
  }

  /// Subscribe to a transform on tf between two frames
  template <class OnTransform, class OnError>
  auto add_tf_subscription(GetValue<std::string> target_frame, GetValue<std::string> source_frame,
                           OnTransform &&on_transform, OnError &&on_error) {
    add_tf_listener_if_needed();
    tf2_listener_->add_subscription(target_frame, source_frame, on_transform, on_error);
    return tf2_listener_;
  }
  
  auto add_tf_broadcaster_if_needed() {
    if (!tf_broadcaster_)
      tf_broadcaster_ =
          std::make_shared<tf2_ros::TransformBroadcaster>(static_cast<NodeBase &>(*this));
    return tf_broadcaster_;
  }

  std::shared_ptr<TFListener> add_tf_listener_if_needed() {
    if (!tf2_listener_) {
      /// We need only one subscription on /tf, but we can have multiple transforms on which we
      /// listen to
      tf2_listener_ = std::make_shared<TFListener>(static_cast<NodeBase &>(*this));
    }
    return tf2_listener_;
  }
protected:
  /// The implementation of rclcpp::Node does not consistently call the free functions (i.e. rclcpp::create_*)
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
    this->validate_param_cb_ = node_parameters_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          for (const auto &parameter : parameters) {
            /// We want to skip validating parameters that we didn't declare, for example here we
            /// are getting called for parameters like "qos_overrides./tf.publisher.durability"
            if (parameter_validators_.contains(parameter.get_name())) {
              result.reason = parameter_validators_.at(parameter.get_name())(parameter);
            }
          }
          result.successful = result.reason == "";
          return result;
        });
  }

  /// A map that stores for each parameter name some ROS entities that we need to hold to be able to
  /// receive parameter updates.
  std::unordered_map<std::string, std::pair<std::shared_ptr<rclcpp::ParameterEventHandler>,
                                            std::shared_ptr<rclcpp::ParameterCallbackHandle>>>
      parameters_;

  /// Parameter validators for each parameter name, containing the values of a parameter.
  std::unordered_map<std::string, FValidate> parameter_validators_;

  /// Callback for validating a list of parameter.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_param_cb_;
  /// TF Support
  std::shared_ptr<TFListener> tf2_listener_;
  /// This is a simple wrapper around a publisher that publishes on /tf. There is really nothing
  /// interesting under the hood of tf2_ros::TransformBroadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  /// All the streams that were created, they correspond mostly to one ROS entity like sub/pub/timer
  /// etc.
  std::vector<std::shared_ptr<impl::StreamImplBase>> stream_impls_;
};

/// The default augmentation of a stream implementation: The context and a timeout.
class StreamImplDefault {
public:
  StreamImplDefault() {}
  /// A weak reference to the Context, it is needed so that Streams can create more streams that
  /// need access to the ROS node, i.e. via `.publish`.
  std::weak_ptr<Context> context;
  /// Timeout is useful when using co_await since we can implement Stream timeouts without an extra
  /// timer.
  std::optional<Duration> timeout{};

  /// The coroutine that will be called once after this stream has a value
  std::coroutine_handle<> continuation_;
  bool registered_continuation_callback_{false};
};

/// Adds a default extention inside the impl::Stream by default.
/// Handy to not force the user to declare this, i.e. to not leak implementation details.
template <class Base>
struct WithDefaults : public Base, public StreamImplDefault {};

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

template <class F, class Arg>
struct check_callback {
  static_assert(std::is_invocable_v<F, Arg>, "The callback has the wrong signature");
  using ReturnType = std::invoke_result_t<F, Arg>;
};

template <class F, class... Args>
struct check_callback<F, std::tuple<Args...>> {
  static_assert(std::is_invocable_v<F, Args...>,
                "The callback has the wrong signature, it has to take multiple arguments (the "
                "Stream holds tuple, the arguments are all the elements of the tuple)");
  using ReturnType = std::invoke_result_t<F, Args...>;
};

struct PromiseTag {};
/// A Promise is an asynchronous abstraction that yields a single value or an error.
/// I can be used with async/await syntax coroutines in C++20.
/// It also allows for wrapping an existing callback-based API.
/// It does not use dynamic memory allocation to store the value.

/// For references, see the promise implementations of this library:
/// [asyncpp] https://github.com/asyncpp/asyncpp/blob/master/include/asyncpp/promise.h
///
/// To some extent, you can also look at the "task" abstraction as a reference. But note that tasks
/// were developed to solve very specific kind of problems that arise when developing *concurrent*
/// asynchronous code, see Lewis Bakers' talk at CppCon 2019
/// (https://www.youtube.com/watch?v=1Wy5sq3s2rg), chapter "The Solution". Since we do not have a
/// concurrent application but a single-threaded event-loop, the Promise is sufficient for us and we
/// not need this "task". 
///
/// - [cppcoro task](https://github.com/lewissbaker/cppcoro/blob/master/include/cppcoro/task.hpp#L456)
/// - [asyncpp](https://github.com/asyncpp/asyncpp/blob/master/include/asyncpp/task.h#L23)
/// - [libcoro](https://github.com/jbaldwin/libcoro/blob/main/include/coro/task.hpp)
template <class _Value = Nothing, class _Error = Nothing>
class PromiseBase : public PromiseTag, public impl::Stream<_Value, _Error, Nothing, Nothing> {
public:
  using Value = _Value;
  using Error = _Error;
  using State = impl::Stream<_Value, _Error, Nothing, Nothing>::State;
  using Self = PromiseBase<Value, Error>;
  using promise_type = Self;
  using Cancel = std::function<void(Self &)>;

  PromiseBase() {
    if (icey_coro_debug_print)
      std::cout << "Promise was default-constructed: " << get_type(*this) << std::endl;
  }
  /// Construct using a handler: This handler is called immediately in the constructor with the
  /// address to this Promise so that it can store it and write to this promise later. This handler
  /// returns a cancellation function that gets called when this Promise is destructed. This
  /// constructor is useful for wrapping an existing callback-based API.
  explicit PromiseBase(std::function<Cancel(Self &)> &&h) {
    if (icey_coro_debug_print) std::cout << "Promise(h) @  " << get_type(*this) << std::endl;
    cancel_ = h(*this);
  }

  std::suspend_never initial_suspend() { return {}; }
  std::suspend_always final_suspend() const noexcept { return {}; }

  /// Store the unhandled exception in case it occurs: We will re-throw it when it's time. (The
  /// compiler can't do this for us because of reasons)
  void unhandled_exception() { exception_ptr_ = std::current_exception(); }

  /// Await the promise
  auto operator co_await() {
    struct Awaiter {
      Self &promise_;
      Awaiter(Self &fut) : promise_(fut) {}
      bool await_ready() const noexcept {
        if (icey_coro_debug_print)
          std::cout << "Await ready on Promise " << get_type(promise_) << " called" << std::endl;
        return !promise_.has_none();
      }
      void await_suspend(std::coroutine_handle<> continuation) noexcept {
        /// Resume the coroutine when this promise is done
        if (icey_coro_debug_print)
          std::cout << "Await suspend was called, held Promise: " << get_type(promise_)
                    << std::endl;
        promise_.register_handler([continuation](auto) {
          if (continuation) continuation.resume();
        });
      }

      auto await_resume() const noexcept {
        if (icey_coro_debug_print)
          std::cout << "Await resume was called, held Promise: " << get_type(promise_) << std::endl;
        if (promise_.exception_ptr_) std::rethrow_exception(promise_.exception_ptr_);
        return promise_.get_state().get();
      }
    };
    return Awaiter{*this};
  }

  ~PromiseBase() {
    if (icey_coro_debug_print)
      std::cout << "Promise was destructed: " << get_type(*this) << std::endl;
    if (cancel_) cancel_(*this);
  }

  /// A synchronous cancellation function. It unregisters for example a ROS callback so that it is
  /// not going to be called anymore. Such cancellations are needed because the ROS callback
  /// captures the Promises object by reference since it needs to write the result to it. But if we
  /// would destruct the Promise and after that this ROS callback gets called, we get an
  /// use-after-free bug. The promise must therefore cancel the ROS callback in the destructor.
  /// Since we have no concurrency, this cancellation function can always be a synchronous function
  /// and thus simply be called in the destructor.
  Cancel cancel_;
  std::exception_ptr exception_ptr_{nullptr};
};

/// The whole point of this PromiseBase and Promise is the design of the operator co_return that
/// requires two differently named operators, depending on whether a Promise holds a value or not
/// (is the void-version) first one is called return_value, second one is called return_void: But
/// choosing between return_value and return_void does not work with SFINAE (Reference:
/// https://devblogs.microsoft.com/oldnewthing/20210330-00/?p=105019) So this is the non-void
/// version:
template <class _Value, class _Error = Nothing>
class Promise : public PromiseBase<_Value, _Error> {
public:
  using Base = PromiseBase<_Value, _Error>;
  using State = Base::State;
  using Base::Base;
  using promise_type = Promise<_Value, _Error>;
  promise_type get_return_object() { return {}; }

  auto return_value() { return this->value(); }
  /// return_value (aka. operator co_return) *sets* the value if called with an argument,
  /// very confusing, I know
  /// (Reference: https://devblogs.microsoft.com/oldnewthing/20210330-00/?p=105019)
  void return_value(const _Value &x) {
    if (icey_coro_debug_print) std::cout << get_type(*this) << " setting value " << std::endl;
    this->put_value(x);
  }

  /// Sets the state to the given one:
  void return_value(const State &x) {
    if (icey_coro_debug_print) std::cout << get_type(*this) << " setting state " << std::endl;
    this->put_state(x);
  }
};

/// And this is the void version:
template <>
class Promise<void, Nothing> : public PromiseBase<Nothing, Nothing> {
public:
  using PromiseBase<Nothing, Nothing>::PromiseBase;
  using promise_type = Promise<void, Nothing>;
  promise_type get_return_object() { return {}; }
  auto return_void() { this->put_value(Nothing{}); }
};

/// An awaiter required to implement operator co_await for Streams. (C++ coroutines)
template <class S>
struct Awaiter {
  S &stream;
  Awaiter(S &s) : stream(s) {}
  bool await_ready() const noexcept {
    if (icey_coro_debug_print)
      std::cout << "Await ready on Stream " << get_type(stream) << " called" << std::endl;
    return !stream.impl()->has_none();
  }
  void await_suspend(std::coroutine_handle<> continuation) noexcept {
    if (icey_coro_debug_print)
      std::cout << "Await suspend on Stream " << get_type(stream) << " called" << std::endl;
    if (!stream.impl()->registered_continuation_callback_) {
      stream.impl()->register_handler([impl = stream.impl()](auto &) {
        /// Important: Call the continuation only once since we may be awaiting the stream only
        /// once
        if (impl->continuation_) {
          auto c = std::exchange(impl->continuation_,
                                 nullptr);  /// Get the continuation and replace it with nullptr
                                            /// so that it is called only once
          c.resume();
        }
      });
      stream.impl()->registered_continuation_callback_ = true;
    }
    stream.impl()->continuation_ = continuation;
  }
  auto await_resume() const noexcept {
    if (icey_coro_debug_print)
      std::cout << "Await resume on Stream " << get_type(stream) << " called" << std::endl;
    if (stream.exception_ptr_)  /// [Coroutine support] The coroutines are specified so that the
                                /// compiler does not do exception handling
      /// everywhere, so they put the burden on the implementers to defer throwing the exception
      std::rethrow_exception(stream.exception_ptr_);
    return stream.impl()->take();
  }
};

/// \brief A stream, an abstraction over an asynchronous sequence of values.
/// It has a state of type Result and a list of callbacks that get notified when this state changes.
/// It is conceptually very similar to a promise in JavaScript except that state transitions are not
/// final. This is the base class for all the other streams.
///
/// \tparam _Value the type of the value
/// \tparam _Error the type of the error. It can also be an exception.
/// \tparam ImplBase a class from which the implementation (impl::Stream) derives, used as an
/// extention point. \note This class does not any fields other than a pointer to the actual
/// implementation, `std::shared_ptr<Impl>`, i.e. it uses the PIMPL idiom. When deriving from this
/// class to implement new Streams, you should never add additional fields because this Stream may
/// go out of scope. Instead, put the additional fields that you need in a separate struct
/// `MyStreamImpl` and pass it as the `ImplBase` template parameter. Then, these fields become
/// available through `impl().<my_field>`, i.e. the Impl-class will derive from ImplBase. This is
/// how you should extend the Stream class when implementing your own Streams.
template <class _Value, class _Error = Nothing, class ImplBase = Nothing>
class Stream : public StreamTag {
  static_assert(std::is_default_constructible_v<ImplBase>,
                "ImplBase must be default constructable");
  friend Context;
  friend Awaiter<Stream<_Value, _Error, ImplBase>>;
public:
  using Value = _Value;
  using Error = _Error;
  using Self = Stream<_Value, _Error, ImplBase>;
  /// The actual implementation of the Stream
  using Impl = impl::Stream<Value, Error, WithDefaults<ImplBase>, WithDefaults<Nothing>>;
  /// [Coroutine support]
  using promise_type = Self;

#ifdef ICEY_DEBUG_PRINT_STREAM_ALLOCATIONS
  Stream() { std::cout << "Created new Stream: " << get_type(*this) << std::endl; }
  ~Stream() { std::cout << "Destroying Stream: " << get_type(*this) << std::endl; }
#else
  Stream() = default;
#endif

  /// Create s new stream using the context.
  explicit Stream(Context &ctx): impl_(ctx.create_stream_impl<Impl>()) {}
  explicit Stream(std::shared_ptr<Impl> impl) : impl_(impl) {}

  /// [Coroutine support]
  Self &get_return_object() { return *this; }
  /// [Coroutine support]
  std::suspend_never initial_suspend() { return {}; }
  /// [Coroutine support]
  std::suspend_never final_suspend() const noexcept { return {}; }
  /// [Coroutine support] return_value returns the value of the Steam.
  auto return_value() { return this->value(); }
  /// [Coroutine support] Store the unhandled exception in case it occurs: We will re-throw it when
  /// it's time. (The compiler can't do this for us because of reasons)
  void unhandled_exception() { exception_ptr_ = std::current_exception(); }

  /// [Coroutine support] Allow this stream to be awaited with `co_await stream` in C++20 coroutines.
  /// It spins the ROS executor until this Stream has something (value or error) and then resumes
  /// the coroutine.
  Awaiter<Self> operator co_await() { return Awaiter{*this}; }

  /// [Coroutine support] Implementation of the operator co_return(x): It *sets* the value of the
  /// Stream object that is about to get returned (the compiler creates it beforehand)
  void return_value(const Value &x) {
    if (icey_coro_debug_print)
      std::cout << get_type(*this) << " setting value using operator co_return for "
                << boost::typeindex::type_id_runtime(x).pretty_name() << " called " << std::endl;
    this->impl()->set_value(x);
  }

  /// Returns a weak pointer to the implementation.
  Weak<Impl> impl() const { return impl_; }

  /// \brief Calls the given function f every time this stream receives a value.  /// It returns a
  /// new stream that receives the values that this function f returns. The returned Stream also
  /// passes though the errors of this stream so that chaining `then`s with an `except` works. \note
  /// The given function must be synchronous, no asynchronous functions are supported. \returns A
  /// new Stream that changes it's value to y every time this stream receives a value x, where y =
  /// f(x). The type of the returned stream is:
  /// - Stream<Nothing, _Error> if F is (X) -> void
  /// - Stream<NewValue, NewError> if F is (X) -> Result<NewValue, NewError>
  /// - Stream<NewValue, _Error> if F is (X) -> std::optional<NewValue>
  /// - Stream<Y, _Error> otherwise
  /// \tparam F: Must be (X) -> Y, where X is:
  ///  - V_1, ..., V_n if Value is std::tuple<V_1, ..., V_n>
  ///  - Value otherwise
  template <class F>
  auto then(F &&f) {
    check_callback<F, Value>{};
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream cannot have values, so you cannot register then() on it.");
    return create_from_impl(impl()->then(std::forward<F>(f)));
  }

  /// \brief Calls the given function f every time this Stream receives an error.
  /// It returns a new Stream that receives the values that this function f returns.
  /// \note The given function must be synchronous, no asynchronous functions are supported.
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
    check_callback<F, Error>{};
    static_assert(!std::is_same_v<Error, Nothing>,
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

  /*!
    \brief Creates a publisher so that every new value of this Stream will get published.
    \sa PublisherStream
  */
  void publish(const std::string &topic_name, const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
               const rclcpp::PublisherOptions publisher_options = {}) {
    assert_we_have_context();
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream does not have a value, there is nothing to publish, so you cannot "
                  "call publish() on it.");
    this->template create_stream<PublisherStream<Value>>(topic_name, qos, publisher_options, this);
  }

  /*!
    \brief Creates a custom publisher so that every new value of this Stream will get published.
    \sa PublisherStream
  */
  template <AnyStream PublisherType, class... Args>
  void publish(Args &&...args) {
    assert_we_have_context();
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream does not have a value, there is nothing to publish, so you cannot "
                  "call publish() on it.");
    /// We create this through the context to register it for attachment to the ROS node
    auto output = this->template create_stream<PublisherType>(std::forward<Args>(args)...);
    this->connect_values(output);
  }

  /*!
    \brief Creates a transform publisher so that every new value of this Stream, which must be of
    type `geometry_msgs::msg::TransformStamped`, will get published. \sa TransformPublisherStream
  */
  void publish_transform() {
    assert_we_have_context();
    static_assert(std::is_same_v<Value, geometry_msgs::msg::TransformStamped>,
                  "The stream must hold a Value of type "
                  "geometry_msgs::msg::TransformStamped[::SharedPtr] to be able to call "
                  "publish_transform() on it.");
    this->template create_stream<TransformPublisherStream>(this);
  }

  /// Unpacks an Stream holding a tuple as value to multiple Streams for each tuple element.
  /// Given that `Value` is of type `std::tuple<Value1, Value2, ..., ValueN>`, it returns
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

  /// Unwraps, i.e. creates an ErrorFreeStream by handling the error with the given function f.
  /// The returned Stream will receive only the values of this stream.
  /// \tparam F Function receiving the Error of this
  /// Stream (it is unpacked if it's a tuple) and returning void.
  template <class F>
  Stream<Value, Nothing> unwrap_or(F &&f) {
    /// TODO Can't we just move this stream in, i.e. reuse it instead of creating a new one ? (In
    /// this case moving means re-using the impl)
    auto output = this->template create_stream<Stream<Value, Nothing>>();
    this->impl()->register_handler(
        [output_impl = output.impl(), f = std::forward<F>(f)](const auto &new_state) {
          if (new_state.has_value()) {
            output_impl->put_value(new_state.value());
          } else if (new_state.has_error()) {
            impl::unpack_if_tuple(f, new_state.error());
          }
        });
    return output;
  }

  /// Outputs the Value only if the given predicate f returns true.
  /// \tparam F Function receiving the Value of this Stream (it is unpacked if it's a tuple) and
  /// returning bool
  template <class F>
  Stream<Value, Error> filter(F f) {
    return this->then([f = std::move(f)](auto x) -> std::optional<Value> {
      if (!f(x))
        return {};
      else
        return x;
    });
  }

  /// Buffers N elements. Each time N elements were accumulated, this Stream will yield a value of
  /// type std::vector<Value> with exactly N elements.
  Buffer<Value> buffer(std::size_t N) const {
    return this->template create_stream<Buffer<Value>>(N, *this);
  }

  /// \return A new Stream that errors on a timeout, i.e. when this stream has not received any
  /// value for some time `max_age`.
  /// \param max_age the maximum age a message is allowed to have before the timeout occurs
  /// \param create_extra_timer If set to false, the timeout will
  /// only be detected after at least one message was received. If set to true, an extra timer is
  /// created so that timeouts can be detected even if no message is received.
  TimeoutFilter<Value> timeout(const Duration &max_age, bool create_extra_timer = true) {
    assert_we_have_context();
    /// We create this through the context to register it for the ROS node
    return this->template create_stream<TimeoutFilter<Value>>(*this, max_age, create_extra_timer);
  }

  // clang-format off
  /*!
    \brief Synchronizes a topic with a transform using the `tf2_ros::MessageFilter`.
    \param target_frame the transform on which we wait is specified by source_frame and target_frame, where source_frame is the frame in the header of the message. 
    \param lookup_timeout The maximum time to wait until the transform gets available for a message
    
    Example:
    \verbatim
      /// Synchronize with a transform: This will yield the message and the transform from the child_frame_id of the header message
      /// and the given target_frame ("map") at the time of the header stamp. It will wait up to 200ms for the transform. 
      node->icey().create_subscription<sensor_msgs::msg::Image>("camera")
        .synchronize_with_transform("map", 200ms)
        .unwrap_or([&](std::string error) { RCLCPP_INFO_STREAM(node->get_logger(), "Transform lookup error: " << error);})
        .then([](sensor_msgs::msg::Image::SharedPtr image, 
            const geometry_msgs::msg::TransformStamped &transform_to_map) {

        });
    \endverbatim
  */
  // clang-format on
  TransformSynchronizer<Value> synchronize_with_transform(const std::string &target_frame,
                                                          const Duration &lookup_timeout) {
    assert_we_have_context();
    static_assert(!std::is_same_v<Value, Nothing>, "");
    /// TODO assert has header
    return this->template create_stream<TransformSynchronizer<Value>>(target_frame, lookup_timeout,
                                                                      this);
  }

  /// Creates a new stream of type S using the Context. See Context::create_stream
  template <AnyStream S, class... Args>
  S create_stream(Args &&...args) const {
    return this->impl()->context.lock()->template create_stream<S>(std::forward<Args>(args)...);
  }

protected:
  void assert_we_have_context() {
    if (!this->impl()->context.lock())
      throw std::runtime_error("This stream does not have context");
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
  Weak<Impl> impl_{nullptr};
  std::exception_ptr exception_ptr_{nullptr};
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
  template <ErrorFreeStream Input>
  explicit Buffer(Context &context, std::size_t N, Input input) : Base(context) {
    this->impl()->N = N;
    input.impl()->register_handler([impl = this->impl()](auto x) {
      impl->buffer->push_back(x.value());
      if (impl->buffer->size() == impl->N) {
        impl->put_value(impl->buffer);
        impl->buffer->clear();
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
/// and less or equal to a maximum value.
template <class Value>
struct Interval {
  static_assert(std::is_arithmetic_v<Value>, "The value type must be a number");
  // Interval(Like<Value> auto _minimum, Like<Value> auto _maximum): minimum(_minimum),
  // maximum(_maximum) {}
  Interval(Value _minimum, Value _maximum) : minimum(_minimum), maximum(_maximum) {}
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
  /// The type of the validator predicate, meaning the function that returns an error if the
  /// parameter update is rejected and an empty string otherwise. Why we don't return a
  /// std::optional<std::string> ?
  ///  Because this way we can ensure through the type system that if a parameter update is
  ///  rejected, it is always rejected for a reason.
  using Validate = std::function<std::string(const rclcpp::Parameter &)>;

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
      if (!set.set_of_values.count(new_value))
        return "The given value is not in the set of allowed values";
      return "";
    };
  }

  Validate get_default_validator() const {
    return [](const rclcpp::Parameter &) { return ""; };
  }

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  /// A predicate that indicates whether a given value is feasible.
  /// By default, a validator always returns true since the parameter is unconstrained
  Validate validate;
};

/// An stream representing parameters. It always registers the callback to obtain the parameter
/// updates.
template <class _Value>
struct ParameterStream : public Stream<_Value> {
  using Base = Stream<_Value>;
  using Value = _Value;
  static_assert(is_valid_ros_param_type<_Value>::value,
                "Type is not an allowed ROS parameter type");
  ParameterStream() = default;
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
  ParameterStream(Context &context, const std::string &parameter_name,
                  const Value &default_value, const Validator<Value> &validator = {},
                  std::string description = "", bool read_only = false,
                  bool ignore_override = false)
      : Base(context) {
    this->parameter_name = parameter_name;
    this->default_value = default_value;
    this->validator = validator;
    this->description = description;
    this->read_only = read_only;
    this->ignore_override = ignore_override;
    this->register_with_ros(context);
  }

  /// Register this paremeter with the ROS node, meaning it actually calls
  /// node->declare_parameter(). After calling this method, this ParameterStream will have a value.
  void register_with_ros(Context &context) {
    context.add_parameter<Value>(
        this->parameter_name, this->default_value,
        [impl = this->impl()](const rclcpp::Parameter &new_param) {
          impl->put_value(new_param.get_value<_Value>());
        },
        this->create_descriptor(), this->validator.validate, this->ignore_override);
    /// Set the default value
    this->impl()->put_value(context.node_base().get_parameter<Value>(this->parameter_name));
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

/// A class that abstracts a plain value and a ParameterStream so that both are supported in promise
/// mode. This is needed for example for timeouts and coordinate system names.
template <class Value>
struct ValueOrParameter {
  ValueOrParameter() = default;

  /// Construct a ValueOrParameter implicitly from a value
  ValueOrParameter(const Value &value)  // NOLINT
      : get([value]() { return value; }) {}

  /// Construct a ValueOrParameter implicitly from a something similar to a value, For example,
  /// `"hello"` is a const char[5] literal that is convertible to a std::string, the value.
  template <class T>
  requires std::convertible_to<T, Value> &&(!AnyStream<T>)ValueOrParameter(const T &v)  // NOLINT
      : get([value = Value(v)]() { return value; }) {}

  /// Construct a ValueOrParameter implicitly from parameter (i.e. ParameterStream)
  ValueOrParameter(const ParameterStream<Value> &param)  // NOLINT
      : get([param_impl = param.impl()]() { return param_impl.lock()->get_value(); }) {}

  /// We use a std::function for the type erasure: When called, it obtains the current value
  std::function<Value()> get;
};

template <class _Message>
struct SubscriptionStreamImpl {
  std::shared_ptr<rclcpp::Subscription<_Message>> subscriber;
};
/// A stream that represents a regular ROS subscriber. It stores as its value always a shared
/// pointer to the message.
template <class _Message>
struct SubscriptionStream
    : public Stream<typename _Message::SharedPtr, Nothing, SubscriptionStreamImpl<_Message>> {
  using Base = Stream<typename _Message::SharedPtr, Nothing, SubscriptionStreamImpl<_Message>>;
  using Value = typename _Message::SharedPtr;
  SubscriptionStream() = default;
  SubscriptionStream(Context &context, const std::string &topic_name, const rclcpp::QoS &qos,
                     const rclcpp::SubscriptionOptions &options)
      : Base(context) {
    this->impl()->subscriber = context.node_base().create_subscription<_Message>(
        topic_name,
        [impl = this->impl()](typename _Message::SharedPtr new_value) {
          impl->put_value(new_value);
        },
        qos, options);
  }
};

struct TransformSubscriptionStreamImpl {
  ValueOrParameter<std::string> source_frame;
  ValueOrParameter<std::string> target_frame;
  /// We do not own the listener, the Book owns it
  Weak<TFListener> tf2_listener;
};

/// A Stream that represents a subscription between two coordinate systems. (See TFListener)
/// It yields a value each time the transform between these two coordinate systems changes.
struct TransformSubscriptionStream : public Stream<geometry_msgs::msg::TransformStamped,
                                                   std::string, TransformSubscriptionStreamImpl> {
  using Base =
      Stream<geometry_msgs::msg::TransformStamped, std::string, TransformSubscriptionStreamImpl>;
  using Message = geometry_msgs::msg::TransformStamped;
  using Self = TransformSubscriptionStream;
  TransformSubscriptionStream() = default;
  TransformSubscriptionStream(Context &context, ValueOrParameter<std::string> target_frame,
                              ValueOrParameter<std::string> source_frame)
      : Base(context) {
    this->impl()->target_frame = target_frame;
    this->impl()->source_frame = source_frame;
    this->impl()->tf2_listener = context.add_tf_subscription(
        target_frame.get, source_frame.get,
        [impl = this->impl()](const geometry_msgs::msg::TransformStamped &new_value) {
          impl->put_value(new_value);
        },
        [impl = this->impl()](const tf2::TransformException &ex) { impl->put_error(ex.what()); });
  }
};

/// A TransformBuffer offers a modern, asynchronous interface for looking up transforms at a given
/// point in time. It is similar to the tf2_ros::AsyncBufferInterface, but returns awaitable
/// futures.
struct TransformBuffer {
  TransformBuffer() = default;
  TransformBuffer(Context &context) : tf_listener(context.add_tf_listener_if_needed()) {}

  /// @brief Does an asynchronous lookup for a single transform that can be awaited using `co_await`
  ///
  /// @param target_frame
  /// @param source_frame
  /// @param time At which time to get the transform
  /// @param timeout How long to wait for the transform
  /// @return A future that resolves with the transform or with an error if a timeout occurs
  Promise<geometry_msgs::msg::TransformStamped, std::string> lookup(const std::string &target_frame,
                                                                    const std::string &source_frame,
                                                                    const Time &time,
                                                                    const Duration &timeout) {
    using P = Promise<geometry_msgs::msg::TransformStamped, std::string>;
    return P([this, target_frame, source_frame, time, timeout](auto &promise) {
      auto request_handle = this->tf_listener->async_lookup(
          target_frame, source_frame, time, timeout,
          [&promise](const geometry_msgs::msg::TransformStamped &tf) { promise.put_value(tf); },
          [&promise](const tf2::TransformException &ex) { promise.put_error(ex.what()); });
      return typename P::Cancel{[this, request_handle](auto &promise) {
        if (promise.has_none()) {
          this->tf_listener->cancel_request(request_handle);
        }
      }};
    });
  }

  /// Overload for different time types.
  /*Promise<geometry_msgs::msg::TransformStamped, std::string>
  lookup(const std::string &target_frame, const std::string &source_frame, const auto &time,
    const Duration &timeout) { return lookup(target_frame, source_frame,
  icey::rclcpp_to_chrono(time), timeout); }
  */
  Weak<TFListener> tf_listener;
};

struct TimerImpl {
  size_t ticks_counter{0};
  rclcpp::TimerBase::SharedPtr timer;
};

/// A Stream representing a ROS-Timer. It saves the number of ticks as it's value.
struct TimerStream : public Stream<size_t, Nothing, TimerImpl> {
  using Base = Stream<size_t, Nothing, TimerImpl>;
  TimerStream() = default;
  TimerStream(Context &context, const Duration &interval, bool is_one_off_timer) : Base(context) {
    this->impl()->timer = context.node_base().create_wall_timer(interval, [impl = this->impl(), is_one_off_timer]() {
      /// Needed as separate state as it might be resetted in async/await mode
      auto cnt = impl->ticks_counter;
      impl->ticks_counter++;
      if (is_one_off_timer) impl->timer->cancel();
      impl->put_value(cnt);
    });
  }
  void reset() { this->impl()->timer->reset(); }
  void cancel() { this->impl()->timer->cancel(); }
};

template <class _Value>
struct PublisherImpl {
  std::shared_ptr<rclcpp::Publisher<remove_shared_ptr_t<_Value>>> publisher;
  void publish(const _Value &message) {
    /// TODO(Ivo) We always copy the message for publishing because we do not support so-called
    /// loaned messages: https://design.ros2.org/articles/zero_copy.html The following comment
    /// explains the reasoning with technical/API issues, but conceptually we simply do not support
    /// loaned messages.
    // Comment: We cannot pass over the pointer since publish expects a unique ptr and we got a
    // shared ptr. We cannot just promote the unique ptr to a shared ptr because we cannot ensure
    // the shared ptr is not referenced somewhere else.
    /// We could check whether use_count is one but this is not a reliable indicator whether the
    /// object not referenced anywhere else.
    // This is because the use_count can change in a multithreaded program immediately after it was
    // retrieved (i.e. a race occurs), see:
    /// https://en.cppreference.com/w/cpp/memory/shared_ptr/use_count (Same holds for
    /// shared_ptr::unique, which is defined simply as shared_ptr::unique -> bool: use_count() == 1)
    /// Therefore, we have to copy the message for publishing.
    if constexpr (is_shared_ptr<_Value>)
      publisher->publish(*message);
    else
      publisher->publish(message);
  }
};

/// A Stream representing a ROS-publisher.
/// \tparam _Value Can be either a `Message` or `std::shared_ptr<Message>` where `Message` must be
/// something publishable, i.e. either a valid ROS message or a masqueraded type.
template <class _Value>
struct PublisherStream : public Stream<_Value, Nothing, PublisherImpl<_Value>> {
  using Base = Stream<_Value, Nothing, PublisherImpl<_Value>>;
  using Message = remove_shared_ptr_t<_Value>;
  PublisherStream() = default;
  template <AnyStream Input = Stream<_Value>>
  PublisherStream(Context &context, const std::string &topic_name,
                  const rclcpp::QoS qos = rclcpp::SystemDefaultsQoS(),
                  const rclcpp::PublisherOptions publisher_options = {},
                  Input *maybe_input = nullptr)
      : Base(context) {
    this->impl()->publisher = context.node_base().create_publisher<Message>(topic_name, qos, publisher_options);
    this->impl()->register_handler(
        [impl = this->impl()](const auto &new_state) { impl->publish(new_state.value()); });
    if (maybe_input) {
      maybe_input->connect_values(*this);
    }
  }
  void publish(const _Value &message) const { this->impl()->publish(message); }
};

// A Stream representing a transform broadcaster that publishes transforms on TF.
struct TransformPublisherStreamImpl {
  Weak<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

struct TransformPublisherStream
    : public Stream<geometry_msgs::msg::TransformStamped, Nothing, TransformPublisherStreamImpl> {
  using Base = Stream<geometry_msgs::msg::TransformStamped, Nothing, TransformPublisherStreamImpl>;
  using Value = geometry_msgs::msg::TransformStamped;
  TransformPublisherStream() = default;
  template <AnyStream Input = Stream<geometry_msgs::msg::TransformStamped>>
  TransformPublisherStream(Context &context, Input *input = nullptr) : Base(context) {
    this->impl()->tf_broadcaster = context.add_tf_broadcaster_if_needed();
    this->impl()->register_handler([impl = this->impl()](const auto &new_state) {
      impl->tf_broadcaster->sendTransform(new_state.value());
    });
    if (input) input->connect_values(*this);
  }

  /// Publish a single message
  void publish(const geometry_msgs::msg::TransformStamped &message) {
    this->impl()->tf_broadcaster->sendTransform(message);
  }
};

template <class ServiceT>
struct ServiceStreamImpl {
  std::shared_ptr<rclcpp::Service<ServiceT>> service;
};

/// A Stream representing a ROS service (server). It stores as its value the RequestID and the the
/// Request itself. It supports synchronous as well as asynchronous responding.
///
/// See as a reference:
/// - https://github.com/ros2/rclcpp/pull/1709
/// - https://github.com/ros2/rclcpp/issues/1707
/// - https://github.com/tgroechel/lifecycle_prac/blob/main/src/async_srv.cpp#L10-L69C1
/// - https://github.com/ijnek/nested_services_rclcpp_demo
template <class _ServiceT>
struct ServiceStream : public Stream<std::tuple<std::shared_ptr<rmw_request_id_t>,
                                                std::shared_ptr<typename _ServiceT::Request>>,
                                     Nothing, ServiceStreamImpl<_ServiceT>> {
  using Base = Stream<
      std::tuple<std::shared_ptr<rmw_request_id_t>, std::shared_ptr<typename _ServiceT::Request>>,
      Nothing, ServiceStreamImpl<_ServiceT>>;
  using Request = std::shared_ptr<typename _ServiceT::Request>;
  using RequestID = std::shared_ptr<rmw_request_id_t>;
  using Response = std::shared_ptr<typename _ServiceT::Response>;

  /// The type of the user callback that can response synchronously (i.e. immediately): It receives
  /// the request and returns the response.
  using SyncCallback = std::function<Response(Request)>;

  /// The type of the user callback that responds asynchronously, i.e. is a coroutine.
  using AsyncCallback = std::function<Promise<Response>(Request)>;

  ServiceStream() = default;

  /*!
     \brief Construct the service server. With this constructor, no callback is provided. Instead,
     you must await requests with co_await `service_server` and then respond by calling
     the`respond`-method on this `service_server`.
  */
  ServiceStream(Context &context, const std::string &service_name,
                const rclcpp::QoS &qos = rclcpp::ServicesQoS())
      : Base(context) {
    this->impl()->service = context.node_base().create_service<_ServiceT>(
        service_name,
        [impl = this->impl()](RequestID request_id, Request request) {
          impl->put_value(std::make_tuple(request_id, request));
        },
        qos);
  }

  /*!
     \brief Construct the service server. A callback may be provided that will be called every time
     this service is called, it returns the response immediately. This callback can be either
     synchronous or asynchronous, i.e. a coroutine, it can contain calls to `co_await`.
  */
  ServiceStream(Context &context, const std::string &service_name, SyncCallback sync_callback,
                const rclcpp::QoS &qos = rclcpp::ServicesQoS())
      : ServiceStream(context, service_name, qos) {
    this->impl()->register_handler([impl = this->impl(), sync_callback](auto new_state) {
      const auto [request_id, request] = new_state.value();
      auto response = sync_callback(request);
      impl->service->send_response(*request_id, *response);
    });
  }

  /*!
     \brief Construct the service server. A callback may be provided that will be called every time
     this service is called, it returns the response immediately. This callback can be either
     synchronous or asynchronous, i.e. a coroutine, it can contain calls to `co_await`.
  */
  ServiceStream(Context &context, const std::string &service_name,
                AsyncCallback async_callback, const rclcpp::QoS &qos = rclcpp::ServicesQoS())
      : ServiceStream(context, service_name, qos) {
    this->impl()->register_handler([impl = this->impl(), async_callback](auto new_state) {
      const auto continuation = [](auto impl, const auto &async_callback, RequestID request_id,
                                   Request request) -> Promise<void> {
        auto response = co_await async_callback(request);
        if (response)  /// If we got nullptr, this means we do not respond.
          impl->service->send_response(*request_id, *response);
        co_return;
      };
      const auto [request_id, request] = new_state.value();
      continuation(impl, async_callback, request_id, request);
    });
  }

  /// Send the service response to the request identified by the request_id.
  ///  It is RMW implementation defined whether this happens synchronously or asynchronously.
  /// \throws an exception from `rclcpp::exceptions::throw_from_rcl_error()`
  /// See for a detailed documentation:
  /// - The C-API that rclcpp uses more or less directly:
  /// [rcl_send_response](http://docs.ros.org/en/jazzy/p/rcl/generated/function_service_8h_1a8631f47c48757228b813d0849d399d81.html#_CPPv417rcl_send_responsePK13rcl_service_tP16rmw_request_id_tPv)
  /// - One layer below:
  /// [rmw_send_response](https://docs.ros.org/en/humble/p/rmw/generated/function_rmw_8h_1abb55ba2b2a957cefb0a77b77ddc5afda.html)
  /// \note ROS does not seem to support awaiting this operation (at the RMW layer), this means you
  /// cannot await until the service client received your response.
  void respond(RequestID request_id, Response response) {
    this->impl()->service->send_response(*request_id, *response);
  }
};

/// A service client. It can we called asynchronously and then it returns a Promise that can be
/// awaited. It is not tracked by the context but instead the service client is unregistered from
/// ROS class goes out of scope.
template <class ServiceT>
struct ServiceClient : public ServiceClientImpl<ServiceT> {
  using Base = ServiceClientImpl<ServiceT>;
  using Self = ServiceClient<ServiceT>;
  using Request = typename ServiceT::Request::SharedPtr;
  using Response = typename ServiceT::Response::SharedPtr;
  
  ServiceClient() = default;
  /// Create a new service client
  /// \param node
  /// \param service_name
  /// \param timeout the timeout that will be used for each request (i.e. `call`) to this service,
  /// as well as for the service discovery, i.e. wait_for_service. \param qos
  ServiceClient(Context &context, const std::string &service_name,
                const rclcpp::QoS &qos = rclcpp::ServicesQoS())
      : Base(context.node_base(), service_name, qos) {}

  // clang-format off
  /*! Make an asynchronous call to the service. Returns a Promise that can be awaited using `co_await`.
  Requests can never hang forever but will eventually time out. Also you don't need to clean up pending requests -- they will be cleaned up automatically. So this function will never cause any memory leaks.
  \param request the request
  \param timeout The timeout for the service call, both for service discovery and the actual call.
  \returns A future that can be awaited to obtain the response or an error. Possible errors are "TIMEOUT", "SERVICE_UNAVAILABLE" or "INTERRUPTED".
  \note The 
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
    return Promise<Response, std::string>{[this, request, timeout](auto &future) {
      auto request_id= Base::call(request, timeout, 
          [&](const auto &x) { future.put_value(x); },
          [&](const auto &x) { future.put_error(x); }
        );
      return Cancel{[this, request_id](auto &) { Base::cancel_request(request_id); }};
    }};
  }
};

/// A filter that detects timeouts, i.e. whether a value was received in a given time window.
/// It simply passes over the value if no timeout occurred, and errors otherwise.
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
  TimeoutFilter(Context &context, Input input, const Duration &max_age,
                bool create_extra_timer = true)
      : Base(context) {
    auto node_clock = context.node_base().get_node_clock_interface();
    rclcpp::Duration max_age_ros(max_age);
    auto check_state = [impl = this->impl(), node_clock, max_age_ros](const auto &new_state) {
      if (!new_state.has_value()) return true;
      const auto &message = new_state.value();
      rclcpp::Time time_now{node_clock->get_clock()->now()};
      rclcpp::Time time_message{message->header.stamp};
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
      timer.then([timer_impl = timer.impl(), input_impl = input.impl(), check_state](size_t) {
        bool timeout_occurred = check_state(input_impl->get_state());
        if (!timeout_occurred) timer_impl->timer->reset();
      });
    } else {
      input.impl()->register_handler(check_state);
    }
  }
};

/// This impl is needed because it makes the signalMessage method public, it is otherwise protected.
template <class Message>
struct SimpleFilterAdapterImpl : public message_filters::SimpleFilter<Message> {
  void signalMessage(auto event) { message_filters::SimpleFilter<Message>::signalMessage(event); }
};

/// Adapts the `message_filters::SimpleFilter` to our
/// `Stream` (which is a similar concept).
/// \note This is essentially the same as what
/// `message_filters::Subscriber` does:
/// https://github.com/ros2/message_filters/blob/humble/include/message_filters/subscriber.h#L349
template <class Message>
struct SimpleFilterAdapter
    : public Stream<typename Message::SharedPtr, Nothing, SimpleFilterAdapterImpl<Message>> {
  using Base = Stream<typename Message::SharedPtr, Nothing, SimpleFilterAdapterImpl<Message>>;
  /// Constructs a new instance and connects this Stream to the `message_filters::SimpleFilter` so
  /// that `signalMessage` is called once a value is received.
  SimpleFilterAdapter(Context &context) : Base(context) {
    this->impl()->register_handler([impl = this->impl()](const auto &new_state) {
      using Event = message_filters::MessageEvent<const Message>;
      impl->signalMessage(Event(new_state.value()));
    });
  }
};

/// Holds a message_filters::Synchronizer and operates on it.
template <class... Messages>
struct ApproxTimeSynchronizerImpl {
  using Self = ApproxTimeSynchronizerImpl<Messages...>;
  using Policy = message_filters::sync_policies::ApproximateTime<Messages...>;
  using Synchronizer = message_filters::Synchronizer<Policy>;
  using Inputs = std::tuple<SimpleFilterAdapter<Messages>...>;
  const Inputs &inputs() const { return *inputs_; }

  /// TODO Hack we know that his class is derived from Derived, we will need to downcast
  /// to put the value in the Stream impl.
  /// A non-hack would be to change every impl to derive from impl::Stream
  /// We have to do this because we cannot capture this in the actual Stream but due to the
  /// very old (pre C++11) callback registering code in the message_filters package
  /// we cannot register a lambda where we could capture the impl. And at Humble, the variadic fix
  /// was not backported yet, as the variadic fix came only 2024.
  ///
  /// Note that this is like CRTP but but not done explicitly.
  /// So this is equally UB as CRTP is UB, not more, not less.
  /// See Stream::Impl
  using Derived = impl::Stream<std::tuple<typename Messages::SharedPtr...>, std::string,
                               WithDefaults<Self>, WithDefaults<Nothing>>;

  void create_synchronizer(Context &context, uint32_t queue_size) {
    queue_size_ = queue_size;
    inputs_ = std::make_shared<Inputs>(std::make_tuple(SimpleFilterAdapter<Messages>(context)...));
    auto synchronizer = std::make_shared<Synchronizer>(Policy(queue_size_));
    synchronizer_ = synchronizer;
    /// Connect with the input streams
    std::apply(
        [synchronizer](auto &...input_filters) {
          synchronizer->connectInput(*input_filters.impl()...);
        },
        *inputs_);
    /// This parameter setting is from the examples
    synchronizer_->setAgePenalty(0.50);
    synchronizer_->registerCallback(&Self::on_messages, this);
  }

  void on_messages(typename Messages::SharedPtr... msgs) {
    static_cast<Derived *>(this)->put_value(std::forward_as_tuple(msgs...));
  }

  uint32_t queue_size_{100};
  std::shared_ptr<Inputs> inputs_;
  std::shared_ptr<Synchronizer> synchronizer_;
};

/// A Stream representing an approximate time synchronizer.
/// \warning All inputs must have the same QoS according to the documentation of message_filters
template <class... Messages>
class ApproxTimeSynchronizer : public Stream<std::tuple<typename Messages::SharedPtr...>,
                                             std::string, ApproxTimeSynchronizerImpl<Messages...>> {
public:
  using Base = Stream<std::tuple<typename Messages::SharedPtr...>, std::string,
                      ApproxTimeSynchronizerImpl<Messages...>>;
  using Self = ApproxTimeSynchronizer<Messages...>;

  /// Constructs the synchronizer and connects it to the input streams so that it is ready to
  /// receive values.
  template <ErrorFreeStream... Inputs>
  ApproxTimeSynchronizer(Context &context, uint32_t queue_size, Inputs... inputs)
      : Base(context) {
    static_assert(sizeof...(Inputs) >= 2, "You need to synchronize at least two inputs.");
    this->impl()->create_synchronizer(context, queue_size);
    this->connect_inputs(inputs...);
  }

protected:
  /// Connects the given streams with the synchronizer inputs.
  template <ErrorFreeStream... Inputs>
  void connect_inputs(Inputs... inputs) {
    using namespace hana::literals;
    auto zipped = hana::zip(std::forward_as_tuple(inputs...), this->impl()->inputs());
    hana::for_each(zipped, [](auto &input_output_tuple) {
      auto &input_stream = input_output_tuple[0_c];
      auto &synchronizer_input = input_output_tuple[1_c];
      input_stream.connect_values(synchronizer_input);
    });
  }
};

/// Synchronizes a topic with a transform using tf2_ros::MessageFilter.
/// TODO we could implement this much simpler in like 50 SLOC once we support coroutines as
/// subscriber callbacks and get rid of the messy tf2_ros::MessageFilter code.
template <class Message>
struct TransformSynchronizerImpl {
  using Self = TransformSynchronizerImpl<Message>;
  /// The book owns it.
  Weak<TFListener> tf_listener;
  std::string target_frame;
  std::shared_ptr<SimpleFilterAdapter<Message>> input_filter;
  std::shared_ptr<tf2_ros::MessageFilter<Message>> synchronizer;

  using Derived =
      impl::Stream<std::tuple<typename Message::SharedPtr, geometry_msgs::msg::TransformStamped>,
                   std::string, WithDefaults<Self>, WithDefaults<Nothing>>;

  void create_synchronizer(Context &context, const std::string &_target_frame,
                           const Duration &lookup_timeout) {
    tf_listener = context.add_tf_listener_if_needed();
    target_frame = _target_frame;
    input_filter = std::make_shared<SimpleFilterAdapter<Message>>(context);
    /// The argument "0" means here infinite message queue size. We set it so
    /// because we must buffer every message for as long as we are waiting for a transform.
    synchronizer = std::make_shared<tf2_ros::MessageFilter<Message>>(
        *input_filter->impl(), *tf_listener->buffer_, target_frame, 0,
        context.node_base().get_node_logging_interface(), context.node_base().get_node_clock_interface(), lookup_timeout);

    synchronizer->registerCallback(&Self::on_message, this);
  }

  void on_message(typename Message::SharedPtr message) {
    const auto timestamp = rclcpp_to_chrono(rclcpp::Time(message->header.stamp));
    auto maybe_transform =
        this->tf_listener->get_from_buffer(target_frame, message->header.frame_id, timestamp);
    if (maybe_transform.has_value())
      static_cast<Derived *>(this)->put_value(std::make_tuple(message, maybe_transform.value()));
    else
      throw std::logic_error(
          "tf2_ros::MessageFilter broke the promise that the transform is available");
  }
};

/// Synchronizes a topic with a transform using tf2_ros::MessageFilter.
template <class Value>
struct TransformSynchronizer
    : public Stream<std::tuple<Value, geometry_msgs::msg::TransformStamped>, std::string,
                    TransformSynchronizerImpl<remove_shared_ptr_t<Value>>> {
  using Base = Stream<std::tuple<Value, geometry_msgs::msg::TransformStamped>, std::string,
                      TransformSynchronizerImpl<remove_shared_ptr_t<Value>>>;
  /*!
    \brief Construct the TransformSynchronizer and connect it to the input.
    \param target_frame the transform on which we wait is specified by source_frame and
     target_frame, where source_frame is the frame in the header of the message
     \param lookup_timeout The maximum time to wait until the transform gets available for a message
  */
  template <ErrorFreeStream Input = Stream<int>>
  TransformSynchronizer(Context &context, const std::string &target_frame,
                        const Duration &lookup_timeout, Input *input = nullptr)
      : Base(context) {
    this->impl()->create_synchronizer(context, target_frame, lookup_timeout);
    if (input) {
      input->connect_values(*this->impl()->input_filter);
    }
  }
};

/*!
  Outputs the value or error of any of the inputs. All the inputs must have the same Value and
  Error type.
  \tparam Inputs A list of Stream<Value, Error> types, i.e. all the input Streams must have the same
  Value and Error type. \returns Stream<Value, Error>
*/
template <AnyStream... Inputs>
static auto any(Inputs... inputs) {
  // assert_all_stream_values_are_same<Inputs...>();
  auto first_input = std::get<0>(std::forward_as_tuple(inputs...));
  using Input = decltype(first_input);
  using InputValue = typename std::remove_reference_t<Input>::Value;
  using InputError = typename std::remove_reference_t<Input>::Error;
  /// First, create a new stream
  auto output = first_input.template create_stream<Stream<InputValue, InputError>>();
  /// Now connect each input with the output
  hana::for_each(std::forward_as_tuple(inputs...),
                 [output](auto &input) { input.connect_values(output); });
  return output;
}

/// Synchronize at least two streams by approximately matching the header time-stamps (using
/// the `message_filters::Synchronizer`). It accepts only an ErrorFreeStream since the synchronizer
/// may emit new errors. To obtain an ErrorFreeStream, use Stream::unwrap_or.
///
/// \tparam Inputs the input stream types, not necessarily all the same
/// \param queue_size the queue size to use, 100 is a good value.
/// \param inputs the input streams, not necessarily all of the same type
template <ErrorFreeStream... Inputs>
static ApproxTimeSynchronizer<MessageOf<Inputs>...> synchronize_approx_time(uint32_t queue_size,
                                                                            Inputs... inputs) {
  auto first_input = std::get<0>(std::forward_as_tuple(inputs...));  /// Use the first Stream
  return first_input.template create_stream<ApproxTimeSynchronizer<MessageOf<Inputs>...>>(
      queue_size, inputs...);
}

template <class ParameterT>
ParameterStream<ParameterT> Context::declare_parameter(const std::string &parameter_name,
                                              const ParameterT &default_value,
                                              const Validator<ParameterT> &validator,
                                              std::string description,
                                              bool read_only,
                                              bool ignore_override) {
  return create_stream<ParameterStream<ParameterT>>(parameter_name, default_value, validator,
                                                    description, read_only, ignore_override);
}

template <class ParameterStruct>
void Context::declare_parameter_struct(
    ParameterStruct &params, const std::function<void(const std::string &)> &notify_callback,
    std::string name_prefix) {
  field_reflection::for_each_field(params, [this, notify_callback, name_prefix](
                                                std::string_view field_name, auto &field_value) {
    using Field = std::remove_reference_t<decltype(field_value)>;
    std::string field_name_r = name_prefix + std::string(field_name);
    if constexpr (is_stream<Field>) {
      field_value.impl_ = this->create_stream_impl<typename Field::Impl>();
      field_value.impl()->context =
          this->shared_from_this();  /// First, give it the missing context
      field_value.parameter_name = field_name_r;
      if (notify_callback) {
        field_value.impl()->register_handler(
            [field_name_r, notify_callback](const auto &) { notify_callback(field_name_r); });
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
          "Every field of the parameters struct must be of type T or icey::Parameter<T> or "
          "a "
          "struct of such, where T is a valid ROS param type (see rcl_interfaces/ParameterType)");
    }
  });
}

template <class MessageT>
SubscriptionStream<MessageT> Context::create_subscription(
    const std::string &name, const rclcpp::QoS &qos,
    const rclcpp::SubscriptionOptions &options) {
  return create_stream<SubscriptionStream<MessageT>>(name, qos, options);
}

inline TransformSubscriptionStream Context::create_transform_subscription(
    ValueOrParameter<std::string> target_frame, ValueOrParameter<std::string> source_frame) {
  return create_stream<TransformSubscriptionStream>(target_frame, source_frame);
}

inline TransformBuffer Context::create_transform_buffer() { return TransformBuffer{*this}; }

template <class Message>
PublisherStream<Message> Context::create_publisher(const std::string &topic_name,
                                          const rclcpp::QoS &qos,
                                          const rclcpp::PublisherOptions publisher_options) {
  return create_stream<PublisherStream<Message>>(topic_name, qos, publisher_options);
}

/// Create a publisher to publish transforms on the `/tf` topic. Works otherwise the same as
/// [tf2_ros::TransformBroadcaster].
inline TransformPublisherStream Context::create_transform_publisher() {
  return create_stream<TransformPublisherStream>();
}

inline TimerStream Context::create_timer(const Duration &period, bool is_one_off_timer) {
  return create_stream<TimerStream>(period, is_one_off_timer);
}

template <class ServiceT, class Callback>
ServiceStream<ServiceT> Context::create_service(const std::string &service_name, Callback &&callback,
                                        const rclcpp::QoS &qos) {
  return create_stream<ServiceStream<ServiceT>>(service_name, callback, qos);
}

template <class ServiceT>
ServiceStream<ServiceT> Context::create_service(const std::string &service_name,
                                        const rclcpp::QoS &qos) {
  return create_stream<ServiceStream<ServiceT>>(service_name, qos);
}


template <class ServiceT>
ServiceClient<ServiceT> Context::create_client(const std::string &service_name,
                                      const rclcpp::QoS &qos) {
  return ServiceClient<ServiceT>{*this, service_name, qos};
}

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
    this->icey_context_ = std::make_shared<Context>(NodeBase(this));
  }

  /// Returns the ICEY context
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

/// Parameter is a type to use only in parameter structs. Currently it is a Stream but this is
/// likely to change in the future
template <class Value>
using Parameter = ParameterStream<Value>;

/// Start spinning either a Node or a LifeCycleNode. Calls `rclcpp::shutdown()` at the end so you do
/// not have to do it.
template <class NodeType>
static void spin(NodeType node) {
  /// We use single-threaded executor because the MT one can starve due to a bug
  rclcpp::executors::SingleThreadedExecutor executor;
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
