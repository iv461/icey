#include <icey/icey_async_await.hpp>

namespace icey {

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
#if ICEY_ASYNC_AWAIT_THREAD_SAFE
    std::atomic_bool finished{false};
    mutable std::mutex state_mutex;
#endif
  };

  /// A request for a transform lookup: It represents effectively a single call to the async_lookup
  /// function. Even if you call async_lookup with the exact same arguments (same frames and time),
  /// this will count as two separate requests.
  using RequestHandle = std::shared_ptr<TransformRequest>;

  explicit TransformBufferImpl(std::weak_ptr<NodeBase> node) : node_(node) { init(); }

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
#if ICEY_ASYNC_AWAIT_THREAD_SAFE
    std::lock_guard<std::recursive_mutex> lock{mutex_};
#endif
    requests_.emplace(request);
    node_.lock()->add_task_for(
        uint64_t(request.get()), timeout, [this, on_error, request = weak_request]() {
          if (auto req = request.lock()) {
#if ICEY_ASYNC_AWAIT_THREAD_SAFE
            bool expected = false;
            if (!req->finished.compare_exchange_strong(expected, true)) {
              return;
            }
            std::lock_guard<std::recursive_mutex> lock{mutex_};
#endif
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
  bool cancel_request(uint64_t request_id) {
    std::lock_guard<std::recursive_mutex> lock{mutex_};
    requests_.erase(request);
    return node_.lock()->cancel_task_for(request_id);
  }

  /// We take a tf2_ros::Buffer instead of a tf2::BufferImpl only to be able to use ROS-time API
  /// (internally TF2 has it's own timestamps...), not because we need to wait on anything (that's
  /// what tf2_ros::Buffer does in addition to tf2::BufferImpl).
  std::shared_ptr<tf2_ros::Buffer> buffer_;

protected:
  void init() {
    init_tf_buffer();
    message_subscription_tf_ = node_.lock()->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", [this](TransformsMsg msg) { on_tf_message(msg, false); },
        tf2_ros::DynamicListenerQoS());
    message_subscription_tf_static_ = node_.lock()->create_subscription<tf2_msgs::msg::TFMessage>(
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
    buffer_ =
        std::make_shared<tf2_ros::Buffer>(node_.lock()->get_node_clock_interface()->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        node_.lock()->get_node_base_interface(), node_.lock()->get_node_timers_interface());
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
        RCLCPP_ERROR(node_.lock()->get_node_logging_interface()->get_logger(),
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
      bool changed;
#if ICEY_ASYNC_AWAIT_THREAD_SAFE
      {
        std::lock_guard<std::mutex> lock{info.state_mutex};
        changed = !info.last_received_transform || tf_msg != *info.last_received_transform;
        if (changed) info.last_received_transform = tf_msg;
      }
#else
      changed = !info.last_received_transform || tf_msg != *info.last_received_transform;
      if (changed) info.last_received_transform = tf_msg;
#endif
      if (changed) {
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
#if ICEY_ASYNC_AWAIT_THREAD_SAFE
      bool expected = false;
      if (!request->finished.compare_exchange_strong(expected, true, std::memory_order_acq_rel,
                                                     std::memory_order_acquire)) {
        return false;
      }
#endif
      request->on_transform(tf_msg);
      /// If the request is destroyed gracefully (lookup succeeded), cancel the associated task
      /// timer
      node_.lock()->cancel_task_for(uint64_t(request.get()));
      return true;
    } catch (
        const tf2::TransformException &e) {  /// TODO we could catch for extrapolation here as well
      /// For any other error, we continue waiting
    }
    return false;
  }

  void notify_if_any_relevant_transform_was_received() {
/// Iterate all requests, notify and maybe erase them if they are requests for a specific time
#if ICEY_ASYNC_AWAIT_THREAD_SAFE
    std::vector<RequestHandle> requests_to_delete;
    std::vector<RequestHandle> requests;
    {
      std::lock_guard<std::recursive_mutex> lock{mutex_};
      requests.reserve(requests_.size());
      for (const auto &req : requests_) requests.push_back(req);
    }
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
    std::lock_guard<std::recursive_mutex> lock{mutex_};
    for (const auto &k : requests_to_delete) requests_.erase(k);
#else
    std::erase_if(requests_, [this](auto req) {
      if (req->maybe_time) {
        return maybe_notify_specific_time(req);
      } else {
        // If it is a regular subscription, is is persistend and never erased
        maybe_notify(*req);
        return false;
      }
    });
#endif
  }

  void on_tf_message(const TransformsMsg &msg, bool is_static) {
    store_in_buffer(*msg, is_static);
    notify_if_any_relevant_transform_was_received();
  }

  std::weak_ptr<NodeBase> node_;  /// Hold weak reference to the NodeBase because the Context owns
                                  /// the NodeBase as well, so we avoid circular reference

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_;

  std::recursive_mutex mutex_;
  std::unordered_set<RequestHandle> requests_;
};

TransformBuffer &TransformBuffer::init(std::weak_ptr<NodeBase> node_base) {
  if (!impl_) {
    impl_ = std::make_shared<TransformBufferImpl>(node_base);
  }
  return *this;
}

}  // namespace icey