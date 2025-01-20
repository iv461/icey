#pragma once

#include <any>
#include <functional>
#include <map>
#include <optional>
#include <tuple>
#include <unordered_map>

/// TF2 support
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

// .h so that this works with humble as well
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace icey {

/// A ROS adapter, abstracting ROS 1 and ROS 2, so that everything works with both
class ROS2Adapter {
public:
  template <class Msg>
  using MsgPtr = std::shared_ptr<Msg>;

  /// TODO should be compatible with the templates, but check
  using Clock = std::chrono::system_clock;
  using Time = std::chrono::time_point<Clock>;
  using Duration = Clock::duration;

  using Timer = rclcpp::TimerBase::SharedPtr;

  template <class Msg>
  using Subscriber = class rclcpp::Subscription<Msg>::SharedPtr;
  template <class Msg>
  using Publisher = class rclcpp::Publisher<Msg>::SharedPtr;

  using NodePtr = std::shared_ptr<rclcpp::Node>;
  using _Node = rclcpp::Node;  /// Underlying Node type

  using QoS = rclcpp::QoS;
  using DefaultQoS = rclcpp::SystemDefaultsQoS;
  using ServiceQoS = rclcpp::ServicesQoS;

  /// A modified listener that can notify when a single transform changes.
  /// It is implemented similarly to the tf2_ros::TransformListener, but without a separate node.
  /// TODO this simple implementation currently checks every time a new message is receved on /tf
  /// for every trnasform we are looking for. If /tf is high-frequency topic, this can become a
  /// performance problem. i.e., the more transforms, the more this becomes a search for the needle
  /// in the heapstack. But without knowing the path in the TF-tree that connects our transforms
  /// that we are looking for, we cannot do bettter. Update: We could obtain the path with
  /// tf2::BufferCore::_chainAsVector however, I need to look into it
  struct TFListener {
    using TransformMsg = geometry_msgs::msg::TransformStamped;
    using OnTransform = std::function<void(const TransformMsg &)>;
    using OnError = std::function<void(const tf2::TransformException &)>;

    TFListener(NodePtr node, tf2_ros::Buffer &buffer) : node_(node), buffer_(buffer) { init(node); }

    /// Add notification for a single transform.
    
    void add_subscription(std::string target_frame, std::string source_frame,
                          const OnTransform &on_transform, 
                          const OnError &on_error) {
      subscribed_transforms_.emplace_back(std::make_pair(target_frame, source_frame), std::nullopt,
                                          on_transform, 
                                          on_error);
    }
  
    tf2_ros::Buffer  &buffer_;

  private:
    using TransformsMsg = tf2_msgs::msg::TFMessage::ConstSharedPtr;
    /// A tf subctiption, the frames, last received transform, and the notify CB
    using FrameNames = std::pair<std::string, std::string>;
    using TFSubscriptionInfo = std::tuple<FrameNames, std::optional<TransformMsg>, OnTransform, OnError>;

    void on_tf_message(TransformsMsg msg, bool is_static) {
      std::cout << "on_tf_message(TransformsMsg " << std::endl;
      store_in_buffer(*msg, is_static);
      notify_if_any_relevant_transform_was_received();
    }

    void notify_if_any_relevant_transform_was_received() {
      for (auto &tf_info : subscribed_transforms_) {
        maybe_notify(tf_info);
      }
    }

    /// This simply looks up the transform in the buffer at the latest stamp and checks if it
    /// changed with respect to the previously received one. If the transform has changed, we know
    /// we have to notify
    void maybe_notify(TFSubscriptionInfo &sub) {
      auto &[frame_names, last_received_transform, on_transform, on_error] = sub;
      const auto &[target_frame, source_frame] = frame_names;
          std::cout << "maybe notify" << std::endl;
      try {
        /// Lookup the latest transform in the buffer to see if we got something new in the buffer 
        geometry_msgs::msg::TransformStamped tf_msg = buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        /// Now check if it is the same as the last one, in this case we return nothing since the
        /// transform did not change. (Instead, we received on /tf some other, unrelated transforms.)
        if (!last_received_transform || tf_msg != *last_received_transform)  {
          last_received_transform = tf_msg;
          on_transform(tf_msg);
        } else {
          std::cout << "Did not change" << std::endl;
        }
      } catch (tf2::TransformException &e) {
        /// Simply ignore. Because we are requesting the latest transform in the buffer, the only
        /// exception we can get is that there is no transform available yet.
        /// TODO duble-check if there is reallly nothing to do here.
        std::cout << "LOOKUP EX" << std::endl;
        on_error(e);
      }
    }

    /// Store the received transforms in the buffer.
    void store_in_buffer(const tf2_msgs::msg::TFMessage &msg_in, bool is_static) {
      std::string authority = "Authority undetectable";
      for (size_t i = 0u; i < msg_in.transforms.size(); i++) {
        try {
          buffer_.setTransform(msg_in.transforms[i], authority, is_static);
        } catch (const tf2::TransformException &ex) {
          // /\todo Use error reporting
          std::string temp = ex.what();
          RCLCPP_ERROR(node_->get_logger(),
                       "Failure to set received transform from %s to %s with error: %s\n",
                       msg_in.transforms[i].child_frame_id.c_str(),
                       msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
        }
      }
    }

    void init(NodePtr node) {
      const rclcpp::QoS qos = tf2_ros::DynamicListenerQoS();
      const rclcpp::QoS &static_qos = tf2_ros::StaticListenerQoS();
      message_subscription_tf_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
          "/tf", qos, [this](TransformsMsg msg) { on_tf_message(msg, false); });
      message_subscription_tf_static_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
          "/tf_static", static_qos, [this](TransformsMsg msg) { on_tf_message(msg, true); });
    }
    NodePtr node_;  /// Stored for logging

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_;

    std::vector<TFSubscriptionInfo> subscribed_transforms_;
  };

  /// A node interface, wrapping to some common functions
  class Node : public rclcpp::Node {
  public:
    using Base = rclcpp::Node;
    using ParameterUpdateCB = std::function<void(const rclcpp::Parameter &)>;

    Node(const std::string &name) : Base(name) {}

    template <class ParameterT, class CallbackT>
    auto declare_parameter(const std::string &name, const std::optional<ParameterT> &default_value,
                           CallbackT &&update_callback,
                           const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
                               rcl_interfaces::msg::ParameterDescriptor(),
                           bool ignore_override = false) {
      rclcpp::ParameterValue v =
          default_value ? rclcpp::ParameterValue(*default_value) : rclcpp::ParameterValue();
      auto param = static_cast<Base &>(*this).declare_parameter(name, v, parameter_descriptor,
                                                                ignore_override);
      auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
      auto cb_handle = param_subscriber->add_parameter_callback(name, std::move(update_callback));
      my_parameters_stuff_.emplace(name, std::make_pair(param_subscriber, cb_handle));
      return param;
    }

    template <class Msg, class F>
    void add_subscription(const std::string &topic, F &&cb, const rclcpp::QoS &qos,
                          const rclcpp::SubscriptionOptions &options) {
      my_subscribers_[topic] = create_subscription<Msg>(topic, qos, cb, options);
    }

    template <class Msg>
    auto add_publication(const std::string &topic, const QoS &qos = DefaultQoS(),
                         std::optional<double> max_frequency = std::nullopt) {
      auto publisher = create_publisher<Msg>(topic, qos);
      my_publishers_.emplace(topic, publisher);

      auto const publish = [this, publisher, topic, max_frequency](const Msg &msg) {
        publisher->publish(msg);
      };
      return publish;
    }

    template <class CallbackT>
    auto add_timer(const Duration &time_interval, bool use_wall_time, CallbackT &&cb) {
      rclcpp::TimerBase::SharedPtr timer =
          use_wall_time
              ? create_wall_timer(time_interval, std::move(cb))
              : create_wall_timer(time_interval, std::move(cb));  /// TODO no normal timer in humble

      my_timers_.push_back(timer);
      return timer;
    }

    template <class ServiceT, class CallbackT>
    void add_service(const std::string &service_name, CallbackT &&callback,
                     const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
      auto service = this->create_service<ServiceT>(
          service_name, std::move(callback));  /// TODO In humble, we cannot pass QoS, only in Jazzy
                                               /// (the API wasn't ready yet, it needs rmw_*-stuff)
      my_services_.emplace(service_name, service);
    }

    /// TODO cb groups !!
    template <class Service>
    auto add_client(const std::string &service_name) {
      auto client = this->create_client<Service>(service_name);
      my_services_clients_.emplace(service_name, client);
      return client;
    }

    /// Subscribe to a transform on tf between two frames
    template <class OnTransform, class OnError>
    auto add_tf_subscription(std::string target_frame, std::string source_frame,
                             OnTransform &&on_transform,
                             OnError &&on_error) {
      add_tf_listener_if_needed();
      tf2_listener_->add_subscription(target_frame, source_frame, on_transform, on_error);
      return tf2_listener_;
    }

    auto add_tf_broadcaster_if_needed() {
      if (!tf_broadcaster_)
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
      const auto publish = [this](const geometry_msgs::msg::TransformStamped &msg) {
        tf_broadcaster_->sendTransform(msg);
      };
      return publish;
    }

    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;  /// Public for using other functions

  private:
    void add_tf_listener_if_needed() {
      if (tf2_listener_)  /// We need only one subscription on /tf, but can have multiple transforms
                          /// on which we listen
        return;
      init_tf_buffer();
      tf2_listener_ = std::make_shared<TFListener>(shared_from_this(), *tf2_buffer_);
    }

    void init_tf_buffer() {
      /// This code is a bit tricky. It's about asynchronous programming essentially. The official
      /// example is rather incomplete
      /// ([official](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html))
      /// . I'm following instead this example
      /// https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/src/imu_transformer.cpp#L16
      /// See also the follwing discussions:
      /// https://answers.ros.org/question/372608/?sort=votes
      /// https://github.com/ros-navigation/navigation2/issues/1182
      /// https://github.com/ros2/geometry2/issues/446
      tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
          this->get_node_base_interface(), this->get_node_timers_interface());
      tf2_buffer_->setCreateTimerInterface(timer_interface);
    }

    /// Do not force the user to do the bookkeeping itself: Do it instead automatically
    std::map<std::string, std::pair<std::shared_ptr<rclcpp::ParameterEventHandler>,
                                    std::shared_ptr<rclcpp::ParameterCallbackHandle>>>
        my_parameters_stuff_;

    std::vector<rclcpp::TimerBase::SharedPtr> my_timers_;
    std::map<std::string, std::any> my_subscribers_;
    std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>> my_publishers_;
    std::map<std::string, std::any> my_services_;
    std::map<std::string, std::any> my_services_clients_;

    std::vector<rclcpp::CallbackGroup::SharedPtr> my_callback_groups_;

    /// TF stuff
    std::shared_ptr<TFListener> tf2_listener_;
    /// This is a simple wrapper around a publisher, there is really nothing intereseting under the
    /// hood of tf2_ros::TransformBroadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  };

  using NodeHandle = Node;
};

using ROSAdapter = ROS2Adapter;

}  // namespace icey

#include <icey/icey.hpp>
