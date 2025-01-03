#pragma once

#include <iostream> 
#include <fmt/core.h>
#include <fmt/ostream.h>

#include <functional>
#include <tuple>
#include <map>
#include <optional>
#include <unordered_map>
#include <any> 

#include <ros/ros.h>

namespace icey {

class ROS1Adapter {
public:

  template <typename MsgT>
  using MsgPtr = boost::shared_ptr<const MsgT>;
  using NodeHandle = ros::NodeHandle;

  /* @brief Typed subscriber, to know how to subscribe without having to specify the type of the message to the subscribe function.
  * Needed only for ROS 1 since ROS 2 does not use the type-erasing subscribers anymore.
  */
  template <typename MessageT>
  struct Subscriber {
    void subscribe(ros::NodeHandle &node_handle, const std::string &topic_name, uint32_t queue_length,
                  std::function<void(const MsgPtr<MessageT> &)> callback) {
      sub_ = node_handle.subscribe<MessageT>(topic_name, queue_length, callback);
    }
    ros::Subscriber sub_;
  };

  template<typename Cb, typename Msg>
  auto subscribe(Subscriber<Msg> &sub, NodeHandle &nh, const std::string &topic, size_t queue_length, Cb callback) {
      sub.subscribe(node_handle_, topic_name, queue_length,
      return sub;
  }
};

using ROSAdapter = ROS1Adapter;

}

#include <icey.hpp> 
