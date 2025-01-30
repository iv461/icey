/// Support for the tf2 message filter
#pragma once

#ifndef ICEY_ROS2_WAS_INCLUDED
#error \
    "You must first include the <icey/icey.hpp> header before you can include the <icey/icey_image_transport.hpp> header."
#endif

#include <functional>
#include <tuple>

#include "tf2_ros/message_filter.h"

namespace icey {

template<class _Value>
struct TF2MessageFilterImpl {
  using MFLFilter = tf2_ros::MessageFilter< Message >;

  explicit TF2MessageFilterImpl(std::string target_frame) {
    
  }

  SimpleFilterAdapter input_adapter_;
  MFLFilter filter_;

};

template<class _Value>
struct TF2MessageFilter : public Stream<_Value, std::string,
  TF2MessageFilterImpl<_Value> > {
  using Self = TF2MessageFilter<_Value>;
  using Message = _Value;
  using MFLFilter = typename TF2MessageFilterImpl<_Value>::MFLFilter;
  
  explicit TF2MessageFilter(std::string target_frame, 
    const Duration &buffer_timeout) {
      this->impl()->name = "tf_filter";
      this->impl()->attach_ = [impl = this->impl(), target_frame]
        (NodeBookkeeping &node) {
          impl->filter_ = 
            std::make_shared<>(impl->input_adapter_, 
              target_frame, 
                10, 
                node.node_.get_node_logging_interface(),
                node.node_.get_node_clock_interface(),
                buffer_timeout),;
          impl->filter->registerCallback(&Self::on_message, this);
      };
  }

  void on_message(const typename _Value::SharedPtr &msg) {
    this->impl()->resolve(msg);
  } 
};

}  // namespace icey