/// Test here Result
/// unpack
/// test timeout filter.
/// synchronize_with_transform
/// test serialize
auto timer_signal = icey.create_timer(period_time);

  /// Receive timer updates
  const auto [a_float, a_strin, an_int] =
      timer_signal
          .then([node](size_t ticks) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Timer ticked: " << ticks);
            return std::make_tuple(3., "hellooo", 66);
          }).unpack();

