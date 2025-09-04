#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
using namespace std::chrono_literals;

class TimerBenchmarkNode : public rclcpp::Node {
public:
  TimerBenchmarkNode(size_t total_timers, std::chrono::milliseconds period)
      : Node("timer_benchmark"), total_timers_(total_timers), completed_(0) {
    timers_.reserve(total_timers_);
    start_time_ = std::chrono::steady_clock::now();

    for (size_t i = 0; i < total_timers_; ++i) {
      auto timer = this->create_wall_timer(period, [this, i]() {
        // Cancel the timer inside the callback
        timers_[i]->cancel();

        // Count completions
        if (++completed_ == total_timers_) {
          auto end_time = std::chrono::steady_clock::now();
          auto duration_ms =
              std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count();

          RCLCPP_INFO(this->get_logger(), "Processed %zu timers in %ld ms (%.0f timers/sec)",
                      total_timers_, duration_ms, (total_timers_ * 1000.0) / duration_ms);

          // Destroy timers explicitly
          timers_.clear();
          std::exit(0);
        }
      });

      timers_.push_back(timer);
    }
  }

private:
  size_t total_timers_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::atomic<size_t> completed_;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  getchar();
  size_t total_timers = 10000;

  auto node = std::make_shared<TimerBenchmarkNode>(total_timers, 0ms);
  
//   auto exec = rclcpp::experimental::executors::EventsExecutor()
//   exec.add(node);
//   exec.spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}