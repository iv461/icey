//#define ICEY_CORO_DEBUG_PRINT
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include <deque>
#include <icey/impl/promise.hpp>
#include <iostream>
#include <thread>
#include <vector>
/// This tests whether nested coroutines work, i.e. have the correct control flow.

using namespace std::chrono_literals;
struct EventLoop : testing::Test {
  void dispatch(auto &&event) { events.push_back(event); }

  void spin() {
    events.push_back(timer_event_);
    timer_added_cnt_++;

    do {
      auto event = events.front();
      events.pop_front();
      if (event) event();

      if (events.empty() && timer_added_cnt_ < number_of_times_) {
        events.push_back(timer_event_);
        timer_added_cnt_++;
      }
      events_processed++;
    } while (!events.empty());
  }

  void set_timer(auto timer_event, size_t number_of_times) {
    timer_event_ = timer_event;
    number_of_times_ = number_of_times;
  }
  size_t events_processed{0};
  std::function<void()> timer_event_;
  size_t number_of_times_{0};
  size_t timer_added_cnt_{0};
  std::deque<std::function<void()>> events;

  std::vector<std::string> event_log;
};

icey::impl::Promise<float> obtain_the_number_async(EventLoop &event_loop) {
  // std::cout << "obtain_the_number_async" << std::endl;
  event_loop.event_log.push_back("obtain_the_number_async");
  return {[&](auto &promise) {
    event_loop.dispatch([&]() {
      std::this_thread::sleep_for(10ms);
      // std::cout << "resolving_promise" << std::endl;
      event_loop.event_log.push_back("resolving_promise");
      promise.resolve(42 + event_loop.timer_added_cnt_ * 3.);
    });
  }};
}

icey::Promise<int> wrapper2(EventLoop &event_loop) {
  // std::cout << "before_wrapper2" << std::endl;
  event_loop.event_log.push_back("before_wrapper2");
  float res = co_await obtain_the_number_async(event_loop);
  // std::cout << "after_wrapper2" << std::endl;
  event_loop.event_log.push_back("after_wrapper2");
  // std::cout << std::to_string(res) << std::endl;
  event_loop.event_log.push_back(std::to_string(res));
  co_return int(res);
}

icey::Promise<int> wrapper1(EventLoop &event_loop) {
  event_loop.event_log.push_back("wrapper1");
  // std::cout << "wrapper1" << std::endl;
  int res = co_await wrapper2(event_loop);
  co_return res;
}

TEST_F(EventLoop, NestedCoroutines) {
  const size_t num_iterations = 10;
  this->set_timer(
      [&]() {
        // std::cout << "before_coro" << std::endl;
        event_log.push_back("before_coro");
        wrapper1(*this).detach();
        event_log.push_back("after_coro");
        // std::cout << "after_coro" << std::endl;
      },
      num_iterations);

  ASSERT_EQ(events_processed, 0);
  ASSERT_TRUE(event_log.empty());
  this->spin();

  EXPECT_EQ(event_log.size(), num_iterations * 8);
  for (size_t i = 0; i < num_iterations; i++) {
    for (size_t j = 0; j < 8; j++) {
      std::string target;
      switch (j) {
        case 0:
          target = "before_coro";
          break;
        case 1:
          target = "wrapper1";
          break;
        case 2:
          target = "before_wrapper2";
          break;
        case 3:
          target = "obtain_the_number_async";
          break;
        case 4:
          target = "after_coro";
          break;
        case 5:
          target = "resolving_promise";
          break;
        case 6:
          target = "after_wrapper2";
          break;
        case 7:
          target = std::to_string(42. + (i + 1) * 3);
          break;
      }
      EXPECT_EQ(event_log.at(i * 8 + j), target);
    }
  }
}