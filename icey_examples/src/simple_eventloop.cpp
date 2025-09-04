//#define ICEY_CORO_DEBUG_PRINT
#include <deque>
#include <icey/impl/promise.hpp>
#include <iostream>
#include <thread>
#include <vector>

/// Jigsaw has captured a C++ programmer and want's him to play this game: He has to explain
/// asynchronous programming without using the word "thread".
using namespace std::chrono_literals;
struct EventLoop {
  void dispatch(auto &&event) { events.push_back(event); }

  void spin() {
    do {
      if (timer_added_cnt_ < number_of_times_) {
        events.push_back(timer_event_);
        timer_added_cnt_++;
      }
      auto event = events.front();
      events.pop_front();
      if (event) event();

    } while (!events.empty());
  }

  void set_timer(auto timer_event, size_t number_of_times) {
    timer_event_ = timer_event;
    number_of_times_ = number_of_times;
  }

  std::function<void()> timer_event_;
  size_t number_of_times_{0};
  size_t timer_added_cnt_{0};
  std::deque<std::function<void()>> events;
};

icey::impl::Promise<float> obtain_the_number_async(EventLoop &event_loop) {
  std::cout << "In obtain_the_number_async" << std::endl;
  return {[&](auto &promise) {
    event_loop.dispatch([&]() {
      std::this_thread::sleep_for(10ms);
      std::cout << "resolving promise" << std::endl;
      promise.resolve(42.1);
    });
  }};
}

icey::Promise<int> wrapper2(EventLoop &event_loop) {
  std::cout << "Before obtain_the_number_async" << std::endl;

  float res = co_await obtain_the_number_async(event_loop);
  std::cout << "After obtain_the_number_async" << std::endl;
  co_return int(res);
}

icey::Promise<int> wrapper1(EventLoop &event_loop) {
  std::cout << "In wrapper1" << std::endl;
  int res = co_await wrapper2(event_loop);
  co_return res;
}

int main() {
  EventLoop event_loop;

  std::cout << "Starting event loop " << std::endl;
  event_loop.set_timer(
      [&]() {
        std::cout << "Before event" << std::endl;
        wrapper1(event_loop);
        std::cout << "After event loop" << std::endl;
      },
      100);
  std::cout << "After set_timer, starting the event loop ... " << std::endl;
  event_loop.spin();
  std::cout << "Done." << std::endl;
  event_loop.events.clear();
}