#define ICEY_CORO_DEBUG_PRINT
#include <deque>
#include <icey/impl/promise.hpp>
#include <iostream>
#include <thread>
#include <vector>

/// Jigsaw has captured a C++ programmer and want's him to play this game: He has to explain
/// asynchronous programming without using the word "thread".
using namespace std::chrono_literals;
struct EventLoop {
  void dispatch(auto &&event) {
    // std::cout << "Dispatching event " << std::endl;
    events.push_back(event);
  }

  void spin() {
    do {
      if (timer_added_cnt < 100) {
        events.push_back(timer_ev);
        timer_added_cnt++;
      }
      auto event = events.front();  // Get the first event
      // events.erase(events.begin());
      events.pop_front();
      // std::cout << "Before ex event" << std::endl;
      if (event) event();
      // std::cout << "After ex event" << std::endl;
    } while (!events.empty());
  }

  void set_timer(auto timer_event) { timer_ev = timer_event; }

  std::function<void()> timer_ev;
  size_t timer_added_cnt{0};
  std::deque<std::function<void()>> events;
};

icey::Task<int> obtain_the_number_async(EventLoop &event_loop) {
  std::cout << "In obtain_the_number_async" << std::endl;
  return icey::Task<int>([&](auto &promise, auto &) {
    event_loop.dispatch([&]() {
      std::this_thread::sleep_for(10ms);
      std::cout << "resolving promise" <<std::endl;
      promise.resolve(42);
    });
  });
}

icey::Task<int> wrapper2(EventLoop &event_loop) {
  std::cout << "b4 obtain_the_number_async" << std::endl;
  int res = co_await obtain_the_number_async(event_loop);
  std::cout << "After obtain_the_number_async" << std::endl;

  std::cout << "b4 obtain_the_number_async2" << std::endl;
  int res2 = co_await obtain_the_number_async(event_loop);
  std::cout << "After obtain_the_number_async2" << std::endl;
  co_return res;
}

icey::Task<int> wrapper1(EventLoop &event_loop) {
  std::cout << "In wrapper1" << std::endl;
  int res = co_await wrapper2(event_loop);
  co_return res;
}

/// TODO using sync crashes since we destroy too early, fix this
icey::Task<int> obtain_the_number_sync() { 
    co_return 42; 
}


int main() {
  {
    EventLoop event_loop;

    std::cout << "E3 Before do_async_stuff " << std::endl;
    event_loop.set_timer([&]() {
      const auto c = [&]() -> icey::Task<void> {
        std::cout << "Before obtain_the_number_async" << std::endl;
        int the_number = co_await wrapper1(event_loop);
        std::cout << "After obtain_the_number_async, the number: " << the_number << std::endl;
        co_return;
      };
      c().resume();
    });
    std::cout << "After do_async_stuff, starting the event loop ... " << std::endl;
    event_loop.spin();
    std::cout << "Done" << std::endl;
    event_loop.events.clear();
  }
}