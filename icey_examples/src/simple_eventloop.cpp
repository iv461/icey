#define ICEY_CORO_DEBUG_PRINT
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
    for (auto ev : events) ev();
  }
  std::vector<std::function<void()>> events;
};

icey::Task<int> obtain_the_number_async(EventLoop &event_loop) {
  co_return [&](auto &promise, auto &) {
    event_loop.dispatch([&]() {
      std::this_thread::sleep_for(10ms);
      promise.resolve(42);
    });
  };
}


icey::Task<int> wrapper1(EventLoop &event_loop) {
  int res = co_await obtain_the_number_async(event_loop);
  co_return res;
}


icey::Task<void> do_async_stuff(EventLoop &event_loop) {
  std::cout << "Before obtain_the_number_async" << std::endl;
  event_loop.dispatch([&]() {
    const auto f = [&]() -> icey::Task<void> {
      std::cout << "Before obtain_the_number_async" << std::endl;
      int the_number = co_await wrapper1(event_loop);//co_await obtain_the_number_async(event_loop);
      std::cout << "Obtained number: " << the_number << std::endl;
      co_return;
    };
    for (size_t i = 0; i < 1000; i++) {
      f().force_destruction();
    }
  });
  co_return;
}


int main() {
  {
    EventLoop event_loop;

    std::cout << "E3 Before do_async_stuff " << std::endl;
    do_async_stuff(event_loop);
    std::cout << "After do_async_stuff, starting the event loop ... " << std::endl;
    event_loop.spin();
    std::cout << "Done" << std::endl;
    event_loop.events.clear();
  }
}