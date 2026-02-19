/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <deque>
#include <functional>
#include <icey/impl/promise.hpp>
#include <string>
#include <vector>

namespace {

icey::Promise<int> immediate_plain() { co_return 7; }

icey::Promise<int, std::string> immediate_result_ok() {
  co_return icey::impl::PromiseState<int, std::string>{
      icey::Result<int, std::string>(icey::Ok<int>(5))};
}

icey::Promise<int, std::string> immediate_result_err() {
  co_return icey::impl::PromiseState<int, std::string>{
      icey::Result<int, std::string>(icey::Err<std::string>("immediate_error"))};
}

struct InlineEventLoop {
  std::deque<std::function<void()>> events;
  void dispatch(std::function<void()> f) { events.push_back(std::move(f)); }
  void run_all() {
    while (!events.empty()) {
      auto f = std::move(events.front());
      events.pop_front();
      f();
    }
  }
};

icey::impl::Promise<int> deferred_plain(InlineEventLoop &loop, int value) {
  return {[&loop, value](auto &promise) { loop.dispatch([&promise, value]() { promise.resolve(value); }); }};
}

icey::impl::Promise<int, std::string> deferred_result(InlineEventLoop &loop, bool fail) {
  return {[&loop, fail](auto &promise) {
    loop.dispatch([&promise, fail]() {
      if (fail) {
        promise.reject("inner_error");
      } else {
        promise.resolve(40);
      }
    });
  }};
}

icey::Promise<void> nested_plain_log(InlineEventLoop &loop, std::vector<std::string> &events) {
  events.emplace_back("outer_start");
  int value = co_await deferred_plain(loop, 41);
  events.emplace_back(std::to_string(value + 1));
  co_return;
}

icey::Promise<void> nested_result_log(InlineEventLoop &loop, bool fail,
                                      std::vector<std::string> &events) {
  events.emplace_back("outer_start");
  auto result = co_await deferred_result(loop, fail);
  if (result.has_value()) {
    events.emplace_back(std::to_string(result.value() + 2));
  } else {
    events.emplace_back(result.error());
  }
  co_return;
}

icey::Promise<void> cancellation_in_scope(bool &cancelled) {
  auto pending = icey::impl::Promise<int, std::string>([&cancelled](auto &promise) {
    promise.set_cancel([&cancelled](auto &) { cancelled = true; });
  });
  (void)pending;
  co_return;
}

}  // namespace

TEST(IceyPromiseAsyncAwaitTest, AwaitImmediateCoReturnWithoutErrorType) {
  bool completed = false;
  int value = 0;

  [&]() -> icey::Promise<void> {
    value = co_await immediate_plain();
    completed = true;
    co_return;
  }();

  EXPECT_TRUE(completed);
  EXPECT_EQ(value, 7);
}

TEST(IceyPromiseAsyncAwaitTest, AwaitImmediateCoReturnWithErrorType) {
  bool completed = false;
  icey::Result<int, std::string> ok_result{icey::Err<std::string>("unset")};
  icey::Result<int, std::string> err_result{icey::Err<std::string>("unset")};

  [&]() -> icey::Promise<void> {
    ok_result = co_await immediate_result_ok();
    err_result = co_await immediate_result_err();
    completed = true;
    co_return;
  }();

  EXPECT_TRUE(completed);
  EXPECT_TRUE(ok_result.has_value());
  EXPECT_EQ(ok_result.value(), 5);
  EXPECT_TRUE(err_result.has_error());
  EXPECT_EQ(err_result.error(), "immediate_error");
}

TEST(IceyPromiseAsyncAwaitTest, NestedCoroutinesWithoutErrorType) {
  InlineEventLoop loop;
  std::vector<std::string> events;
  auto nested = nested_plain_log(loop, events);
  loop.run_all();
  std::vector<std::string> expected{"outer_start", "42"};
  EXPECT_EQ(events, expected);
  (void)nested;
}

TEST(IceyPromiseAsyncAwaitTest, NestedCoroutinesWithErrorType) {
  InlineEventLoop loop;
  std::vector<std::string> events_ok;
  std::vector<std::string> events_err;
  auto nested_ok = nested_result_log(loop, false, events_ok);
  auto nested_err = nested_result_log(loop, true, events_err);
  loop.run_all();
  std::vector<std::string> expected_ok{"outer_start", "42"};
  std::vector<std::string> expected_err{"outer_start", "inner_error"};
  EXPECT_EQ(events_ok, expected_ok);
  EXPECT_EQ(events_err, expected_err);
  (void)nested_ok;
  (void)nested_err;
}

TEST(IceyPromiseAsyncAwaitTest, CancellationInUnawaitedCoroutineScopeIsNotTriggered) {
  bool cancelled = false;

  cancellation_in_scope(cancelled);

  EXPECT_FALSE(cancelled);
}
