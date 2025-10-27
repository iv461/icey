/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

/// This test tests whether the Streams behave like promises in JavaScript, especially
/// regarding catch-handlers executed correctly (fall-through behavior).
/// Note that for our use-case, we need only resolving with values, not with other promises.
/// In the following, "put_value" corresponds to "resolve" and "put_error" corresponds to "reject"
/// of a stream.
#include <gtest/gtest.h>

#include <icey/impl/stream.hpp>
#include <iostream>

class PromiseTest : public testing::Test {
protected:
  enum MarkerBehavior { Some, None, Err };
  template <MarkerBehavior behavior>
  auto marker(size_t i) {
    return [this, i](auto) {
      events.push_back(i);
      // std::cout << "Marker " << i << " called with val " << v << std::endl;
      if constexpr (behavior == MarkerBehavior::Some)
        return std::string("marker_from_" + std::to_string(i));
      else if constexpr (behavior == MarkerBehavior::None)
        return std::optional<std::string>{};
      else
        return icey::Err("erroring_" + std::to_string(i));
    };
  }

  using ResolveValue = std::string;
  using ErrorValue = std::string;
  using Stream = icey::impl::Stream<ResolveValue, ErrorValue, icey::Nothing, icey::Nothing>;
  std::shared_ptr<Stream> stream{icey::impl::create_stream<Stream>()};

  std::vector<size_t> events;
};

/// Smoke, tests correct chaining and skipping of except-handler in case of no exception, including
/// skipping everything after the except-handler
TEST_F(PromiseTest, Smoke) {
  stream->then(marker<Some>(1))
      ->then(marker<Some>(2))
      ->then(marker<Some>(3))
      ->except(marker<Some>(4))
      ->then(marker<Some>(5));

  EXPECT_TRUE(events.empty());  /// Negative test
  stream->put_value("hello");

  std::vector<size_t> target_order1{1, 2, 3};

  EXPECT_EQ(target_order1, events);
  events.clear();

  stream->put_error("BigExcEptiOn");
  std::vector<size_t> target_order2{4, 5};
  EXPECT_EQ(target_order2, events);
}

/// Test error from then-handler and no return from except handler
TEST_F(PromiseTest, VoidCatch) {
  stream->then(marker<Some>(1))
      ->then(marker<Err>(2))
      ->then(marker<Some>(3))
      ->except(marker<None>(4))
      ->then(marker<Some>(5));

  EXPECT_TRUE(events.empty());

  stream->put_value("hello2");
  std::vector<size_t> target_order{1, 2, 4};
  EXPECT_EQ(target_order, events);

  events.clear();
  stream->put_error("BigExcEptiOn");
  target_order = {4};
  EXPECT_EQ(target_order, events);
}

/// Test that .then() can error
TEST_F(PromiseTest, ThenErroring) {
  stream->then(marker<Err>(1))
      ->then(marker<Err>(2))
      ->then(marker<Some>(3))
      ->except(marker<Err>(4))
      ->then(marker<Some>(6))
      ->then(marker<Err>(7));

  EXPECT_TRUE(events.empty());

  stream->put_value("GoodVal");
  std::vector<size_t> target_order{1, 4, 5};
  EXPECT_EQ(target_order, events);
}

/// Test whether a .then() handler that returns void still propagates correctly errors
TEST_F(PromiseTest, VoidThenPropagating) {
  stream->then(marker<Some>(1))
      ->then(marker<Some>(2))
      ->then(marker<Err>(3))
      ->then(marker<None>(4))
      ->except(marker<Some>(5))
      ->then(marker<Some>(6));

  EXPECT_TRUE(events.empty());

  stream->put_value("GoodVal");
  std::vector<size_t> target_order{1, 2, 3, 5, 6};
  EXPECT_EQ(target_order, events);

  events.clear();
  stream->put_error("BigExcEptiOn3");
  target_order = {5, 6};
  EXPECT_EQ(target_order, events);
}

/// Test whether there is no value after calling .take()
TEST_F(PromiseTest, TakeTest) {
  const std::string value = "GoodVal";
  stream->put_value(value);

  EXPECT_TRUE(stream->has_value());
  EXPECT_FALSE(stream->has_error());
  EXPECT_FALSE(stream->has_none());

  EXPECT_EQ(stream->value(), value);

  icey::Result<std::string, std::string> current_state = stream->take();

  EXPECT_FALSE(stream->has_value());
  EXPECT_TRUE(stream->has_none());

  EXPECT_TRUE(current_state.has_value());
  EXPECT_EQ(current_state.value(), value);
}
