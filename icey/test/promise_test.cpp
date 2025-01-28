/// This test tests whether the Promises behave like promises in JavaScript, especially
/// regarding catch-handlers executed correctly (fall-through behavior).
/// Note that for our use-case, we need only resolving with values, not with other promises
#include <gtest/gtest.h>

#include <icey/impl/stream.hpp>
#include <iostream>

class PromiseTest : public testing::Test {
protected:
  enum MarkerBehavior { Some, None, Err };
  template <MarkerBehavior behavior>
  auto marker(size_t i) {
    return [this, i](auto v) {
      using Result = icey::Result<std::string, std::string>;
      events.push_back(i);
      std::cout << "Marker " << i << " called with val " << v << std::endl;
      if constexpr (behavior == MarkerBehavior::Some)
        return std::string("marker_from_" + std::to_string(i));
      else if constexpr (behavior == MarkerBehavior::None)
        return std::optional<std::string>{};
      else
        return Result::Err("erroring_" + std::to_string(i));
    };
  }

  using ResolveValue = std::string;
  using ErrorValue = std::string;
  using Obs = icey::impl::Stream<ResolveValue, ErrorValue>;
  std::shared_ptr<Obs> promise{icey::impl::create_observable<Obs>()};

  std::vector<size_t> events;
};

/// Smoke, tests correct chaining and skipping of except-handler in case of no exception, including
/// skipping everything after the except-handler
TEST_F(PromiseTest, Smoke) {
  promise->then(marker<Some>(1))
      ->then(marker<Some>(2))
      ->then(marker<Some>(3))
      ->except(marker<Some>(4))
      ->then(marker<Some>(5));

  EXPECT_TRUE(events.empty());  /// Negative test
  promise->resolve("hello");

  std::vector<size_t> target_order1{1, 2, 3};

  EXPECT_EQ(target_order1, events);
  events.clear();

  promise->reject("BigExcEptiOn");
  std::vector<size_t> target_order2{4, 5};
  EXPECT_EQ(target_order2, events);
}

/// Test error from then-handler and no return from except handler
TEST_F(PromiseTest, VoidCatch) {
  promise->then(marker<Some>(1))
      ->then(marker<Err>(2))
      ->then(marker<Some>(3))
      ->except(marker<None>(4))
      ->then(marker<Some>(5));

  EXPECT_TRUE(events.empty());

  promise->resolve("hello2");
  std::vector<size_t> target_order{1, 2, 4};
  EXPECT_EQ(target_order, events);

  events.clear();
  promise->reject("BigExcEptiOn");
  target_order = {4};
  EXPECT_EQ(target_order, events);
}

/// Test that .then() can error
TEST_F(PromiseTest, ThenErroring) {
  promise->then(marker<Err>(1))
      ->then(marker<Err>(2))
      ->then(marker<Some>(3))
      ->except(marker<Err>(4))
      ->except(marker<Err>(5))
      ->then(marker<Some>(6))
      ->then(marker<Err>(7));

  EXPECT_TRUE(events.empty());

  promise->resolve("GoodVal");
  std::vector<size_t> target_order{1, 4, 5};
  EXPECT_EQ(target_order, events);
}

/// Test whether a .then() handler that returns void still propagates correctly errors
TEST_F(PromiseTest, VoidThenPropagating) {
  promise->then(marker<Some>(1))
      ->then(marker<Some>(2))
      ->then(marker<Err>(3))
      ->then(marker<None>(4))
      ->except(marker<Some>(5))
      ->then(marker<Some>(6));

  EXPECT_TRUE(events.empty());

  promise->resolve("GoodVal");
  std::vector<size_t> target_order{1, 2, 3, 5, 6};
  EXPECT_EQ(target_order, events);

  events.clear();
  promise->reject("BigExcEptiOn3");
  target_order = {5, 6};
  EXPECT_EQ(target_order, events);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
