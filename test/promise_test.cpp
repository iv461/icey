/// This test tests whether the Promises behave like promises in JavaScript, especially 
/// regarding catch-handlers executed correctly (fall-through behavior).
/// Note that for our use-case, we need only resolving with values, not with other promises
#include <icey/impl/observable.hpp>
#include <iostream>
#include <gtest/gtest.h>

class PromiseTest : public testing::Test {
 protected:
  
  PromiseTest() {
     // You can do set-up work for each test here.
  }

  ~PromiseTest() override {
     // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override {
     // Code here will be called immediately after the constructor (right
     // before each test).
  }

  void TearDown() override {
     // Code here will be called immediately after each test (right
     // before the destructor).
  }

   enum MarkerBehavior { Some, None, Err };
   template<MarkerBehavior behavior>
  auto marker(size_t i) {
    return [this, i](auto v) {
      using Result = icey::Result<std::string, std::string>;
        events.push_back(i);
        std::cout << "Marker " << i << " called with val " << v << std::endl;
        if constexpr(behavior == MarkerBehavior::Some)
         return std::string("marker_from_" + std::to_string(i));
        else if constexpr (behavior == MarkerBehavior::None)
         return std::optional<std::string>{};
       else
         return Result::Err("erroring_" + std::to_string(i));
    };
  }
  size_t marker_counter{0};

  std::vector<size_t> events;
};

/// Smoke, tests correct chaining and skipping of except-handler in case of no exception, including skipping everything after the except-handler
TEST_F(PromiseTest, Smoke) {
    using ResolveValue = std::string;
    using ErrorValue = std::string;
    auto promise = icey::create_observable< icey::impl::Observable<ResolveValue, ErrorValue> > ();

   promise
        ->then(marker<Some>(1))
        ->then(marker<Some>(2))
        ->then(marker<Some>(3))
        ->except(marker<Some>(4))
        ->then(marker<Some>(5));

   EXPECT_TRUE(events.empty()); /// Negative test
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
    using ResolveValue = std::string;
    using ErrorValue = std::string;
    auto promise = icey::create_observable< icey::impl::Observable<ResolveValue, ErrorValue> > ();

       promise
        ->then(marker<Some>(1))
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
    using ResolveValue = std::string;
    using ErrorValue = std::string;
    auto promise = icey::create_observable< icey::impl::Observable<ResolveValue, ErrorValue> > ();

       promise
        ->then(marker<Err>(1))
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
    using ResolveValue = std::string;
    using ErrorValue = std::string;
    auto promise = icey::create_observable< icey::impl::Observable<ResolveValue, ErrorValue> > ();

       promise
        ->then(marker<Some>(1))
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
