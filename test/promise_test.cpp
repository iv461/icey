/// This test tests whether the Promises behave like promises as specified in JavaScript, especially 
/// regarding catch-handlers executed correctly (fall-through behavior).

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
        std::cout << "Marker " << i << " called " << std::endl;
        if constexpr(behavior == MarkerBehavior::Some)
         return std::string("marker_from_" + std::to_string(i));
        else if constexpr (behavior == MarkerBehavior::None)
         return std::optional<std::string>{};
       else
         return Result::Err("erroring_" + std::to_string(i));
    };
  }

  std::vector<size_t> events;
};

TEST_F(PromiseTest, Fallthrough) {
    using ResolveValue = std::string;
    using ErrorValue = int;

    auto my_promise2 = icey::create_observable< icey::Observable<ResolveValue, ErrorValue> > ();

    my_promise2
        ->then(marker<Some>(1))
        ->then(marker<Err>(2))
        ->then(marker<Some>(3))
        ->except(marker<None>(4))
        ->then(marker<Some>(5));

    EXPECT_TRUE(events.empty());
    my_promise2->resolve("resolution");
    std::vector<size_t> target_order1{0, 1};

    EXPECT_EQ(target_order1, events);
    events.clear();

    my_promise2->reject(-3);

    std::vector<size_t> target_order2{2, 3};
    EXPECT_EQ(target_order2, events);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
