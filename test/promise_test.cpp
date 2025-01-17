/// This test tests whether the Promises behave like promises as specified in JavaScript, especially 
/// regarding catch-handlers executed correctly (fall-through behavior).

#include <icey/icey_ros2.hpp>
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

  auto marker(size_t i) {
    return [this, i](auto v) {
        events.push_back(i);
        std::cout << "Marker " << i << " called " << std::endl;
        return std::string("marker_from_" + std::to_string(i));
    };
  }

  using Promise = icey::Observable<std::string>; 

  std::shared_ptr<icey::Context> ctx_{std::make_shared<icey::Context>()};
  std::vector<size_t> events;
};

TEST_F(PromiseTest, Fallthrough) {
    using ResolveValue = std::string;
    using ErrorValue = int;
    auto my_promise2 = ctx_->create_observable< icey::Observable<ResolveValue, ErrorValue> >();

    my_promise2
        ->then(marker(0))
        ->then(marker(1))
        ->except(marker(2))
        ->then(marker(3));

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
