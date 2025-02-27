/// Tests whether all functions accepting callbacks copy the callback so that it lives for as long
/// as the stream lives. The callbacks should be copied regardless of whether they are lvalues or
/// rvalues, so both cases are tested. We also assert that the callback is copied exactly once and
/// not unnecessarily many times.
#include <memory>

#include "node_fixture.hpp"
/// A class to track the copies, moves and destructions by using a weak_ptr.
struct CopyTracker {
  CopyTracker(auto *ctx) : value(ctx->copy_tracker_value) {}
  std::shared_ptr<int> value;
};

class CallbackLifetimeTest : public NodeTest {
public:
  std::shared_ptr<int> copy_tracker_value{std::make_shared<int>(0)};
  bool callback_was_called{false};
};

TEST_F(CallbackLifetimeTest, ThenRvalue) {
  auto int_stream = node_->icey().create_stream<icey::Stream<int, std::string>>();
  /// We track the copies of the clojure object (aka lambda-function) by capturing the "CopyTracker"
  /// object by value. This way, each time the clojure is copied, the "CopyTracker" is copied as
  /// well, tracking these copies. First, we test the rvalue case, where the clojure gets created
  /// while calling the
  int_stream.then([this, copy_tracker = CopyTracker(this)](auto) { callback_was_called = true; });

  int_stream.impl()->put_value(5);
  EXPECT_TRUE(callback_was_called);
  /// The count must be 2: This means once it is referenced by the fixture and once by the callback
  /// clojure that is copied inside the stream. If the callback clojure would not be copied, this
  /// would be 1.
  EXPECT_EQ(copy_tracker_value.use_count(), 2);
}

TEST_F(CallbackLifetimeTest, ThenLvalue) {
  auto int_stream = node_->icey().create_stream<icey::Stream<int, std::string>>();
  {
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker = CopyTracker(this)](auto) {
      callback_was_called = true;
      EXPECT_EQ(copy_tracker_value.use_count(), 2);
    };
    int_stream.then(lvalue_lambda);
  }

  int_stream.impl()->put_value(6);
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, ExceptRvalue) {
  auto int_stream = node_->icey().create_stream<icey::Stream<int, std::string>>();
  /// Pass an rvalue reference lambda function, should get copied
  int_stream.except([this, copy_tracker = CopyTracker(this)](auto) {
    callback_was_called = true;
    EXPECT_EQ(copy_tracker_value.use_count(), 2);
  });

  int_stream.impl()->put_error("error");
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, ExceptLvalue) {
  auto int_stream = node_->icey().create_stream<icey::Stream<int, std::string>>();
  {
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker = CopyTracker(this)](auto) {
      callback_was_called = true;
      EXPECT_EQ(copy_tracker_value.use_count(), 2);
    };
    int_stream.except(lvalue_lambda);
  }

  int_stream.impl()->put_error("error");
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, FilterRvalue) {
  auto int_stream = node_->icey().create_stream<icey::Stream<int, std::string>>();
  /// Pass an rvalue reference lambda function, should get copied
  int_stream.filter([this, copy_tracker = CopyTracker(this)](auto) {
    callback_was_called = true;
    EXPECT_EQ(copy_tracker_value.use_count(), 2);

    return true;
  });

  int_stream.impl()->put_value(7);
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, FilterLvalue) {
  auto int_stream = node_->icey().create_stream<icey::Stream<int, std::string>>();
  {
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker = CopyTracker(this)](auto) {
      callback_was_called = true;
      EXPECT_EQ(copy_tracker_value.use_count(), 2);
      return true;
    };
    int_stream.filter(lvalue_lambda);
  }

  int_stream.impl()->put_value(7);
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, UnwrapOrRvalue) {
  auto int_stream = node_->icey().create_stream<icey::Stream<int, std::string>>();
  /// Pass an rvalue reference lambda function, should get copied
  int_stream.unwrap_or([this, copy_tracker = CopyTracker(this)](auto) {
    callback_was_called = true;
    EXPECT_EQ(copy_tracker_value.use_count(), 2);
  });

  int_stream.impl()->put_error("error");
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, UnwrapOrLvalue) {
  auto int_stream = node_->icey().create_stream<icey::Stream<int, std::string>>();
  {
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker = CopyTracker(this)](auto) {
      callback_was_called = true;
      EXPECT_EQ(copy_tracker_value.use_count(), 2);
      return true;
    };
    int_stream.unwrap_or(lvalue_lambda);
  }

  int_stream.impl()->put_error("error");
  EXPECT_TRUE(callback_was_called);
}
