/// Tests whether all functions accepting callbacks copy the callback so that it lives for as long
/// as the stream lives. The callbacks should be copied regardless of whether they are lvalues or
/// rvalues, so both cases are tested. We also assert that the callback is copied exactly once and
/// not unnecessarily many times.
#include "node_fixture.hpp"

/// A class to track the copies, moves and destructions
struct CopyTracker {
  CopyTracker(auto *ctx)
      : times_copied(&ctx->times_callback_was_copied), dtor_called(&ctx->callback_was_destructed) {
    std::cout << "CopyTracker was constructed" << std::endl;
  }
  CopyTracker(const CopyTracker &other) : times_copied(other.times_copied),dtor_called(other.dtor_called) {
    std::cout << "CopyTracker was copied" << std::endl;    
    (*times_copied)++;
  }
  CopyTracker(CopyTracker &&other) : times_copied(other.times_copied),dtor_called(other.dtor_called) {
    std::cout << "CopyTracker was was move-constructed" << std::endl;
  }
  CopyTracker &operator=(const CopyTracker &other) {
    std::cout << "CopyTracker was was copy-assigned" << std::endl;
    times_copied = other.times_copied;
    dtor_called = other.dtor_called;
    (*times_copied)++;
    return *this;
  }
  CopyTracker &operator=(CopyTracker &&other) {
    std::cout << "CopyTracker was was move-assigned" << std::endl;
    times_copied = other.times_copied;
    dtor_called = other.dtor_called;
    return *this;
  }
  ~CopyTracker() {
    std::cout << "Destructor was called" << std::endl;
    *dtor_called = true; 
  }
  std::size_t *times_copied{0};
  /// A variable that we write true in case the destructor was called.
  /// Since the object gets destroyed, we cannot store this flag into the object itself.
  /// Note that reading from an object after it's destructor was called is undefined behavior.
  /// We also have to store a pointer an cannot store a reference because we need to be able to copy
  /// this object.
  bool *dtor_called{nullptr};
};

class CallbackLifetimeTest : public NodeTest {
public:
  icey::Stream<int, std::string> int_stream{
      node_->icey().create_stream<icey::Stream<int, std::string>>()};

  std::size_t times_callback_was_copied{0};
  bool callback_was_called{false};
  bool callback_was_destructed{false};
};

TEST_F(CallbackLifetimeTest, ThenRvalue) {
  /// We track the copies of the clojure object (aka lambda-function) by capturing the "CopyTracker"
  /// object by value. This way, each time the clojure is copied, the "CopyTracker" is copied as
  /// well, tracking these copies. First, we test the rvalue case, where the clojure gets created
  /// while calling the
  int_stream.then([this, copy_tracker = CopyTracker(this)](auto) {
    callback_was_called = true;
    EXPECT_EQ(times_callback_was_copied, 1);
    EXPECT_FALSE(callback_was_destructed);
  });

  int_stream.impl()->put_value(5);
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, ThenLvalue) {
  {
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker = CopyTracker(this)](auto) {
      callback_was_called = true;
      EXPECT_EQ(times_callback_was_copied, 1);
      EXPECT_FALSE(callback_was_destructed);
    };
    int_stream.then(lvalue_lambda);
  }

  int_stream.impl()->put_value(6);
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, ExceptRvalue) {
  /// Pass an rvalue reference lambda function, should get copied
  int_stream.except([this, copy_tracker = CopyTracker(this)](auto) {
    callback_was_called = true;
    EXPECT_EQ(times_callback_was_copied, 1);
    EXPECT_FALSE(callback_was_destructed);
  });

  int_stream.impl()->put_error("error");
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, ExceptLvalue) {
  {
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker = CopyTracker(this)](auto) {
      callback_was_called = true;
      EXPECT_EQ(times_callback_was_copied, 1);
      EXPECT_FALSE(callback_was_destructed);
    };
    int_stream.except(lvalue_lambda);
  }

  int_stream.impl()->put_error("error");
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, FilterRvalue) {
  /// Pass an rvalue reference lambda function, should get copied
  int_stream.filter([this, copy_tracker = CopyTracker(this)](auto) {
    callback_was_called = true;
    EXPECT_EQ(times_callback_was_copied, 1);
    EXPECT_FALSE(callback_was_destructed);
    return true;
  });

  int_stream.impl()->put_value(7);
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, FilterLvalue) {
  {
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker = CopyTracker(this)](auto) {
      callback_was_called = true;
      EXPECT_EQ(times_callback_was_copied, 1);
      EXPECT_FALSE(callback_was_destructed);
      return true;
    };
    int_stream.filter(lvalue_lambda);
  }

  int_stream.impl()->put_value(7);
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, UnwrapOrRvalue) {
  /// Pass an rvalue reference lambda function, should get copied
  int_stream.unwrap_or([this, copy_tracker = CopyTracker(this)](auto) {
    callback_was_called = true;
    EXPECT_EQ(times_callback_was_copied, 1);
    EXPECT_FALSE(callback_was_destructed);
  });

  int_stream.impl()->put_error("error");
  EXPECT_TRUE(callback_was_called);
}

TEST_F(CallbackLifetimeTest, UnwrapOrLvalue) {
  {
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker = CopyTracker(this)](auto) {
      callback_was_called = true;
      EXPECT_EQ(times_callback_was_copied, 1);
      EXPECT_FALSE(callback_was_destructed);
      return true;
    };
    int_stream.unwrap_or(lvalue_lambda);
  }

  int_stream.impl()->put_error("error");
  EXPECT_TRUE(callback_was_called);
}
