/// Tests whether all functions accepting callbacks copy the callback so that it lives for as long as the stream lives. 
/// The callbacks should be copied regardless of whether they are lvalues or rvalues, so both cases are tested.
/// We also assert that the callback is copied exactly once and not unnecessarily many times.
#include "node_fixture.hpp"

/// A class to track the copies, moves and destructions
struct CopyTracker {
    CopyTracker() {
        //std::cout << "CopyTracker was constructed" << std::endl;
    }
    CopyTracker(const CopyTracker &other): dtor_called(other.dtor_called) {
        //std::cout << "CopyTracker was copied" << std::endl;
        times_copied = other.times_copied + 1;
    }
    CopyTracker(CopyTracker &&other): dtor_called(other.dtor_called) {
        //std::cout << "CopyTracker was was move-constructed" << std::endl;
    }
    CopyTracker &operator=(const CopyTracker &other) {
        //std::cout << "CopyTracker was was copy-assigned" << std::endl;
        dtor_called = other.dtor_called;
        times_copied = other.times_copied + 1;
        return *this;
    }
    CopyTracker &operator=(CopyTracker &&other) {
        //std::cout << "CopyTracker was was move-assigned" << std::endl;
        dtor_called = other.dtor_called;
        return *this;
    }
    ~CopyTracker() {
        dtor_called = true;
    }
    std::size_t times_copied{0};
    /// Note that the memory does not get overwritten, therefore we can actually check whether something was destructed by reading this out.
    bool dtor_called{false};
};

class CallbackLifetimeTest : public NodeTest {
protected:
    icey::Stream<int, std::string> int_stream{node_->icey().create_stream<icey::Stream<int, std::string>>()};
    bool cb_called{false};
};

TEST_F(CallbackLifetimeTest, ThenRvalue) {

/// Pass an rvalue reference lambda function, should get copied
int_stream.then([this, copy_tracker=CopyTracker()](auto) {
    cb_called = true;
    EXPECT_EQ(copy_tracker.times_copied, 1);
    EXPECT_FALSE(copy_tracker.dtor_called);
});

int_stream.impl()->put_value(5);
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeTest, ThenLvalue) {
{
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker=CopyTracker()](auto) {
        cb_called = true;
        EXPECT_EQ(copy_tracker.times_copied, 1);
        EXPECT_FALSE(copy_tracker.dtor_called);
    };
    int_stream.then(lvalue_lambda);
}

int_stream.impl()->put_value(6);
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeTest, ExceptRvalue) {

/// Pass an rvalue reference lambda function, should get copied
int_stream.except([this, copy_tracker=CopyTracker()](auto) {
    cb_called = true;
    EXPECT_EQ(copy_tracker.times_copied, 1);
    EXPECT_FALSE(copy_tracker.dtor_called);
});

int_stream.impl()->put_error("error");
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeTest, ExceptLvalue) {
{
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker=CopyTracker()](auto) {
        cb_called = true;
        EXPECT_EQ(copy_tracker.times_copied, 1);
        EXPECT_FALSE(copy_tracker.dtor_called);
    };
    int_stream.except(lvalue_lambda);
}

int_stream.impl()->put_error("error");
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeTest, FilterRvalue) {

/// Pass an rvalue reference lambda function, should get copied
int_stream.filter([this, copy_tracker=CopyTracker()](auto) {
    cb_called = true;
    EXPECT_EQ(copy_tracker.times_copied, 1);
    EXPECT_FALSE(copy_tracker.dtor_called);
    return true;
});


int_stream.impl()->put_value(7);
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeTest, FilterLvalue) {
{
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker=CopyTracker()](auto) {
        cb_called = true;
        EXPECT_EQ(copy_tracker.times_copied, 1);
        EXPECT_FALSE(copy_tracker.dtor_called);
        return true;
    };
    int_stream.filter(lvalue_lambda);
}

int_stream.impl()->put_value(7);
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeTest, UnwrapOrRvalue) {

    /// Pass an rvalue reference lambda function, should get copied
    int_stream.unwrap_or([this, copy_tracker=CopyTracker()](auto) {
        cb_called = true;
        EXPECT_EQ(copy_tracker.times_copied, 1);
        EXPECT_FALSE(copy_tracker.dtor_called);
    });


int_stream.impl()->put_error("error");
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeTest, UnwrapOrLvalue) {
{
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, copy_tracker=CopyTracker()](auto) {
        cb_called = true;
        EXPECT_EQ(copy_tracker.times_copied, 1);
        EXPECT_FALSE(copy_tracker.dtor_called);
        return true;
    };
    int_stream.unwrap_or(lvalue_lambda);
}

int_stream.impl()->put_error("error");
EXPECT_TRUE(cb_called);
}
