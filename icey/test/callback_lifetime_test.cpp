/// Tests whether all functions accepting callbacks copy the callback so that it lives for as long as the stream lives. 
/// The callbacks should be copied regardless of whether they are lvalues or rvalues, so both cases are tested.
/// We also assert that the callback is copied exactly once and not unnecessarily many times.
#include "node_fixture.hpp"

/// A class to track the copies, moves and destructions
struct Foo {
    Foo() {
        std::cout << "Foo was constructed" << std::endl;
    }
    Foo(const Foo &other): dtor_called(other.dtor_called) {
        std::cout << "Foo was copied" << std::endl;
        times_copied = other.times_copied + 1;
    }
    Foo(Foo &&other): dtor_called(other.dtor_called) {
        std::cout << "Foo was was move-constructed" << std::endl;
    }
    Foo &operator=(const Foo &other) {
        std::cout << "Foo was was copy-assigned" << std::endl;
        this->dtor_called = other.dtor_called;
        times_copied = other.times_copied + 1;
        return *this;
    }
    Foo &operator=(Foo &&other) {
        std::cout << "Foo was was move-assigned" << std::endl;
        this->dtor_called = other.dtor_called;
        return *this;
    }
    
    ~Foo() {
        dtor_called = true;
    }
    std::size_t times_copied{0};
    /// Note that the memory does not get overwritten, therefore we can actually check whether something was destructed by reading this out.
    bool dtor_called{false};
};

class CallbackLifetimeFixture : public NodeTest {
protected:
icey::Stream<int, std::string> int_stream{node_->icey().create_stream<icey::Stream<int, std::string>>()};
bool cb_called{false};
};
TEST_F(CallbackLifetimeFixture, ThenRvalue) {

/// Pass an rvalue reference lambda function, should get copied
int_stream.then([this, foo=Foo()](auto) {
    cb_called = true;
    EXPECT_EQ(foo.times_copied, 1);
    EXPECT_FALSE(foo.dtor_called);
});

int_stream.impl()->put_value(5);
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeFixture, ThenLvalue) {
{
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, foo=Foo()](auto) {
        cb_called = true;
        EXPECT_EQ(foo.times_copied, 1);
        EXPECT_FALSE(foo.dtor_called);
    };
    int_stream.then(lvalue_lambda);
}

int_stream.impl()->put_value(6);
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeFixture, ExceptRvalue) {

/// Pass an rvalue reference lambda function, should get copied
int_stream.except([this, foo=Foo()](auto) {
    cb_called = true;
    EXPECT_EQ(foo.times_copied, 1);
    EXPECT_FALSE(foo.dtor_called);
});

int_stream.impl()->put_error("error");
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeFixture, ExceptLvalue) {
{
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, foo=Foo()](auto) {
        cb_called = true;
        EXPECT_EQ(foo.times_copied, 1);
        EXPECT_FALSE(foo.dtor_called);
    };
    int_stream.except(lvalue_lambda);
}

int_stream.impl()->put_error("error");
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeFixture, FilterRvalue) {

/// Pass an rvalue reference lambda function, should get copied
int_stream.filter([this, foo=Foo()](auto) {
    cb_called = true;
    EXPECT_EQ(foo.times_copied, 1);
    EXPECT_FALSE(foo.dtor_called);
    return true;
});


int_stream.impl()->put_value(7);
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeFixture, FilterLvalue) {
{
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, foo=Foo()](auto) {
        cb_called = true;
        EXPECT_EQ(foo.times_copied, 1);
        EXPECT_FALSE(foo.dtor_called);
        return true;
    };
    int_stream.filter(lvalue_lambda);
}

int_stream.impl()->put_value(7);
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeFixture, UnwrapOrRvalue) {

    /// Pass an rvalue reference lambda function, should get copied
    int_stream.unwrap_or([this, foo=Foo()](auto) {
        cb_called = true;
        EXPECT_EQ(foo.times_copied, 1);
        EXPECT_FALSE(foo.dtor_called);
    });


int_stream.impl()->put_error("error");
EXPECT_TRUE(cb_called);
}

TEST_F(CallbackLifetimeFixture, UnwrapOrLvalue) {
{
    /// Pass an lvalue reference lambda function, should get copied
    auto lvalue_lambda = [this, foo=Foo()](auto) {
        cb_called = true;
        EXPECT_EQ(foo.times_copied, 1);
        EXPECT_FALSE(foo.dtor_called);
        return true;
    };
    int_stream.unwrap_or(lvalue_lambda);
}

int_stream.impl()->put_error("error");
EXPECT_TRUE(cb_called);
}
