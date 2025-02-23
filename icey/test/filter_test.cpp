#include "node_fixture.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

using ExampleService = std_srvs::srv::SetBool;

TEST_F(NodeTest, FilterTest) {
    auto timer = node_->icey().create_timer(50ms);
    
   size_t timer_ticked{0};
   EXPECT_EQ(timer_ticked, 0);

   timer
     .filter([&](size_t ticks) { return !(ticks % 3); })
     .then([&](size_t ticks) {
         if(ticks > 10) {
             timer.impl()->timer->cancel();
             return;
        }
        ASSERT_FALSE(ticks % 3);
        timer_ticked++;
    });   

    spin(1100ms);
    /// 0, 3, 6, 9, four times
    EXPECT_EQ(timer_ticked, 4);  
}

TEST_F(NodeTest, CallbackLifetime) {
    auto int_stream = node_->icey().create_stream<icey::Stream<int>>();
    
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

    bool cb_called{false};
    {
        std::cout << "registering then .. " << std::endl;
        /// Pass an rvalue reference lambda function, should get copied
        int_stream.then([&cb_called, foo=Foo()](auto x) {
            cb_called = true;
            EXPECT_EQ(foo.times_copied, 1);
            EXPECT_FALSE(foo.dtor_called);
        });
        std::cout << "Registered . " << std::endl;
    }

    int_stream.impl()->put_value(5);
    EXPECT_TRUE(cb_called);

    
    cb_called = false;
    {
        /// Pass an lvalue reference lambda function, should get copied
        auto lvalue_lambda = [&cb_called, foo=Foo()](auto x) {
            cb_called = true;
            EXPECT_EQ(foo.times_copied, 1);
            EXPECT_FALSE(foo.dtor_called);
        };
        int_stream.then(lvalue_lambda);
    }

    int_stream.impl()->put_value(6);
    EXPECT_TRUE(cb_called);
}
