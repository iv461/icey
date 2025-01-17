/// This test tests whether the Promises behave like promises as specified in JavaScript, especially 
/// regarding catch-handlers executed correctly (fall-through behavior) and whether finally gets always executed.

#include <icey/icey_ros2.hpp>
#include <iostream>

int main() {
    auto ctx = std::make_shared<icey::Context>();

    using Promise = icey::Observable<std::string>;
    auto my_promise = ctx->create_observable<Promise>();

    my_promise
        ->then([](std::string res) {
            std::cout << "Got " << res << std::endl;
            return "foo";
        })
        ->then([](std::string res) {
            std::cout << "Got " << res << std::endl;
            return "bar";
        });

    my_promise->resolve("heloo");

    using ResolveValue = std::string;
    using ErrorValue = int;
    auto my_promise2 = ctx->create_observable< icey::Observable<ResolveValue, ErrorValue> >();

my_promise2->then([](ResolveValue res) {
            std::cout << "Got " << res << std::endl;
            return std::string("foo");
        })
        ->then([](ResolveValue res) {
            std::cout << "Got " << res << std::endl;
            return std::string("foo");
        })->except([](ErrorValue retcode) {
            std::cout << "Got error " << retcode << std::endl;
            return 7.f;
        })
        ->then([](float x) {
            std::cout << "Got float after error:  " << x << std::endl;
        });

    my_promise2->resolve("resolution");
    my_promise2->reject(-3);
}