/// This test tests whether the Promises behave like promises as specified in JavaScript, especially 
/// regarding catch-handlers executed correctly (fall-through behavior) and whether finally gets always executed.

#include <icey/icey_ros2.hpp>
#include <iostream>

int main() {
    auto ctx = std::make_shared<icey::Context>();

    using Promise = icey::Observable<std::string>;
    auto my_promise = ctx.create_observable<Promise>();

    my_promise
        ->then([](std::string res1) {
            return "foo";
        })
        ->then([](std::string res) {
            return "bar";
        });

    my_promise->_set_and_notify("heloo");
}