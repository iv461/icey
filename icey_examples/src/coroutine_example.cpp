/// This example demonstrates the fundamentals of coroutines: 
#include <icey/icey.hpp>

icey::Promise<void> a_coroutine(icey::Context &ctx) {
    std::cout << "2. Before create_timer " << std::endl;
    auto timer = ctx.create_timer(1s, true);
    std::cout << "3. Before awaiting timer  " << std::endl;
    co_await timer;
    std::cout << "4. After awaiting timer  " << std::endl;
    co_return;
}

int main(int argc, char **argv) {
    auto node = icey::create_node("icey_coroutine_example");
    std::cout << "1. Before calling coroutine " << std::endl;
    a_coroutine(node->icey());
    std::cout << "5. After calling coroutine " << std::endl;
    icey::spin(node);
    std::cout << "6. After sync_wait " << std::endl;
}