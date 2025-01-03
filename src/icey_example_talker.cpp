#include <icey/icey_ros2.hpp>

#include "std_msgs/msg/string.hpp"

using StringMsg = std_msgs::msg::String;

using namespace std::chrono_literals;
int main(int argc, char **argv) {

    auto my_string = icey::create_state<StringMsg>("my_string");

    size_t cnt{0};
    icey::create_timer(100ms, [&my_string, &cnt] () {
        StringMsg msg;
        msg.data = "hello_hello";
        msg.data += std::to_string(cnt);
        cnt ++;
        my_string->set(msg);
    });

    icey::spawn(argc, argv, "talker_node");

    return 0;           
}