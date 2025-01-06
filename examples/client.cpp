#include <icey/icey_ros2.hpp>

#include "example_interfaces/srv/add_two_ints.hpp"

/* TODO this does not make sense without async/await. We first need the node
int main(int argc, char **argv) {

    auto client = icey::create_client("add_two_ints");

    
    //icey::create_service<std_msgs::msg::Float32>("add_two_ints", cb);

    icey::spawn(argc, argv, "add_two_ints_server"); /// Create and start node
}
*/