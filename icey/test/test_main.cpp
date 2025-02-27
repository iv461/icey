#include "rclcpp/rclcpp.hpp"
#include <gtest/gtest.h>
#include <unordered_set>

namespace icey::impl { 
  /// A set tracking all the allocated stream-impls. Defined here so that there is only one across all translation units.
  std::unordered_set<void*> g_impls;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}
