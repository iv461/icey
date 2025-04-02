/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved. 
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <unordered_set>

#include "rclcpp/rclcpp.hpp"

namespace icey::impl {
/// A set tracking all the allocated stream-impls. Defined here so that there is only one across all
/// translation units.
std::unordered_set<void *> g_impls;
}  // namespace icey::impl

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
