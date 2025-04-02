/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved. 
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#pragma once

#define ICEY_DEBUG_TRACK_STREAM_ALLOCATIONS  /// We also test whether all impl Streams get destroyed
                                             /// in every unit-test, for this we enable the
                                             /// allocation tracking
//#define ICEY_DEBUG_PRINT_STREAM_ALLOCATIONS /// optionally enable to see the allocations

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include <icey/icey.hpp>
#include <iostream>

using namespace std::chrono_literals;

/// We want to test both node types but do not want to do a templated test because
/// then everything would become:
///      this->icey()-> template declare_parameter<std::string>("param");
/// instead of just:
///      this->icey()->declare_parameter<std::string>("param");
enum class NodeType { RegularNode, LifecycleNode };

/// Note what when you use this fixture, you are not allowed to hold Streams in it.
/// This is because when this fixture is tore down, it destroys the node and checks if there are
/// still some stream-impls allocated, i.e. it checks for memory leaks.
class NodeTest : public testing::Test {
protected:
  void SetUp() override {
    // Code here will be called immediately after the constructor (right
    // before each test).
    executor_.add_node(node_->get_node_base_interface());
  }

  void TearDown() override {
    // Code here will be called immediately after each test (right
    // before the destructor).
    node_.reset();
    if (!icey::impl::g_impls.empty()) {
      fmt::print("Stream impls are still allocated:\n");
      for (auto k : icey::impl::g_impls) std::cout << "0x" << std::hex << size_t(k) << "\n";
    }
    EXPECT_TRUE(icey::impl::g_impls.empty())
        << "Some stream impls are still allocated after the node was destroyed, you likely have a "
           "circular reference. Do not capture Streams in lambdas by value, but capture only the "
           "impl, i.e. stream->impl() by value.";
  }

  void spin(icey::Duration timeout) {
    auto start = icey::Clock::now();
    while ((icey::Clock::now() - start) < timeout) {
      executor_.spin_once(10ms);
    }
  }
  void spin_all(icey::Duration timeout) { executor_.spin_all(timeout); }
  void spin_some() { executor_.spin_some(); }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<icey::Node> node_{std::make_shared<icey::Node>("icey_context_test_node")};
  // std::shared_ptr<icey::Node> node_{std::make_shared<icey::Node>("icey_context_test_node")};
};

class TwoNodesFixture : public testing::Test {
protected:
  void SetUp() override {
    // Code here will be called immediately after the constructor (right
    // before each test).
    ASSERT_TRUE(icey::impl::g_impls.empty());
  }

  TwoNodesFixture() {
    /// Put both nodes in the same executor so that if it spins, both nodes get what they want
    executor_.add_node(sender_->get_node_base_interface());
    executor_.add_node(receiver_->get_node_base_interface());
  }

  static void spin(rclcpp::executors::SingleThreadedExecutor &executor, icey::Duration timeout) {
    auto start = icey::Clock::now();
    while ((icey::Clock::now() - start) < timeout) {
      executor.spin_once(10ms);
    }
  }

  void spin(icey::Duration timeout) { spin(executor_, timeout); }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<icey::Node> sender_{std::make_shared<icey::Node>("icey_test_sender_node")};
  std::shared_ptr<icey::Node> receiver_{std::make_shared<icey::Node>("icey_test_receiver_node")};
};
