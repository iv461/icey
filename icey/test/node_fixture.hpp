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
      node_->icey().get_executor()->spin_once(10ms);
    }
  }
  void spin_all(icey::Duration timeout) { node_->icey().get_executor()->spin_all(timeout); }

  void spin_some() { node_->icey().get_executor()->spin_some(); }

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
    if (sender_->icey().get_executor()) {
      sender_->icey().get_executor()->remove_node(sender_);
      receiver_->icey().get_executor()->add_node(sender_->get_node_base_interface());
      /// Assing the context executor so that both nodes use the same executor in async/await mode
      sender_->icey().get_executor() = receiver_->icey().get_executor();
    }
  }

  static void spin(rclcpp::executors::SingleThreadedExecutor &executor, icey::Duration timeout) {
    auto start = icey::Clock::now();
    while ((icey::Clock::now() - start) < timeout) {
      executor.spin_once(10ms);
    }
  }

  void spin(icey::Duration timeout) { spin(*receiver_->icey().get_executor(), timeout); }

  std::shared_ptr<icey::Node> sender_{std::make_shared<icey::Node>("icey_test_sender_node")};
  std::shared_ptr<icey::Node> receiver_{std::make_shared<icey::Node>("icey_test_receiver_node")};
};
