#pragma once 

#include <icey/icey.hpp>
#include <iostream>
#include <gtest/gtest.h>

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

using namespace std::chrono_literals;

/// We want to test both node types but do not want to do a templated test because 
/// then everything becomes
///      this->icey()-> template declare_parameter<std::string>("param"); 
// 
enum class NodeType {
    RegularNode,
    LifecycleNode
};

class NodeTest : public testing::Test {
 protected:

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override {
     // Code here will be called immediately after the constructor (right
     // before each test).
     if(!icey::impl::g_impls.empty()) {
      fmt::print("Exptected no impls, but we have:\n");
      for(auto k : icey::impl::g_impls) std::cout << "0x" << std::hex << size_t(k) << "\n";
     }
     EXPECT_TRUE(icey::impl::g_impls.empty());
  }

  void TearDown() override {
     // Code here will be called immediately after each test (right
     // before the destructor).
     
  }

  void spin(icey::Duration timeout) {
   auto start = icey::Clock::now();
   while ((icey::Clock::now() - start) < timeout) {
     node_->icey().get_executor()->spin_once(10ms);
   }
  }

  std::shared_ptr<icey::Node> node_{std::make_shared<icey::Node>("icey_context_test_node")};
  //std::shared_ptr<icey::Node> node_{std::make_shared<icey::Node>("icey_context_test_node")};
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
            /// Assing the context executor
            sender_->icey().get_executor() = receiver_->icey().get_executor();
        }
     }
    
     static void spin(rclcpp::executors::SingleThreadedExecutor &executor, icey::Duration timeout) {
      auto start = icey::Clock::now();
      while ((icey::Clock::now() - start) < timeout) {
        executor.spin_once(10ms);
      }
     }

     void spin(icey::Duration timeout) {
        spin(*receiver_->icey().get_executor(), timeout);
     }

     std::shared_ptr<icey::Node> sender_{std::make_shared<icey::Node>("icey_test_sender_node")};
     std::shared_ptr<icey::Node> receiver_{std::make_shared<icey::Node>("icey_test_receiver_node")};
};

