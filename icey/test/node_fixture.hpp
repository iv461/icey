#pragma once 

#include <icey/icey.hpp>
#include <iostream>
#include <gtest/gtest.h>

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
  
  NodeTest() {
     // You can do set-up work for each test here.
  }

  ~NodeTest() override {
     // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override {
     // Code here will be called immediately after the constructor (right
     // before each test).
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
    TwoNodesFixture() {
        if (sender_->icey().get_executor()) {
            sender_->icey().get_executor()->remove_node(sender_);
            sender_->icey().get_executor().reset();
        }
        if (receiver_->icey().get_executor()) {
            receiver_->icey().get_executor()->remove_node(receiver_);
            receiver_->icey().get_executor().reset();
        }
        executor_.add_node(sender_);
        executor_.add_node(receiver_);
     }
    
     static void spin(rclcpp::executors::SingleThreadedExecutor &executor, icey::Duration timeout) {
      auto start = icey::Clock::now();
      while ((icey::Clock::now() - start) < timeout) {
        executor.spin_once(10ms);
      }
     }

     void spin(icey::Duration timeout) {
        spin(executor_, timeout);
     }

     rclcpp::executors::SingleThreadedExecutor executor_;
     std::shared_ptr<icey::Node> sender_{std::make_shared<icey::Node>("icey_test_sender_node")};
     std::shared_ptr<icey::Node> receiver_{std::make_shared<icey::Node>("icey_test_receiver_node")};
};

