#pragma once 

#include <icey/icey.hpp>
#include <iostream>
#include <gtest/gtest.h>

using namespace std::chrono_literals;

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
};