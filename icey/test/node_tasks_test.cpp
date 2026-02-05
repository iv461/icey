/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#include "node_fixture.hpp"

using namespace std::chrono_literals;

struct NodeTasksTest : NodeTest {};

TEST_F(NodeTasksTest, TaskFiresAndCleans) {
  icey::ContextAsyncAwait ctx(node_.get());
  EXPECT_EQ(ctx.oneoff_active_task_count(), 0u);

  bool fired = false;
  ctx.add_task_for(1, 50ms, [&] { fired = true; });
  spin(100ms);
  EXPECT_TRUE(fired);

  // After firing, the timer moves into deferred-cancelled set; schedule a dummy to flush cleanup
  ctx.add_task_for(2, 1ms, [] {});
  spin(10ms);

  // Let dummy fire and cleanup deferred set
  spin(20ms);
  EXPECT_EQ(ctx.oneoff_active_task_count(), 0u);
}

TEST_F(NodeTasksTest, TaskCancelPreventsCallbackAndCleans) {
  icey::ContextAsyncAwait ctx(node_.get());
  bool fired = false;
  ctx.add_task_for(3, 100ms, [&] { fired = true; });
  EXPECT_EQ(ctx.oneoff_active_task_count(), 1u);

  EXPECT_TRUE(ctx.cancel_task_for(3));
  // schedule another to flush deferred cleanup
  ctx.add_task_for(4, 1ms, [] {});
  spin(10ms);

  EXPECT_FALSE(fired);
  // Let cleanup of dummy happen as well
  spin(10ms);
  EXPECT_EQ(ctx.oneoff_active_task_count(), 0u);
}

TEST_F(NodeTasksTest, SharedPtrKeyCancelAndFire) {
  icey::ContextAsyncAwait ctx(node_.get());
  auto key = std::make_shared<int>(42);
  bool fired = false;
  ctx.add_task_for(key, 80ms, [&] { fired = true; });
  EXPECT_EQ(ctx.oneoff_active_task_count(), 1u);

  // Cancel should succeed and prevent callback
  EXPECT_TRUE(ctx.cancel_task_for(key));
  spin(100ms);
  EXPECT_FALSE(fired);

  // Now schedule another with same key and let it fire
  ctx.add_task_for(key, 20ms, [&] { fired = true; });
  spin(50ms);
  EXPECT_TRUE(fired);

  // Flush deferred cleanup
  ctx.add_task_for(5, 1ms, [] {});
  spin(10ms);
  spin(10ms);
  EXPECT_EQ(ctx.oneoff_active_task_count(), 0u);
}
