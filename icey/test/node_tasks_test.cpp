#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <icey/icey_async_await.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace {

void spin_for(rclcpp::executors::MultiThreadedExecutor &exec, std::chrono::milliseconds duration) {
  auto until = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < until) {
    exec.spin_some();
    std::this_thread::sleep_for(1ms);
  }
}

void stress_node_oneoff_tasks(icey::NodeBase &node_base, int workers, int iterations) {
  std::vector<std::thread> threads;
  threads.reserve(workers);
  std::atomic_int callbacks{0};
  for (int t = 0; t < workers; ++t) {
    threads.emplace_back([&, t] {
      for (int i = 0; i < iterations; ++i) {
        const uint64_t id = (uint64_t(t) << 32U) | uint64_t(i);
        node_base.add_task_for(id, 1ms, [&callbacks]() { callbacks.fetch_add(1); });
        if ((i % 2) == 0) {
          node_base.cancel_task_for(id);
        }
      }
    });
  }
  for (auto &th : threads) th.join();
}

void stress_tf_lookup_cancel(icey::TransformBuffer &tf, int workers, int iterations) {
  std::vector<std::thread> threads;
  threads.reserve(workers);
  for (int t = 0; t < workers; ++t) {
    threads.emplace_back([&, t] {
      (void)t;
      for (int i = 0; i < iterations; ++i) {
        auto handle = tf.lookup("map", "base_link", icey::Clock::now(), 1ms,
                                [](const auto &) {}, [](const auto &) {});
        if ((i % 2) == 0) {
          tf.cancel_request(handle);
        }
      }
    });
  }
  for (auto &th : threads) th.join();
}

}  // namespace

TEST(NodeTasksThreadSafety, OneoffTaskMapsStress) {
  auto node = std::make_shared<rclcpp::Node>("node_tasks_mt_stress");
  icey::NodeBase node_base(node.get());

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node->get_node_base_interface());

  stress_node_oneoff_tasks(node_base, 8, 600);
  spin_for(exec, 80ms);
  exec.remove_node(node->get_node_base_interface());

  SUCCEED();
}

TEST(NodeTasksThreadSafety, TransformBufferLookupCancelStress) {
  auto node = std::make_shared<rclcpp::Node>("tf_lookup_cancel_mt_stress");
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());
  auto tf = ctx->create_transform_buffer();

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node->get_node_base_interface());

  stress_tf_lookup_cancel(tf, 8, 400);
  spin_for(exec, 80ms);
  exec.remove_node(node->get_node_base_interface());

  SUCCEED();
}

TEST(NodeTasksThreadSafety, TSanThreadSafetyAssertion) {
#if defined(__SANITIZE_THREAD__)
  auto node = std::make_shared<rclcpp::Node>("tsan_mt_safety_assert");
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());
  auto tf = ctx->create_transform_buffer();
  icey::NodeBase node_base(node.get());

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node->get_node_base_interface());

  stress_node_oneoff_tasks(node_base, 10, 1000);
  stress_tf_lookup_cancel(tf, 10, 600);
  spin_for(exec, 120ms);

  exec.remove_node(node->get_node_base_interface());
  SUCCEED() << "TSAN would fail this test if races are present.";
#else
  GTEST_SKIP() << "Build with TSAN to enable this assertion test.";
#endif
}
