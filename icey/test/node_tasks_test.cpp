#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <example_interfaces/action/fibonacci.hpp>
#include <icey/icey_async_await.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

using namespace std::chrono_literals;

namespace {

using ExampleService = std_srvs::srv::SetBool;
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = icey::rclcpp_action::ServerGoalHandle<Fibonacci>;

struct Harness {
  std::shared_ptr<rclcpp::Node> sender{std::make_shared<rclcpp::Node>("node_tasks_sender")};
  std::shared_ptr<rclcpp::Node> receiver{std::make_shared<rclcpp::Node>("node_tasks_receiver")};
  std::shared_ptr<icey::ContextAsyncAwait> sender_ctx{
      std::make_shared<icey::ContextAsyncAwait>(sender.get())};
  std::shared_ptr<icey::ContextAsyncAwait> receiver_ctx{
      std::make_shared<icey::ContextAsyncAwait>(receiver.get())};
  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 8};

  Harness() {
    exec.add_node(sender->get_node_base_interface());
    exec.add_node(receiver->get_node_base_interface());
  }

  ~Harness() {
    exec.cancel();
    exec.remove_node(sender->get_node_base_interface());
    exec.remove_node(receiver->get_node_base_interface());
  }
};

template <class Pred>
void spin_until(Harness &h, Pred pred, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline && !pred()) {
    h.exec.spin_some();
  }
}

void run_tf_stress() {
  Harness h;
  auto tf = h.receiver_ctx->create_transform_buffer();
  auto tf_pub = h.sender->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  std::atomic_int64_t stamp_ns{0};
  std::atomic_uint64_t dispatches{0};
  std::atomic_uint64_t hits{0};
  std::atomic_uint64_t timeout_scenarios{0};
  std::atomic_uint64_t unexpected_errors{0};
  std::atomic_bool done{false};

  auto pub_timer = h.sender->create_wall_timer(6ms, [&]() {
    static std::atomic_uint64_t tick{0};
    geometry_msgs::msg::TransformStamped t;
    auto ns = 1700000000000000000LL + int64_t((tick.fetch_add(1) + 1) * 1000000ULL);
    t.header.stamp = icey::rclcpp_from_chrono(icey::Time{std::chrono::nanoseconds(ns)});
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.rotation.w = 1.0;
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(t);
    tf_pub->publish(msg);
    stamp_ns.store(ns, std::memory_order_release);
  });
  spin_until(h, [&] { return stamp_ns.load(std::memory_order_acquire) != 0; }, 2s);

  const auto tf_loop = [&]() -> icey::Promise<void> {
    for (int i = 0; i < 300; ++i) {
      auto ns = stamp_ns.load(std::memory_order_acquire);
      const bool should_hit = (i % 2) == 0;
      auto res = co_await tf.lookup("map", should_hit ? "base_link" : "missing_frame",
                                    rclcpp::Time(ns), 40ms);
      if (res.has_value()) {
        hits.fetch_add(1, std::memory_order_relaxed);
      } else if (should_hit) {
        unexpected_errors.fetch_add(1, std::memory_order_relaxed);
      } else {
        timeout_scenarios.fetch_add(1, std::memory_order_relaxed);
      }
    }
    done.store(true, std::memory_order_release);
    co_return;
  };
  tf_loop().detach();

  std::array<std::shared_ptr<rclcpp::TimerBase>, 4> timers;
  for (auto &timer : timers) {
    timer = h.receiver->create_wall_timer(
        6ms, [&]() { dispatches.fetch_add(1, std::memory_order_relaxed); });
  }

  spin_until(h, [&] { return dispatches.load(std::memory_order_relaxed) >= 5000; }, 12s);
  spin_until(h, [&] { return done.load(std::memory_order_acquire); }, 12s);
  for (auto &timer : timers) timer->cancel();
  pub_timer->cancel();

  EXPECT_GE(dispatches.load(std::memory_order_relaxed), 5000U);
  EXPECT_TRUE(done.load(std::memory_order_acquire));
  EXPECT_GT(hits.load(std::memory_order_relaxed), 0U);
  EXPECT_GT(timeout_scenarios.load(std::memory_order_relaxed), 0U);
  EXPECT_EQ(unexpected_errors.load(std::memory_order_relaxed), 0U);
}

void run_service_stress() {
  Harness h;
  h.sender_ctx->create_service<ExampleService>(
      "stress_set_bool", [](std::shared_ptr<ExampleService::Request> req) {
        auto resp = std::make_shared<ExampleService::Response>();
        resp->success = req->data;
        return resp;
      });
  auto client = h.receiver_ctx->create_client<ExampleService>("stress_set_bool");
  std::atomic_uint64_t dispatches{0};
  std::atomic_uint64_t ok{0};
  std::atomic_uint64_t errors{0};
  std::atomic_bool done{false};

  const auto service_loop = [&]() -> icey::Promise<void> {
    for (int i = 0; i < 200; ++i) {
      auto req = std::make_shared<ExampleService::Request>();
      req->data = true;
      auto res = co_await client.call(req, 400ms);
      if (res.has_value() && res.value()->success)
        ok.fetch_add(1, std::memory_order_relaxed);
      else
        errors.fetch_add(1, std::memory_order_relaxed);
    }
    done.store(true, std::memory_order_release);
    co_return;
  };
  service_loop().detach();

  std::array<std::shared_ptr<rclcpp::TimerBase>, 4> timers;
  for (auto &timer : timers) {
    timer = h.receiver->create_wall_timer(
        5ms, [&]() { dispatches.fetch_add(1, std::memory_order_relaxed); });
  }

  spin_until(h, [&] { return dispatches.load(std::memory_order_relaxed) >= 5000; }, 12s);
  spin_until(h, [&] { return done.load(std::memory_order_acquire); }, 12s);
  for (auto &timer : timers) timer->cancel();

  EXPECT_GE(dispatches.load(std::memory_order_relaxed), 5000U);
  EXPECT_TRUE(done.load(std::memory_order_acquire));
  EXPECT_GT(ok.load(std::memory_order_relaxed) + errors.load(std::memory_order_relaxed), 0U);
}

void run_action_stress() {
  Harness h;
  h.sender_ctx->create_action_server<Fibonacci>(
      "stress_fib",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandleFibonacci>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [](std::shared_ptr<GoalHandleFibonacci> gh) {
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = {0, 1, 1, 2};
        gh->succeed(result);
      });
  auto client = h.receiver_ctx->create_action_client<Fibonacci>("stress_fib");
  std::atomic_uint64_t dispatches{0};
  std::atomic_uint64_t ok{0};
  std::atomic_uint64_t errors{0};
  std::atomic_bool done{false};

  const auto action_loop = [&]() -> icey::Promise<void> {
    for (int i = 0; i < 120; ++i) {
      Fibonacci::Goal goal;
      goal.order = 4;
      auto sent = co_await client.send_goal(goal, 400ms, [](auto, auto) {});
      if (!sent.has_value()) {
        errors.fetch_add(1, std::memory_order_relaxed);
      } else {
        auto res = co_await sent.value().result(400ms);
        if (res.has_value() && res.value().code == rclcpp_action::ResultCode::SUCCEEDED)
          ok.fetch_add(1, std::memory_order_relaxed);
        else
          errors.fetch_add(1, std::memory_order_relaxed);
      }
    }
    done.store(true, std::memory_order_release);
    co_return;
  };
  action_loop().detach();

  std::array<std::shared_ptr<rclcpp::TimerBase>, 4> timers;
  for (auto &timer : timers) {
    timer = h.receiver->create_wall_timer(
        5ms, [&]() { dispatches.fetch_add(1, std::memory_order_relaxed); });
  }

  spin_until(h, [&] { return dispatches.load(std::memory_order_relaxed) >= 5000; }, 12s);
  spin_until(h, [&] { return done.load(std::memory_order_acquire); }, 15s);
  for (auto &timer : timers) timer->cancel();

  EXPECT_GE(dispatches.load(std::memory_order_relaxed), 5000U);
  EXPECT_TRUE(done.load(std::memory_order_acquire));
  EXPECT_GT(ok.load(std::memory_order_relaxed) + errors.load(std::memory_order_relaxed), 0U);
}

void run_race_repro_async_internals() {
  Harness h;
  icey::NodeBase node_base(h.receiver.get());
  auto tf = h.receiver_ctx->create_transform_buffer();
  auto tf_pub = h.sender->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  std::atomic_uint64_t ops{0};
  std::array<std::atomic_uint64_t, 12> seq{};
  std::mutex worker_ids_mutex;
  std::unordered_set<std::thread::id> worker_ids;
  for (auto &x : seq) x.store(0, std::memory_order_relaxed);

  auto pub_timer = h.sender->create_wall_timer(2ms, [&]() {
    static std::atomic_uint64_t tick{0};
    geometry_msgs::msg::TransformStamped t;
    auto ns = 1700000000000000000LL + int64_t((tick.fetch_add(1) + 1) * 1000000ULL);
    t.header.stamp = icey::rclcpp_from_chrono(icey::Time{std::chrono::nanoseconds(ns)});
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.rotation.w = 1.0;
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(t);
    tf_pub->publish(msg);
  });

  std::vector<rclcpp::CallbackGroup::SharedPtr> groups;
  std::vector<std::shared_ptr<rclcpp::TimerBase>> timers;
  groups.reserve(seq.size());
  timers.reserve(seq.size());
  for (size_t t = 0; t < seq.size(); ++t) {
    auto group = h.receiver->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    groups.push_back(group);
    timers.push_back(h.receiver->create_wall_timer(
        1ms,
        [&, t]() {
          {
            std::lock_guard<std::mutex> lock(worker_ids_mutex);
            worker_ids.insert(std::this_thread::get_id());
          }
          const auto i = seq[t].fetch_add(1, std::memory_order_relaxed) + 1;
          const auto id = (uint64_t(t) << 32U) | i;
          node_base.add_task_for(id, 1ms, []() {});
          if ((i % 2U) == 0U) node_base.cancel_task_for(id);
          auto hnd = tf.lookup("map", "base_link", icey::Clock::now(), 3ms, [](const auto &) {},
                               [](const auto &) {});
          if ((i % 3U) == 0U) tf.cancel_request(hnd);
          ops.fetch_add(1, std::memory_order_relaxed);
        },
        group));
  }

  std::thread spin_thread([&]() { h.exec.spin(); });
  const auto deadline = std::chrono::steady_clock::now() + 4s;
  while (std::chrono::steady_clock::now() < deadline &&
         ops.load(std::memory_order_relaxed) < 30000U) {
    std::this_thread::sleep_for(2ms);
  }

  for (auto &timer : timers) timer->cancel();
  pub_timer->cancel();
  h.exec.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  EXPECT_GE(ops.load(std::memory_order_relaxed), 10000U);
  EXPECT_GT(worker_ids.size(), 1U);
}

}  // namespace

TEST(NodeTasksThreadSafety, TfWithFourTimers) { run_tf_stress(); }

TEST(NodeTasksThreadSafety, ServiceWithFourTimers) { run_service_stress(); }

TEST(NodeTasksThreadSafety, ActionWithFourTimers) { run_action_stress(); }

TEST(NodeTasksThreadSafety, TSanRaceReproAsyncInternals) { run_race_repro_async_internals(); }
