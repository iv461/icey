#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <example_interfaces/action/fibonacci.hpp>
#include <icey/icey_async_await.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>

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
  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 2};
  std::thread spin_thread;

  Harness() {
    exec.add_node(sender->get_node_base_interface());
    exec.add_node(receiver->get_node_base_interface());
    spin_thread = std::thread([this]() { exec.spin(); });
  }

  ~Harness() {
    exec.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    exec.remove_node(sender->get_node_base_interface());
    exec.remove_node(receiver->get_node_base_interface());
  }
};

void wait_for_dispatches(const std::atomic_uint64_t &dispatches, uint64_t target,
                         std::chrono::seconds max_time) {
  const auto deadline = std::chrono::steady_clock::now() + max_time;
  while (std::chrono::steady_clock::now() < deadline &&
         dispatches.load(std::memory_order_relaxed) < target) {
    std::this_thread::sleep_for(10ms);
  }
}

void run_tf_stress() {
  Harness h;
  auto tf = std::make_shared<icey::TransformBuffer>(h.receiver_ctx->create_transform_buffer());
  auto tf_pub = h.sender->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  auto stamp_ns = std::make_shared<std::atomic_int64_t>(0);
  auto dispatches = std::make_shared<std::atomic_uint64_t>(0);
  auto tf_hits = std::make_shared<std::atomic_uint64_t>(0);
  auto errors = std::make_shared<std::atomic_uint64_t>(0);
  auto pub_timer = h.sender->create_wall_timer(6ms, [stamp_ns, tf_pub]() {
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
    stamp_ns->store(ns, std::memory_order_release);
  });
  std::array<std::shared_ptr<rclcpp::TimerBase>, 4> timers;
  for (size_t i = 0; i < timers.size(); ++i) {
    timers[i] = h.receiver->create_wall_timer(6ms, [=]() {
      dispatches->fetch_add(1, std::memory_order_relaxed);
      auto ns = stamp_ns->load(std::memory_order_acquire);
      if (ns == 0) return;
      auto handle = tf->lookup(
          "map", "base_link", icey::Time{std::chrono::nanoseconds(ns)}, 200ms,
          [tf_hits](const auto &) { tf_hits->fetch_add(1, std::memory_order_relaxed); },
          [errors](const auto &) { errors->fetch_add(1, std::memory_order_relaxed); });
      if ((dispatches->load(std::memory_order_relaxed) % 3U) == 0U) tf->cancel_request(handle);
    });
  }
  wait_for_dispatches(*dispatches, 5000, 10s);
  for (auto &timer : timers) timer->cancel();
  pub_timer->cancel();
  std::this_thread::sleep_for(200ms);
  EXPECT_GE(dispatches->load(std::memory_order_relaxed), 5000U);
  EXPECT_GT(tf_hits->load(std::memory_order_relaxed), 0U);
  EXPECT_EQ(errors->load(std::memory_order_relaxed), 0U);
}

void run_service_stress() {
  Harness h;
  h.sender_ctx->create_service<ExampleService>("stress_set_bool",
                                               [](std::shared_ptr<ExampleService::Request> req) {
                                                 auto resp = std::make_shared<ExampleService::Response>();
                                                 resp->success = req->data;
                                                 return resp;
                                               });
  auto client = std::make_shared<icey::ServiceClient<ExampleService>>(
      h.receiver_ctx->create_client<ExampleService>("stress_set_bool"));
  auto dispatches = std::make_shared<std::atomic_uint64_t>(0);
  auto ok = std::make_shared<std::atomic_uint64_t>(0);
  auto errors = std::make_shared<std::atomic_uint64_t>(0);
  auto done = std::make_shared<std::atomic_bool>(false);
  [=]() -> icey::Promise<void> {
    for (int i = 0; i < 200; ++i) {
      try {
        auto req = std::make_shared<ExampleService::Request>();
        req->data = true;
        auto res = co_await client->call(req, 400ms);
        if (res.has_value() && res.value()->success)
          ok->fetch_add(1, std::memory_order_relaxed);
        else
          errors->fetch_add(1, std::memory_order_relaxed);
      } catch (...) {
        errors->fetch_add(1, std::memory_order_relaxed);
      }
    }
    done->store(true, std::memory_order_release);
    co_return;
  }()
      .detach();
  std::array<std::shared_ptr<rclcpp::TimerBase>, 4> timers;
  for (size_t i = 0; i < timers.size(); ++i) {
    timers[i] = h.receiver->create_wall_timer(5ms, [=]() {
      dispatches->fetch_add(1, std::memory_order_relaxed);
    });
  }
  wait_for_dispatches(*dispatches, 5000, 12s);
  const auto done_deadline = std::chrono::steady_clock::now() + 12s;
  while (std::chrono::steady_clock::now() < done_deadline &&
         !done->load(std::memory_order_acquire))
    std::this_thread::sleep_for(10ms);
  for (auto &timer : timers) timer->cancel();
  EXPECT_GE(dispatches->load(std::memory_order_relaxed), 5000U);
  EXPECT_TRUE(done->load(std::memory_order_acquire));
  EXPECT_GT(ok->load(std::memory_order_relaxed) + errors->load(std::memory_order_relaxed), 0U);
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
  auto client = std::make_shared<icey::ActionClient<Fibonacci>>(
      h.receiver_ctx->create_action_client<Fibonacci>("stress_fib"));
  auto dispatches = std::make_shared<std::atomic_uint64_t>(0);
  auto ok = std::make_shared<std::atomic_uint64_t>(0);
  auto errors = std::make_shared<std::atomic_uint64_t>(0);
  auto done = std::make_shared<std::atomic_bool>(false);
  [=]() -> icey::Promise<void> {
    for (int i = 0; i < 120; ++i) {
      try {
        Fibonacci::Goal goal;
        goal.order = 4;
        auto sent = co_await client->send_goal(goal, 400ms, [](auto, auto) {});
        if (!sent.has_value()) {
          errors->fetch_add(1, std::memory_order_relaxed);
        } else {
          auto res = co_await sent.value().result(400ms);
          if (res.has_value() && res.value().code == rclcpp_action::ResultCode::SUCCEEDED)
            ok->fetch_add(1, std::memory_order_relaxed);
          else
            errors->fetch_add(1, std::memory_order_relaxed);
        }
      } catch (...) {
        errors->fetch_add(1, std::memory_order_relaxed);
      }
    }
    done->store(true, std::memory_order_release);
    co_return;
  }()
      .detach();
  std::array<std::shared_ptr<rclcpp::TimerBase>, 4> timers;
  for (size_t i = 0; i < timers.size(); ++i) {
    timers[i] = h.receiver->create_wall_timer(5ms, [=]() {
      dispatches->fetch_add(1, std::memory_order_relaxed);
    });
  }
  wait_for_dispatches(*dispatches, 5000, 12s);
  const auto done_deadline = std::chrono::steady_clock::now() + 15s;
  while (std::chrono::steady_clock::now() < done_deadline &&
         !done->load(std::memory_order_acquire))
    std::this_thread::sleep_for(10ms);
  for (auto &timer : timers) timer->cancel();
  EXPECT_GE(dispatches->load(std::memory_order_relaxed), 5000U);
  EXPECT_TRUE(done->load(std::memory_order_acquire));
  EXPECT_GT(ok->load(std::memory_order_relaxed) + errors->load(std::memory_order_relaxed), 0U);
}

}  // namespace

TEST(NodeTasksThreadSafety, TfWithFourTimers) { run_tf_stress(); }

TEST(NodeTasksThreadSafety, ServiceWithFourTimers) { run_service_stress(); }

TEST(NodeTasksThreadSafety, ActionWithFourTimers) { run_action_stress(); }
