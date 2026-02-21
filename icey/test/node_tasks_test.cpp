#include <gtest/gtest.h>

#include <chrono>
#include <example_interfaces/action/fibonacci.hpp>
#include <icey/icey_async_await.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
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
[[maybe_unused]]
void spin_until(Harness &h, Pred pred, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline && !pred()) {
    h.exec.spin_some();
    std::this_thread::sleep_for(1ms);
  }
}

void spin_executor_for(Harness &h, std::chrono::seconds duration) {
  std::thread spin_thread([&]() { h.exec.spin(); });
  std::this_thread::sleep_for(duration);
  h.exec.cancel();
  if (spin_thread.joinable()) spin_thread.join();
}

void run_tf_race_repro() {
  Harness h;
  auto tf_pub = h.sender->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

  auto pub_timer = h.sender->create_wall_timer(10ms, [&]() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = icey::rclcpp_from_chrono(icey::Clock::now());
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.rotation.w = 1.0;
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(t);
    tf_pub->publish(msg);
  });

  auto tf = h.receiver_ctx->create_transform_buffer();
  auto cb_group{h.receiver->create_callback_group(rclcpp::CallbackGroupType::Reentrant)};
  for (size_t t = 0; t < 100; ++t) {
    h.receiver_ctx->create_timer_async(
        50ms,
        [&](std::size_t) -> icey::Promise<void> {
          (void)co_await tf.lookup("map", "base_link", icey::Clock::now(), 3ms);
        },
        cb_group);
  }
  (void)pub_timer;
  spin_executor_for(h, 10s);
}

void run_service_race_repro() {
  Harness h;
  h.sender_ctx->create_service<ExampleService>(
      "race_set_bool", [](std::shared_ptr<ExampleService::Request> req) {
        auto resp = std::make_shared<ExampleService::Response>();
        resp->success = req->data;
        return resp;
      });
  auto client = h.receiver_ctx->create_client<ExampleService>("race_set_bool");
  auto cb_group{h.receiver->create_callback_group(rclcpp::CallbackGroupType::Reentrant)};
  for (size_t t = 0; t < 100; ++t) {
    h.receiver_ctx->create_timer_async(
        50ms,
        [&](std::size_t) -> icey::Promise<void> {
          auto req = std::make_shared<ExampleService::Request>();
          req->data = true;
          (void)co_await client.call(req, 3ms);
        },
        cb_group);
  }
  spin_executor_for(h, 10s);
}

void run_action_race_repro() {
  Harness h;
  h.sender_ctx->create_action_server<Fibonacci>(
      "race_fib",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandleFibonacci>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [](std::shared_ptr<GoalHandleFibonacci> gh) {
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = {0, 1, 1, 2};
        gh->succeed(result);
      });
  auto client = h.receiver_ctx->create_action_client<Fibonacci>("race_fib");
  auto cb_group{h.receiver->create_callback_group(rclcpp::CallbackGroupType::Reentrant)};
  for (size_t t = 0; t < 100; ++t) {
    h.receiver_ctx->create_timer_async(
        50ms,
        [&](std::size_t) -> icey::Promise<void> {
          Fibonacci::Goal goal;
          goal.order = 4;
          auto sent = co_await client.send_goal(goal, 3ms, [](auto, auto) {});
          if (sent.has_value()) (void)co_await sent.value().result(3ms);
        },
        cb_group);
  }
  spin_executor_for(h, 10s);
}

}  // namespace

TEST(ThreadSafety, TFRaces) { run_tf_race_repro(); }
TEST(ThreadSafety, Services) { run_service_race_repro(); }
TEST(ThreadSafety, Actions) { run_action_race_repro(); }
