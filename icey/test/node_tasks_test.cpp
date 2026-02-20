#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <example_interfaces/action/fibonacci.hpp>
#include <icey/icey_async_await.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

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
  icey::NodeBase receiver_base{receiver.get()};
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

void spin_for(Harness &h, std::chrono::milliseconds duration) {
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) h.exec.spin_some();
}

TEST(NodeTasksThreadSafety, TfWithFourTimers) {
  Harness h;
  auto tf = h.receiver_ctx->create_transform_buffer();
  auto tf_pub = h.sender->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  int64_t stamp_ns = 0;
  size_t mode = 0;

  auto pub_timer = h.sender->create_wall_timer(2ms, [&]() {
    geometry_msgs::msg::TransformStamped t;
    stamp_ns += 1000000;
    t.header.stamp = rclcpp::Time(1700000000000000000LL + stamp_ns);
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.rotation.w = 1.0;
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(t);
    tf_pub->publish(msg);
  });

  std::vector<rclcpp::CallbackGroup::SharedPtr> groups;
  std::array<std::shared_ptr<rclcpp::TimerBase>, 4> timers;
  for (size_t i = 0; i < timers.size(); ++i) {
    auto group = h.receiver->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    groups.push_back(group);
    timers[i] = h.receiver_base.create_wall_timer(
        2ms,
        [&]() -> icey::Promise<void> {
          bool hit = ((mode++) % 2) == 0;
          (void)co_await tf.lookup("map", hit ? "base_link" : "missing_frame", rclcpp::Time(1700000000000000000LL + stamp_ns), 40ms);
          co_return;
        },
        group);
  }

  spin_for(h, 4s);
  for (auto &timer : timers) timer->cancel();
  pub_timer->cancel();
  SUCCEED();
}

TEST(NodeTasksThreadSafety, ServiceWithFourTimers) {
  Harness h;
  h.sender_ctx->create_service<ExampleService>(
      "stress_set_bool", [](std::shared_ptr<ExampleService::Request> req) {
        auto resp = std::make_shared<ExampleService::Response>();
        resp->success = req->data;
        return resp;
      });
  auto client = h.receiver_ctx->create_client<ExampleService>("stress_set_bool");
  bool flag = false;

  std::vector<rclcpp::CallbackGroup::SharedPtr> groups;
  std::array<std::shared_ptr<rclcpp::TimerBase>, 4> timers;
  for (size_t i = 0; i < timers.size(); ++i) {
    auto group = h.receiver->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    groups.push_back(group);
    timers[i] = h.receiver_base.create_wall_timer(
        2ms,
        [&]() -> icey::Promise<void> {
          auto req = std::make_shared<ExampleService::Request>();
          req->data = flag;
          flag = !flag;
          (void)co_await client.call(req, 200ms);
          co_return;
        },
        group);
  }

  spin_for(h, 4s);
  for (auto &timer : timers) timer->cancel();
  SUCCEED();
}

TEST(NodeTasksThreadSafety, ActionWithFourTimers) {
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
  int order = 2;

  std::vector<rclcpp::CallbackGroup::SharedPtr> groups;
  std::array<std::shared_ptr<rclcpp::TimerBase>, 4> timers;
  for (size_t i = 0; i < timers.size(); ++i) {
    auto group = h.receiver->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    groups.push_back(group);
    timers[i] = h.receiver_base.create_wall_timer(
        2ms,
        [&]() -> icey::Promise<void> {
          Fibonacci::Goal goal;
          goal.order = order;
          order = (order % 10) + 1;
          auto sent = co_await client.send_goal(goal, 200ms, [](auto, auto) {});
          if (sent.has_value()) (void)co_await sent.value().result(200ms);
          co_return;
        },
        group);
  }

  spin_for(h, 4s);
  for (auto &timer : timers) timer->cancel();
  SUCCEED();
}

TEST(NodeTasksThreadSafety, TSanRaceReproAsyncInternals) {
  Harness h;
  auto tf = h.receiver_ctx->create_transform_buffer();
  auto tf_pub = h.sender->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  size_t seq = 0;

  auto pub_timer = h.sender->create_wall_timer(1ms, [&]() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = rclcpp::Time(1700000000000000000LL + static_cast<int64_t>(++seq) * 1000000LL);
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.rotation.w = 1.0;
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(t);
    tf_pub->publish(msg);
  });

  std::vector<rclcpp::CallbackGroup::SharedPtr> groups;
  std::array<std::shared_ptr<rclcpp::TimerBase>, 12> timers;
  for (size_t i = 0; i < timers.size(); ++i) {
    auto group = h.receiver->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    groups.push_back(group);
    timers[i] = h.receiver_base.create_wall_timer(
        1ms,
        [&]() -> icey::Promise<void> {
          auto id = ++seq;
          h.receiver_base.add_task_for(id, 1ms, []() {});
          if ((id % 2U) == 0U) h.receiver_base.cancel_task_for(id);
          auto handle = tf.lookup("map", "base_link", icey::Clock::now(), 3ms, [](const auto &) {},
                                  [](const auto &) {});
          if ((id % 3U) == 0U) tf.cancel_request(handle);
          co_return;
        },
        group);
  }

  spin_for(h, 4s);
  for (auto &timer : timers) timer->cancel();
  pub_timer->cancel();
  SUCCEED();
}

}  // namespace
