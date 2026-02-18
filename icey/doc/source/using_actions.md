# Actions with async/await

ICEY provides an async/await API for ROS 2 actions on top of `rclcpp_action`.
The goal is to keep action code sequential and explicit while preserving asynchronous execution.

This page covers:
- creating an async action client
- sending goals and awaiting acceptance/result/cancellation
- creating an action server with synchronous or coroutine callbacks
- how this differs from plain ROS 2 action APIs

All examples are aligned with:
- `icey_examples/src/action_client_async_await.cpp`
- `icey_examples/src/action_server_async_await.cpp`

## Action client

Create a context and action client:

```c++
#include <example_interfaces/action/fibonacci.hpp>
#include <icey/icey_async_await.hpp>

using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;

icey::Promise<void> call_action(std::shared_ptr<rclcpp::Node> node) {
  auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());
  auto client = ctx->create_action_client<Fibonacci>("/icey_test_action_fibonacci");

  Fibonacci::Goal goal;
  goal.order = 7;

  auto maybe_goal_handle = co_await client.send_goal(
      goal,
      5s,  // timeout for goal acceptance
      [node](auto /*goal_handle*/, auto /*feedback*/) {
        RCLCPP_INFO(node->get_logger(), "Got action feedback");
      });

  if (maybe_goal_handle.has_error()) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Goal send failed: " << maybe_goal_handle.error());
    co_return;
  }

  auto goal_handle = maybe_goal_handle.value();

  auto maybe_result = co_await goal_handle.result(20s);  // timeout for result
  if (maybe_result.has_error()) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Result failed: " << maybe_result.error());
    co_return;
  }

  const auto &result = maybe_result.value();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node->get_logger(), "Goal succeeded");
  } else {
    RCLCPP_WARN(node->get_logger(), "Goal finished with code %d", static_cast<int>(result.code));
  }
}
```

### Client API behavior

`create_action_client<ActionT>(name)` returns `icey::ActionClient<ActionT>`.

`co_await client.send_goal(goal, timeout, feedback_cb)` returns:
- `icey::Result<icey::AsyncGoalHandle<ActionT>, std::string>`

`co_await goal_handle.result(timeout)` returns:
- `icey::Result<typename GoalHandle::WrappedResult, std::string>`

`co_await goal_handle.cancel(timeout)` returns:
- `icey::Result<CancelResponse::SharedPtr, std::string>`

Important:
- `goal_handle.cancel(timeout)` should be `co_await`ed immediately (do not fire-and-forget it).

Common error strings include:
- `TIMEOUT`
- `GOAL REJECTED`
- `RESULT TIMEOUT`
- `FAILED`
- exception messages from `rclcpp_action` in invalid-goal-handle scenarios

## Action server

The action server in `ContextAsyncAwait` accepts both synchronous and asynchronous callbacks.

```c++
auto handle_goal =
    [&](const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
        -> icey::Promise<rclcpp_action::GoalResponse> {
  // You can do async work before accepting/rejecting:
  // auto upstream = co_await some_client.call(req, 1s);
  co_return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
};

auto handle_cancel =
    [](std::shared_ptr<ServerGoalHandleFibonacci>) -> rclcpp_action::CancelResponse {
  return rclcpp_action::CancelResponse::REJECT;
};

auto handle_accepted = [&](std::shared_ptr<ServerGoalHandleFibonacci> goal_handle) {
  // Execute goal and publish final state
  std::this_thread::sleep_for(5s);
  goal_handle->succeed(std::make_shared<Fibonacci::Result>());
};

ctx->create_action_server<Fibonacci>(
    "/icey_test_action_fibonacci", handle_goal, handle_cancel, handle_accepted);
```

### Server callback types

`create_action_server` supports:
- `handle_goal`: sync or coroutine callback
- `handle_cancel`: sync or coroutine callback
- `handle_accepted`: currently regular callback

Coroutine callbacks let you `co_await` other async ICEY APIs (service calls, timers, other actions) before returning the action response.

## Plain ROS 2 vs ICEY

With plain `rclcpp_action` client code, you typically:
- build `SendGoalOptions`
- register goal-response/result/feedback callbacks manually
- track request and timeout lifecycle yourself
- spread control flow across callbacks

With ICEY async/await client code, you:
- call `send_goal(...)` and `co_await` it
- receive a typed `Result` immediately in one expression
- `co_await` result/cancel with explicit timeout
- keep control flow local and linear

With plain `rclcpp_action` server code, async pre-processing inside `handle_goal` / `handle_cancel` usually requires manual callback plumbing or additional state machines. ICEY allows these callbacks themselves to be coroutines, so asynchronous pre-checks can stay in one function.

## Practical migration notes

1. Replace `rclcpp_action::create_client` with `ctx->create_action_client<ActionT>(...)`.
2. Replace callback-driven send/result flow with `co_await send_goal` then `co_await result`.
3. Keep per-operation timeout values explicit in code.
4. For servers, migrate `handle_goal` and/or `handle_cancel` to `icey::Promise<...>` return types only where asynchronous work is needed.
5. Keep `rclcpp::spin(...)` as usual; ICEY integrates with the ROS 2 executor.
