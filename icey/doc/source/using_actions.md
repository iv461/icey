# Actions with async/await

ICEY wraps `rclcpp_action` with coroutine-friendly APIs so goal flow stays linear instead of callback-driven.

References:
- `icey_examples/src/action_client_async_await.cpp`
- `icey_examples/src/action_server_async_await.cpp`

## Client API in three operations

Create client once:

```c++
auto ctx = std::make_shared<icey::ContextAsyncAwait>(node.get());
auto client = ctx->create_action_client<Fibonacci>("/icey_test_action_fibonacci");
```

### 1) Send goal (`send_goal`)

```c++
Fibonacci::Goal goal;
goal.order = 7;

auto maybe_handle = co_await client.send_goal(
    goal,
    5s,  // acceptance timeout
    [node](auto, auto) { RCLCPP_INFO(node->get_logger(), "feedback"); });

if (maybe_handle.has_error()) {
  // e.g. "TIMEOUT", "GOAL REJECTED"
  co_return;
}

auto handle = maybe_handle.value();
```

What you get:
- `icey::Result<icey::AsyncGoalHandle<ActionT>, std::string>`
- explicit timeout for acceptance

### 2) Await result (`result`)

```c++
auto maybe_result = co_await handle.result(20s);  // result timeout
if (maybe_result.has_error()) {
  // e.g. "RESULT TIMEOUT", "FAILED"
  co_return;
}

const auto &wrapped = maybe_result.value();
if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
  // wrapped.result is ActionT::Result
}
```

What you get:
- `icey::Result<typename ClientGoalHandle::WrappedResult, std::string>`
- result timeout and cleanup handled by ICEY

### 3) Cancel goal (`cancel`)

```c++
auto maybe_cancel = co_await handle.cancel(2s);
if (maybe_cancel.has_error()) {
  // timeout / invalid-handle style errors
  co_return;
}
```

What you get:
- `icey::Result<CancelResponse::SharedPtr, std::string>`

Important:
- `cancel(...)` must be awaited immediately (do not fire-and-forget it).

## Server API

`create_action_server` accepts synchronous or coroutine callbacks for `handle_goal`, `handle_cancel` and `handle_accepted `.

```c++
auto handle_goal = [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>)
    -> icey::Promise<rclcpp_action::GoalResponse> {
  // can await service calls, timers, etc.
  co_return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
};

auto handle_cancel = [](std::shared_ptr<ServerGoalHandleFibonacci>) {
  return rclcpp_action::CancelResponse::REJECT;
};

auto handle_accepted = [](std::shared_ptr<ServerGoalHandleFibonacci> gh) {
  gh->succeed(std::make_shared<Fibonacci::Result>());
};

ctx->create_action_server<Fibonacci>("/icey_test_action_fibonacci",
                                     handle_goal,
                                     handle_cancel,
                                     handle_accepted);
```

## What this enables (beyond syntactic sugar)

1. Compose action stages directly:
`send_goal -> result -> conditional cancel/retry` in one coroutine.
2. Perform async prechecks in server callbacks:
await service calls or timers before accepting/rejecting a goal.
3. Keep timeout policy close to business logic:
acceptance/result/cancel each have separate timeouts.
4. Write deterministic control flow:
less shared mutable callback state, fewer manually managed request IDs.

## Plain ROS 2 vs ICEY

Plain `rclcpp_action` commonly needs manual `SendGoalOptions`, multiple callbacks, and external state to connect acceptance/result/cancel steps.

ICEY keeps the same ROS executor model (`rclcpp::spin(...)`) but lets you express those steps with typed `co_await` operations and explicit per-step timeout handling.
