# Actions using async/await


Similar to services, ICEY provides an async await based API for actions. 


## Action server 

## Use coroutines as callbacks:
```c++
 /// A function having a co_await expression must be a coroutine, i.e. return a promise
const auto on_handle_goal = [](auto uuid, auto goal) -> icey::Promise<void> {
    /// Here is your async/await based service call:
    auto result = co_await service_->call(req, timeout);
};
/// To be able to use coroutines as callbacks, simply wrap them inside a regular function and call them without co_await:
rclcpp_action::create_server<MyAction>(
      node, "my_action_name", [on_handle_goal](const rclcpp_action::GoalUUID & uuid,  std::shared_ptr<const MyAction::Goal> goal) {
        on_handle_goal(uuid, goal);
      }, ...);

```

## Beware the footgun: Lifetime of lambda coroutines

If a lambda function is a coroutine, you cannot simplify 
```c++
const auto f = []() -> icey::Promise<void> {
};
f();
```
into 
```c++
[]() -> icey::Promise<void> { }();
```
because this leads to an use-after-free. This is because temporary lifetime extension of lambda functions is not coroutine-aware (see [1](https://lists.isocpp.org/std-proposals/2020/05/1391.php))


# Action client 

The action client provides an async/await based API for requesting goals and awaiting the result: 

```c++
#include <example_interfaces/action/fibonacci.hpp>
using Fibonacci = example_interfaces::action::Fibonacci;

auto client = ctx->create_action_client<Fibonacci>("/fibonacci");

Fibonacci::Goal goal;
goal.order = 7;

// Request a goal with 2 seconds timeout: 
icey::Result<icey::AsyncGoalHandle<Fibonacci>, std::string> maybe_goal_handle =
    co_await client.send_goal(goal, 2s, [node](auto goal_handle, auto feedback) {
      RCLCPP_WARN(node->get_logger(), "Got some feedback");
    });

if (maybe_goal_handle.has_error()) {
  RCLCPP_WARN_STREAM(node->get_logger(),
                      "Goal request was rejected: " << maybe_goal_handle.error());
  co_return;
}

const icey::AsyncGoalHandle<Fibonacci> &goal_handle = maybe_goal_handle.value();
/// Now wait for the result of the action for 20 seconds.
auto maybe_result = co_await goal_handle.result(20s);
if (maybe_result.has_error()) {
  RCLCPP_WARN_STREAM(node->get_logger(), "Action failed: " << maybe_result.error());
  co_return;
}
auto const &result = maybe_result.value();
if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
  RCLCPP_INFO(node->get_logger(), "Fibonacci done. Size: %zu", result.result->sequence.size());
} else {
  RCLCPP_WARN(node->get_logger(), "Action finished with code %d",
              static_cast<int>(result.code));
    }
```