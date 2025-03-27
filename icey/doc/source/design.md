# Why not use `std::promise`/`std::future` ? 

ICEY introduces it's own asynchronous abstractions `icey::Promise` and `icey::Stream`. 
Given that there are classes called `std::future` and `std::promise` in the C++ standard library, you may wonder why we introduce our own types. 

The answer is that the `std::promise` are greatly under-specified -- they have great limitations in what you can do with them, most importantly you can't register a continuation [4] to a `std::promise`, i.e. you can't call `.then()`. Without the ability for adding a continuation however, you cannot sequence asynchronous operations. But this is the most useful thing of promises (and actually what differentiates them from a futures). 
With such a fundamental limitation, they are not really useful -- they end up being merely a wrapper over a value with a corresponding condition variable. 

Since C++20, you can even `co_await` a `std::future` -- but what this does is unexpected and actually really undesirable [2]. It spawns a new thread and then *detaches* it. This has a lot of problems -- you cannot specify timeouts and the thread may run for an infinitely long time [2], you have no control over the thread creation, usually you want to use a thread pool to amortize the cost of a thread creation context switch. 
You may actually not want to use thread at all, if you have your own event loop, like we do with ROS. 

Overall, the design of `std::future` and `std::promise` is widely recognized as a failure, a bad compromise. 
The promises in the standard library neither offer what is usually expected from multiple other programming languages such as JavaScript and C#.
This is why usually other (big) projects also implement these asynchronous abstractions, see e.g. Mozilla's Promise [3], Meta's old promises [5] used in the Folly library [6] or the new ones in the libunifex [7] library.


## References

- [2] *What happens if you co_await a std::future, and why is it a bad idea?* by Raymond Chen: https://devblogs.microsoft.com/oldnewthing/20230217-00/?p=107842
- [3] Mozilla Promise implementation: https://firefox-source-docs.mozilla.org/xpcom/mozpromise.html
- [4] std::future lacks continuation, current (2024) state of the art: https://ikriv.com/blog/?p=4916
- [5] https://engineering.fb.com/2015/06/19/developer-tools/futures-for-c-11-at-facebook/
- [6] [Folly ](https://github.com/facebook/folly)
- [7] https://github.com/facebookexperimental/libunifex