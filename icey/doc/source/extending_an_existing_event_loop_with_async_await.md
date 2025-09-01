# How to use C++ 20 coroutines to add an async/await API to an existing event loop with a callback-based API

# Background on the origins of async/await syntax

The main selling point of C++20 coroutines is that they allow to use the async/await syntax for asynchronous programming. 
This is an abstraction that greatly enhances code readability. 
Async/await as a syntactic programming language feature was first introduced in the C# language. It was later adopted in Python in 2015 and JavaScript in 2017. TODO Kotlin, Rust. 
Async/await syntax can be implemented in a programming language purely as a syntactic transformation. 
On the other hand, in systems programming languages it is usually implemented using *coroutines*. (TODO I think Rust and Kotlin as well)
This is how both concepts are related. 


# What this article is about 

This article is for developers that want to implement an async/await API for other event loops that already offer a callback-based API. 
In this article, we will focus however on only one application: Adding async/await syntax to an *existing* *single-threaded* event loop that has already has a callback-based API.
This use case is very relevant because there are many existing event-loop (or task-based parallelism) libraries out there that people do not want to reinvent (asio, ROS, Qt, etc.). 
Also, it is perfectly possible to do everything using a single thread: Asynchronous programming is an abstraction that may not necessarily be implemented using threads. 
As such, async/await becomes merely a syntactic sugar to solve the so-called *callback-hell* problem:

```cpp
void callback_based_function(std::function<void(int)> callback) {
    get_number_of_participants_async([](int result) {

        get_from_database_async([](int result2) {

            callback(result + result2);
        });
    });
}
```

### About the style of this article

C++ 20 coroutines are notoriously hard to understand. I think this is mainly an issue of presentation: They have many details that are unimportant for a first overview. 
They cover many use cases, but only one use case may be presented at first. 
A minor issue I think is also the specification itself: Confusing interfaces, inconsistenly named functions, simple concepts hiding behind opaque types.

In this article, I will try my best to explain coroutines understandably.
For this, I will deviate from the actual interface plementation while going through different *levels of detail*. Many things I will state as a fact about the behavrior are actually customizable. The concepts (state machine, coroutine frame) and the actual compiler transform are however very accurately explained, this is what the compiler actually does. 
I will skip over interface details, and simply ignore issues such as exceptio handling.
I think this is the didactically more sensible approach, in constrast to simply naming classes and functions without any motivation. 

# Background on coroutines 
Coroutines in computer science are functions that can return in the middle of the function, only to jump back in the middle of the function and continue running where they previously stopped.
Coroutines are an old concept, they are known since the 60s (TODO cide).
In high-level programming languages however, coroutines are difficult to implement since there is the function call stack: If we return from a function, local variables, especially function arguments that saved on stack get lost. 
So the main challenge is to save these local variables, this is why coroutines are a  language feature, because compiler support is needed.
Coroutines do not have stack, (at least C++20 coroutines are stackless) i.e. they do not store anything using stack pointer. 
Instead, the compiler allocates space for the coroutine arguments on the heap. 

# What is an event loop 

Here, I will briefly explain what the existing event loop with an callback-based API is. Essentially, it is a list of functions that are getting executed in order when calling `event_loop.spin()`. 
The event loop also has a method for adding new functions, i.e. callbacks. 
Here is how an implementation looks like:

```cpp
class EventLoop {
    using Task = std::function<void()>;
    void add_task(Task task) {
        tasks.push_back(task);
    }
    void spin() {
        while(!tasks.empty()) {
            auto task = tasks.pop();
            task();
        }
    }

    std::queue<Task> tasks;
};
```
The important part is that `add_task` does not start the task, but simply schedules is to be executed later (when `spin` is called). This is an important property that the event loop must have.

This is how this event loop is typically used: 

```cpp

void get_number_of_participants_from_db(EventLoop &event_loop, DBConnection db_connection, std::function<void(int)> callback) {
    event_loop.add_task([=]() {
        std::cout << "starting to query the Database..." <<std::endl;
        //db_connection.connect();
        std::this_thread::sleep(1s);
        std::cout << "Done!" <<std::endl;
        /// Call the callback after we are done 
        callback(42):
    });
}

void do_async(EventLoop &event_loop, DBConnection db_connection, std::function<void(int)> callback) {
    get_number_of_participants_from_db([=](int result){


        get_participants_from_db([=](int result2) {


            callback(result + result2);
        });
    });
}

int main()  {
    EventLoop ev;
    do_async(ev, [](int final_result) {
        std::cout << "Final result is: " << final_result <<std::endl;
    });
    ev.spin();
}
```

# How coroutines work as state machines

What we want is to be able to write the above callback-based code using async/await syntax, i.e. as the follwing coroutine:
```cpp

task<int> do_async_coroutine(EventLoop &event_loop, DBConnection *db_connection) {
    std::cout << "Before number_of_participants "<<std::endl;
    int result = co_await get_number_of_participants_from_db(event_loop, db_connection);

    std::cout << "Normal code here "<<std::endl;
    int result2 = co_await database(db_connection);
    co_return result + result2;
}

int main()  {
    EventLoop ev;
    /// This lambda is just a wrapper since the main function cannot use `co_await`
    const auto coro = []() {
        int final_result = co_await do_async_coroutine();
        std::cout << "Final result is: " << final_result <<std::endl;
    };
    coro();
    ev.spin();
}

```

For this, we must first understand how coroutines work. First, we will look how *returning out of the coroutine*, called coroutine *suspension*, and *jumping back in in the middle*, called coroutine *resuming*, is actually implemented.

The functions `number_of_participants` and `get_number_of_participants_from_db` are asynchronous functions, that only add a task to the event loop (as above). They return a type that can be `co_await`ed (a type that implements `operator co_await`)
The behavior that we want is to suspend the coroutine (i.e. return) after each `co_await` statement, so that we can start waiting in our event loop.

### state machine transformation
The above code is transformed by the compiler to a state machine. At very simplified version (first level of detail) looks like this [1]: 

```cpp
int do_async_coroutine(int current_index) {
    switch (current_index) {
        case 0: {
            std::cout << "Before number_of_participants "<<std::endl;
            auto promise = number_of_participants();
        } break;
        case 1: {
            int result = promise.get();
            std::cout << "Before database "<<std::endl;
            auto promise2 = database(db_connection);
        } break;
        case 2: {
            int result2 = promise2.get();
            /// Write the result somewhere: result + result2
        } break;
    }
    return 0;
}
```

This code snippet allows for some interesting observations:

- Coroutines are split into different secttions, for each `co_await` one
- Returning from one section and jumping back to another works using a switch-case
- The coroutine uses a variable `current_index` to remember how far it got
- Only one section is executed at a time: After the first section is executed, we break and therefore return.
- The transformed function does not have a stackframe: It cannot use any local variables. (If you are wondering how `current_index` is stored: in registers)


On the other hand, this example raises multiple questions however: 
- Where are the function arguments stored ? (i.e. `db_connection`)
- How does the coroutine remember how far it got, i.e. where is `current_index` stored ?
- Where are the intermediate results stored ?
- How is the result written back ?

# The coroutine frame 

To anwswer the questions above, we must introduce a new concept: the coroutine frame. This is how coroutines solve the problem of storing function argumets
Coroutines do not have stack, (at least C++20 coroutines are stackless) i.e. they do not store anything using stack pointer. 
Instead, the compiler allocates space for the coroutine arguments on the heap. 
This space is called the coroutine frame. 
A coroutine frame would look simethign like this for the above coroutine: 

```cpp
struct CoroFrame1 {
    int current_index{0};
    int result;
    DBConnection db_connection;
};
```

Before calling a coroutine, the compiler first allocates the coroutine frame. This frame is then passed to the coroutines via a pointer. The function containing the state machine is called the resume-function (*`_resume`). 

Overall, this is how the compiler transform looks like with one level of detail higher: 

```cpp

void do_async_coroutine() {
    CoroFrame *coro_frame = new CoroFrame1();
    do_async_coroutine_resume(frame);
}

int do_async_coroutine_resume(CoroFrame1 *frame) {
    int current_index = frame->current_index; /// Stored in register
    switch (current_index) {
        case 0: {
            std::cout << "Before number_of_participants "<<std::endl;
            auto promise = number_of_participants();
        } break;
        case 1: {
            int result = promise.get();
            std::cout << "Before database "<<std::endl;
            auto promise2 = database(frame->db_connection);
        } break;
        case 2: {
            int result2 = promise2.get();
            /// Write result for co_return
            frame->result = result2 + result;
        } break;
        case 3: { /// Deleter case:
            delete frame;
            /// Write result for co_return
        } break;
    }
    return 0;
}

int main() {
    do_async_coroutine();
}
```


## Lifetimes 

Now, it is important to talk about lifetimes about function arguments, local variables. 

- Function arguments are always stored in the coroutine frame and live during the entire coroutine.

The lifetime of local variables is a bit tricky.
You may have observed the scopes around the cases of the switch-case. This is really the lifetime of local variables, they only live until the next call to `co_await` ! 
Local variables are therefore not really preserved across different calls to `co_await`, i.e. *suspension points*, but only across one suspention point. The scopes in the switch-case accurately reflect this fact. 

### Coroutine frame

The coroutine frame is not destroyed automatically, but must be destroyed manually.



# References

- [1] Own research based on decompiling an minimal example binary (compiled using MSVC 19)
- [2] https://lewissbaker.github.io/2017/09/25/coroutine-theory