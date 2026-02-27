# Developing ICEY

## Run tests: 

Tests are build by default, run them with: 

```sh
cd <colcon-ws root>
./build/icey/test_main
```

## Debug example node 

Useful: Install mixins: https://github.com/colcon/colcon-mixin-repository

```
colcon build  --packages-select icey --mixin=debug 
```

## Run with GDB

```sh
ros2 run --prefix 'gdb -ex run --args' icey_examples <node name>
```

## GDB 101: 

- `r` run 
- `c`: continue 
- `s, n`: step, next, respectively 
- `b <function name>`: just type the function name to set a breakpoint

- `bt` backtrace 
- `p <expression name>` : print the content of a variable

If you want to look at an exception that gets catched: 
- `catch throw`
If your node crashes at Ctrl+C:
- `handle SIGINT noprint nostop pass`

## Debug unit test: 

Since gtest catches the exceptions, we need to catch them earlier. 

So type in gdb: `catch throw`.

Then, do not use FastDDS since it uses exceptions as part of it's regular (instead of exceptional) control-flow. CylconeDDS worked for me instead. Therefore, the overall command is: 

```sh 
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp gdb ./build/icey/test_main
```


## TSAN (ThreadSanitizer)

TSAN is used to detect data races with multi-threaded executors. 
Build both `icey` and `icey_examples` with TSAN instrumentation:

```sh
cd <colcon-ws root>
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j6" colcon build --packages-select icey icey_examples --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DICEY_ENABLE_TSAN=ON -DICEY_ASYNC_AWAIT_THREAD_SAFE=1
```

Run an example binary under TSAN:

```sh
TSAN_OPTIONS="halt_on_error=1:detect_deadlocks=1:suppressions=/home/ivo/colcon_ws/src/icey/icey/test/tsan_dds.supp" setarch x86_64 -R /home/ivo/colcon_ws/install/icey_examples/lib/icey_examples/service_client_async_await_example
```

Why these workarounds are needed:

- `setarch x86_64 -R`: disables ASLR for the process. Without this, TSAN can fail early with `unexpected memory mapping` on Ubuntu 24 (https://bugs.launchpad.net/ubuntu/+source/linux/+bug/2056762, https://github.com/google/sanitizers/issues/1716#issuecomment-2010399341)
- `suppressions=.../tsan_dds.supp`: filters for known DDS races and deadlocks that only cause noise. 
- `halt_on_error=1`: fail fast 
- `detect_deadlocks=0`: There is lock inversion issue in the rclcpp_actions currently

By setting `-DICEY_ASYNC_AWAIT_THREAD_SAFE=0` during build, you can verify that indeed TSAN is able to detect the data races. 
## Run clang-tidy: 


1. Compile with compile-commands:
    ```sh
    colcon build  --packages-select icey --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    ```

1. Install clang-15 and clang-tidy-15 apt packages, other versions do not work for me
1. Run clang-tidy
    ```sh
    ~/autoware/src/icey/icey
    clang-tidy -p ~/autoware/build/icey/ include/icey/*.hpp --fix -fix-errors
    ```

To fix the compilation with clang in case it does not build: 
```sh
colcon build  --packages-select icey --cmake-args -DCMAKE_C_COMPILER=clang-15 -DCMAKE_CXX_COMPILER=clang++-15
```

## Build documentation 

The `doc` folder contains all files needed to build a documentation using the Sphinx site generator. 
We parse the C++-code using doxygen 

Dependencies: 

1. Install  doxygen: 
    ```sh
    sudo apt install doxygen
    ```
1. pip : 
    ```sh
    pip install -r requirements.txt 
    ```

1. Build:
    ```sh
    make html
    ```

Then, open the file [_build/html/index.html](_build/html/index.html) in your browser for an offline view. 


TODO deploy/GitHub actions
