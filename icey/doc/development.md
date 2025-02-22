# Developing ICEY

## Run tests: 

Tests are build by default, run them with: 

```sh
cd <colcon-ws root>
./build/icey/test_main
```

## Debug build 

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
