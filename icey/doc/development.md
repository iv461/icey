# Developing ICEY

## Build 

```
colcon build  --packages-select icey --mixin=debug 
```

Useful: Install mixins: https://github.com/colcon/colcon-mixin-repository

## Debug 

```sh
ros2 run --prefix 'gdb -ex run --args' icey tf_approx_filter_example
```

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
