# Developing ICEY

## Build 


colcon build  --packages-select icey

## Debug 

```sh
ros2 run --prefix 'gdb -ex run --args' icey tf_approx_filter_example
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
