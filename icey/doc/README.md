# Documentation 

Documentation site build using Sphinx, Doxygen and Breathe. 

# Building 

To build the documentation page: 

1. Install  doxygen: 
    ```sh
    sudo apt install doxygen
    ```
1. Install python dependencies: 
    ```sh
    pip install -r requirements.txt 
    ```

1. Build:
    ```sh
    make html
    ```

Then, open the file [_build/html/index.html](_build/html/index.html) in your browser for an offline view. 
