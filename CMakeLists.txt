cmake_minimum_required(VERSION 3.5)
project(icey)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
    include
)

ament_auto_add_executable(icey_example_node
  src/icey_example_node.cpp 
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)