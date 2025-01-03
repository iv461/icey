cmake_minimum_required(VERSION 3.5)
project(icey)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(icey_example_node
    src/icey_example_node.cpp 
    )

ament_target_dependencies(icey_example_node
    rclcpp 
    geometry_msgs 
    sensor_msgs 
    nav_msgs
    tf2_ros
    )

install(TARGETS
  icey_example_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()