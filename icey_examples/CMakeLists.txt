cmake_minimum_required(VERSION 3.15)
project(icey_examples)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic -Werror=return-type -Werror=init-self) 
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Werror=return-local-addr -fmax-errors=1) 
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Werror=return-stack-address) 
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Add examples 
ament_auto_add_executable(signal_generator_example src/signal_generator.cpp)
ament_auto_add_executable(parameters_struct_example src/parameters_struct.cpp)
ament_auto_add_executable(tf_subscription_example src/tf_subscription.cpp)
ament_auto_add_executable(tf_broadcaster_example src/tf_broadcaster.cpp)
ament_auto_add_executable(class_based_api_example src/class_based_api.cpp)
ament_auto_add_executable(service_client_example src/service_client.cpp)
ament_auto_add_executable(service_server_example src/service_server.cpp)

ament_auto_add_executable(signal_generator_async_await_example src/async_await/signal_generator_async_await.cpp)

ament_auto_add_executable(listener_async_await_example src/async_await/listener_async_await.cpp)
ament_auto_add_executable(talker_async_await_example src/async_await/talker_async_await.cpp)

ament_auto_add_executable(service_server_async_await_example src/async_await/service_server_async_await.cpp)
ament_auto_add_executable(service_client_async_await_example src/async_await/service_client_async_await.cpp)
ament_auto_add_executable(using_image_transport_example src/using_image_transport.cpp)

ament_auto_add_executable(listener_example src/listener.cpp)
ament_auto_add_executable(multi_node_example src/multi_node.cpp)
ament_auto_add_executable(message_filters_interoperability_example src/message_filters_interoperability.cpp)
ament_auto_add_executable(synchronization_example src/synchronization.cpp)

ament_auto_add_executable(lifecycle_node_example src/lifecycle_nodes.cpp)

# ament_auto_add_library(icey_autoware_gyro_odometer SHARED
#  autoware_gyro_odometer/gyro_odometer_core.cpp
# )

# rclcpp_components_register_node(icey_autoware_gyro_odometer
#  PLUGIN "autoware::gyro_odometer::GyroOdometerNode"
#  EXECUTABLE icey_autoware_gyro_odometer_node
#  EXECUTOR SingleThreadedExecutor
# )

ament_auto_package(
  INSTALL_TO_SHARE
)