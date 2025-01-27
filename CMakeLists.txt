cmake_minimum_required(VERSION 3.15)
project(icey)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fcoroutines -Werror=return-type -Werror=return-local-addr -Werror=init-self -Wno-strict-aliasing -fmax-errors=1)
  #add_compile_options(-Wall -Wextra -pedantic -Werror=return-type -Werror=return-local-addr -Werror=init-self -fmax-errors=1) 
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
    include
)

# Add examples 

#ament_auto_add_executable(listener_example examples/listener.cpp)
#ament_auto_add_executable(tf_approx_filter_example examples/tf_approx_filter.cpp)
#ament_auto_add_executable(tf_broadcaster_example examples/tf_broadcaster.cpp)
#ament_auto_add_executable(class_based_api_example examples/class_based_api.cpp)
#ament_auto_add_executable(multi_node_example examples/multi_node.cpp)
#ament_auto_add_executable(parameter_based_initialization_example examples/parameter_based_initialization.cpp)
#ament_auto_add_executable(managing_state_example examples/managing_state.cpp)
#ament_auto_add_executable(message_filters_interoperability_example examples/message_filters_interoperability.cpp)
#ament_auto_add_executable(service_client_and_server_example examples/service_client_and_server.cpp)
ament_auto_add_executable(signal_generator_example examples/signal_generator.cpp)

#ament_auto_add_executable(service_client_example examples/service_client.cpp)
#ament_auto_add_executable(service_server_example examples/service_server.cpp)

#ament_auto_add_executable(service_client_class_api_example examples/service_client_class_api.cpp)

#ament_auto_add_executable(parameters_struct_example examples/parameters_struct.cpp)


#ament_auto_add_executable(promise_test_async test/promise_test_async.cpp)
#ament_auto_add_executable(async_hello test/async_hello.cpp)

#ament_auto_add_executable(signal_generator_async_await_example examples/async_await/signal_generator_async_await.cpp)
#ament_auto_add_executable(service_client_async_await_example examples/async_await/service_client_async_await.cpp)


#ament_auto_add_executable(using_image_transport_example examples/using_image_transport.cpp)
#ament_auto_add_executable(automatic_synchronization_example examples/automatic_synchronization.cpp)

ament_auto_add_executable(mp_scratchpad examples/mp_scratchpad.cpp)

if(BUILD_TESTING)
  # Automatic inclusion of lint-related tests
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()

  # Add gtests
  find_package(ament_cmake_gtest REQUIRED)
  #add_subdirectory(test)
  
  #ament_auto_add_gtest(promise_test test/promise_test.cpp) 

  ##target_include_directories(promise_test PUBLIC include/)
  #ament_target_dependencies(promise_test rclcpp) 

endif()

ament_auto_package(
  INSTALL_TO_SHARE
)