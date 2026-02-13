cmake_minimum_required(VERSION 3.15)
project(icey_examples)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

set(USE_ASAN_CHECKS OFF)
if(USE_ASAN_CHECKS)
    set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined")
    set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined")
    set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined")

    set (CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fno-omit-frame-pointer -fsanitize=address,undefined")
    set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-omit-frame-pointer -fsanitize=address,undefined")
    set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined")

    set (CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -fno-omit-frame-pointer -fsanitize=address,undefined")
    set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -fno-omit-frame-pointer -fsanitize=address,undefined")
    set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic -Werror=return-type -Werror=init-self)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Werror=return-local-addr)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Werror=return-stack-address)
endif()

find_package(fmt)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Add examples
#ament_auto_add_executable(service_server_example src/service_server.cpp)
#ament_auto_add_executable(signal_generator_example src/signal_generator.cpp)
#ament_auto_add_executable(parameters_struct_example src/parameters_struct.cpp)
#ament_auto_add_executable(tf_subscription_example src/tf_subscription.cpp)
#ament_auto_add_executable(tf_broadcaster_example src/tf_broadcaster.cpp)
#ament_auto_add_executable(using_icey_context_example src/using_icey_context.cpp)
#
#ament_auto_add_executable(signal_generator_async_await_example src/signal_generator_async_await.cpp)
#ament_auto_add_executable(listener_async_await_example src/listener_async_await.cpp)
#ament_auto_add_executable(talker_async_await_example src/talker_async_await.cpp)
#
#ament_auto_add_executable(tf_pub_test_example src/tf_pub_test.cpp)
#ament_auto_add_executable(tf_sychronization_example src/tf_sychronization.cpp)
#target_link_libraries(tf_sychronization_example fmt::fmt)
#
#ament_auto_add_executable(service_server_async_await_example src/service_server_async_await.cpp)
#ament_auto_add_executable(using_async_await_only_context_example src/using_async_await_only_context.cpp)
#
#ament_auto_add_executable(tf_lookup_async_await_example src/tf_lookup_async_await.cpp)
#
#ament_auto_add_executable(using_image_transport_example src/using_image_transport.cpp)
#
#ament_auto_add_executable(listener_example src/listener.cpp)
#ament_auto_add_executable(synchronization_example src/synchronization.cpp)
#
#ament_auto_add_executable(lifecycle_node_example src/lifecycle_nodes.cpp)
#ament_auto_add_executable(coroutine_example src/coroutine_example.cpp)
#ament_auto_add_executable(service_client_async_await_example src/service_client_async_await.cpp)
#target_link_libraries(service_client_async_await_example fmt::fmt)

ament_auto_add_executable(action_client_async_await_example src/action_client_async_await.cpp)
target_link_libraries(action_client_async_await_example fmt::fmt)

ament_auto_add_executable(action_server_async_await_example src/action_server_async_await.cpp)
target_link_libraries(action_server_async_await_example fmt::fmt)

if(ament_cmake_auto_VERSION EQUAL 2.7.3)
  ament_auto_package()
else()
  ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR  # can be removed after Jazzy EOL
  )
endif()

