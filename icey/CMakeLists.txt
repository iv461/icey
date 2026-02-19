cmake_minimum_required(VERSION 3.15)
project(icey)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ICEY_ASYNC_AWAIT_THREAD_SAFE "Enable synchronization in async/await internals" ON)
option(ICEY_ENABLE_TSAN "Build tests with ThreadSanitizer instrumentation" OFF)

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

include_directories(
    include
)

ament_auto_add_library(icey 
  src/actions/client.cpp 
  src/actions/server.cpp 
  src/actions/server_goal_handle.cpp  
  )

if(NOT ICEY_ASYNC_AWAIT_THREAD_SAFE)
  target_compile_definitions(icey PUBLIC ICEY_ASYNC_AWAIT_THREAD_SAFE=0)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_auto_add_gtest(test_main
    test/entities_test.cpp
    test/entities_async_await_test.cpp
    test/action_server_test.cpp
    test/action_client_test.cpp
    test/node_tasks_test.cpp
    test/test_main.cpp
    test/filters_test.cpp
    test/callback_lifetime_test.cpp
    test/context_test.cpp
    test/promise_test.cpp
    test/nested_coroutines.cpp
    test/promise_async_await_test.cpp
  )  
  target_link_libraries(test_main fmt::fmt)
  set_tests_properties(test_main PROPERTIES TIMEOUT 180)

  if(NOT ICEY_ASYNC_AWAIT_THREAD_SAFE)
    target_compile_definitions(test_main PRIVATE ICEY_ASYNC_AWAIT_THREAD_SAFE=0)
  endif()

  if(ICEY_ENABLE_TSAN)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(icey PUBLIC -fsanitize=thread -fno-omit-frame-pointer)
      target_link_options(icey PUBLIC -fsanitize=thread)
      target_compile_options(test_main PRIVATE -fsanitize=thread -fno-omit-frame-pointer)
      target_link_options(test_main PRIVATE -fsanitize=thread)
    else()
      message(WARNING "ICEY_ENABLE_TSAN is ON but compiler does not support TSAN flags")
    endif()
  endif()

endif()

if(ament_cmake_auto_VERSION EQUAL 2.7.3)
  ament_auto_package()
else()
  ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR  # can be removed after Jazzy EOL
  )
endif()
