cmake_minimum_required(VERSION 3.15)
project(icey)

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

find_package(fmt)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
    include
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_auto_add_gtest(test_main
    test/entities_test.cpp
    test/entities_async_await_test.cpp
    test/actions_async_await_test.cpp
    test/node_tasks_test.cpp
    test/test_main.cpp
    test/filters_test.cpp
    test/callback_lifetime_test.cpp
    test/context_test.cpp
    test/promise_test.cpp
    test/nested_coroutines.cpp
  )
  ament_target_dependencies(test_main example_interfaces rclcpp_action)
  target_link_libraries(test_main fmt::fmt)

endif()

if(ament_cmake_auto_VERSION EQUAL 2.7.3)
  ament_auto_package()
else()
  ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR  # can be removed after Jazzy EOL
  )
endif()
