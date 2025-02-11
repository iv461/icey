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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
    include
)

if(BUILD_TESTING)
  # Automatic inclusion of lint-related tests
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()

  # Add gtests
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_auto_add_gtest(promise_test test/promise_test.cpp) 

  target_include_directories(promise_test PUBLIC include/)
  ament_target_dependencies(promise_test rclcpp) 

endif()

ament_auto_package(
  INSTALL_TO_SHARE
)