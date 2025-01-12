cmake_minimum_required(VERSION 3.15)
project(icey)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wno-unused-parameter -Wno-reorder])
  add_compile_options(-fmax-errors=1)
   
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
    include
)

# Add examples 
ament_auto_add_executable(simple_example examples/simple.cpp)
ament_auto_add_executable(listener_example examples/listener.cpp)
ament_auto_add_executable(talker_example examples/talker.cpp)
ament_auto_add_executable(service_example examples/service.cpp)
ament_auto_add_executable(piping_example examples/piping.cpp)
ament_auto_add_executable(tf_approx_filter_example examples/tf_approx_filter.cpp)
ament_auto_add_executable(class_based_api_example examples/class_based_api.cpp)
ament_auto_add_executable(signal_generator_example examples/signal_generator.cpp)
ament_auto_add_executable(multi_node_example examples/multi_node.cpp)
ament_auto_add_executable(parameter_based_initialization_example examples/parameter_based_initialization.cpp)
ament_auto_add_executable(managing_state_example examples/managing_state.cpp)
ament_auto_add_executable(message_filters_interoperability_example examples/message_filters_interoperability.cpp)

ament_auto_package(
  INSTALL_TO_SHARE
)