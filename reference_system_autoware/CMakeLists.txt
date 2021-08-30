cmake_minimum_required(VERSION 3.5)
project(reference_system_autoware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# future reference system executables can be added here

# Single Threaded Executor
ament_auto_add_executable(autoware_default_singlethreaded
  src/ros2/executor/autoware_default_singlethreaded.cpp
)

# Multi Threaded Executor
ament_auto_add_executable(autoware_default_multithreaded
  src/ros2/executor/autoware_default_multithreaded.cpp
)

if(${BUILD_TESTING})
  # run test psrecord to generate KPIs
  # arguments for script are as follows:
  # 1. executable name
  # 2. path to save log data
  # 3. path to save plot
  # 4. duration to run test
  #  add_custom_command(TARGET autoware_default_singlethreaded
  #    POST_BUILD
  #    COMMAND
  #      ${CMAKE_CURRENT_SOURCE_DIR}/test/generate_graph.sh
  #      autoware_default_singlethreaded
  #      ${CMAKE_CURRENT_SOURCE_DIR}/results/cpu_mem_usage_results.txt
  #      ${CMAKE_CURRENT_SOURCE_DIR}/results/plot.png
  #      5
  #    ARG
  #    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  #    COMMENT "Run psrecord for specified target")
endif()

ament_auto_package(
  INSTALL_TO_SHARE test
)