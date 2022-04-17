cmake_minimum_required(VERSION 3.5)
project(autoware_reference_system)

option(RUN_BENCHMARK "Run the full benchmark tests" OFF)  # default to off for CI purposes
option(SKIP_TRACING "Skip the ros2_tracing (LTTng) tests" ON)  # default to off until ros-realtime/reference_system#35 is closed
option(TEST_PLATFORM "Test if running on a supported platform" OFF)  # default to off for development purposes
option(ALL_RMWS "Run tests for all available RMWs" OFF)  # default to off so only one RMW is tested by default
set(FRAMEWORK ros CACHE STRING "The framework to build for. Currently supported is ROS 2 (\"ros\").")
set(AVAILABLE_FRAMEWORKS ros)

if(NOT ${FRAMEWORK} IN_LIST AVAILABLE_FRAMEWORKS)
  message(FATAL_ERROR "Unsupported framework: ${FRAMEWORK}")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Number Cruncher Benchmark
ament_auto_add_executable(number_cruncher_benchmark
  src/number_cruncher_benchmark.cpp)

set(BENCHMARK_EXECUTABLES "")
macro(add_benchmark_executable target src)
  ament_auto_add_executable(${target} ${src} src/priorities.cpp)
  list(APPEND BENCHMARK_EXECUTABLES ${target})
endmacro()

if(${FRAMEWORK} STREQUAL ros)
  # Single Threaded Executor
  add_benchmark_executable(autoware_default_singlethreaded
    src/ros2/executor/autoware_default_singlethreaded.cpp)
  # Multi Threaded Executor
  add_benchmark_executable(autoware_default_multithreaded
    src/ros2/executor/autoware_default_multithreaded.cpp)

  # Static Single Threaded Executor
  add_benchmark_executable(autoware_default_staticsinglethreaded
    src/ros2/executor/autoware_default_staticsinglethreaded.cpp)

  # Prioritized Executor
  add_benchmark_executable(autoware_default_prioritized
    src/ros2/executor/autoware_default_prioritized.cpp)

  # Multiple executors on callback-group-level with suitable prioritization of critical path.
  add_benchmark_executable(autoware_default_cbg
    src/ros2/executor/autoware_default_cbg.cpp)
endif()

# Add new executors to test here
#add_benchmark_executable(autoware_default_custom
#  src/ros2/executor/autoware_default_custom.cpp
#)

if(${BUILD_TESTING})
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  if(${TEST_PLATFORM})
    # check current platform
    ament_add_pytest_test(platform_test
      test/test_platform.py
      TIMEOUT 5
    )
  endif()

  if(${RUN_BENCHMARK})
    # Test all executables by default.
    # Modify this variable to test only a subset of executables
    set(TEST_TARGETS ${BENCHMARK_EXECUTABLES})

    # Add more run times here (time to run traces for)
    set(RUN_TIMES
      5
      # 10
      # 30
      # 60
    )

    # Add more trace types here
    # The CPU trace type is disabled since it produces too many events
    # see https://github.com/ros-realtime/reference-system/pull/33#issuecomment-928264240
    set(TRACE_TYPES
      callback  # uses ros2_tracing, LTTng
      memory    # uses psrecord
      std       # parses the log files that include the prints from std::cout
      # cpu     # built into memory tests using psrecord
    )

    # remove ros2_tracing trace_types if SKIP_TRACING is ON
    if(${SKIP_TRACING})
      message(STATUS "SKIP_TRACING is ON")
      message(STATUS "Removing callback trace tests")
      list(REMOVE_ITEM TRACE_TYPES "callback")
    endif()

    find_package(ros_testing REQUIRED)

    # get available rmw implementations
    find_package(rmw_implementation_cmake REQUIRED)
    get_available_rmw_implementations(rmws_available)

    # only use default RMW by default
    if(${ALL_RMWS} MATCHES OFF)
      list(REVERSE rmws_available)
      foreach(rmw ${rmws_available})
        list(LENGTH rmws_available COUNT)
        if(NOT COUNT MATCHES 1)
          # message("Removing ${rmw} from tests")
          list(REMOVE_AT rmws_available COUNT)
        endif()
      endforeach()
    endif()

    # include cmake functions to use later on
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/test_requirements.cmake)

    # check each executable matches the system requirements
    foreach(exe ${TEST_TARGETS})
      test_requirements(${exe} 3)
    endforeach()

    list(JOIN rmws_available "," RMWS_COMMA_LIST)
    list(JOIN TEST_TARGETS "," EXES_COMMA_LIST)
    list(JOIN TRACE_TYPES "," TRACE_TYPES_COMMA_LIST)
    list(JOIN RUN_TIMES "," RUN_TIMES_COMMA_LIST)

    # generate traces for each executable
    set(TEST_NAME benchmark)
    include(FindPython3)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(
        NAME ${TEST_NAME}
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/scripts/benchmark.py
        --trace_types ${TRACE_TYPES_COMMA_LIST}
        --rmws ${RMWS_COMMA_LIST}
        ${RUN_TIMES_COMMA_LIST} ${EXES_COMMA_LIST})
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE test scripts
)
