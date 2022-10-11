cmake_minimum_required(VERSION 3.5)
project(autoware_reference_system)

# pass in via command line or set to True to test if platform is supported
message(STATUS "TEST_PLATFORM=${TEST_PLATFORM}")
if(${TEST_PLATFORM})
  message(STATUS "Building platform test")
else()
  message(STATUS "Not building platform tests")
endif()

# pass in via command line or set to True to run full benchmark tests
message(STATUS "RUN_BENCHMARK=${RUN_BENCHMARK}")
if(${RUN_BENCHMARK})
  # pass in via command line or set to True to skip ros2_tracing tests
  message(STATUS "SKIP_TRACING=${SKIP_TRACING}")
  if(${SKIP_TRACING})
    message(STATUS "Only building memory and CPU usage benchmark tests")
    message(STATUS "Skipping ros2_tracing (LTTng) tests")
  else()
    message(STATUS "Building full benchmark tests")
  endif()
else()
  message(STATUS "Not building benchmark tests")
endif()

# pass in via command line or set to True to run for all available RMWs
message(STATUS "ALL_RMWS=${ALL_RMWS}")
if(${ALL_RMWS})
  message(STATUS "Run tests on all available RMW's")
else()
  set(ALL_RMWS OFF)
  message(STATUS "Using default RMW for tests")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(kpsr_ros2_executor REQUIRED)
find_package(Klepsydra REQUIRED)
find_package(KlepsydraAdmin REQUIRED)
find_package(KlepsydraStreaming REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Single Threaded Executor
ament_auto_add_executable(autoware_default_singlethreaded
  src/ros2/executor/autoware_default_singlethreaded.cpp
)

# Static Single Threaded Executor
ament_auto_add_executable(autoware_default_staticsinglethreaded
  src/ros2/executor/autoware_default_staticsinglethreaded.cpp
)

# Multi Threaded Executor
ament_auto_add_executable(autoware_default_multithreaded
  src/ros2/executor/autoware_default_multithreaded.cpp
)

# Number Cruncher Benchmark
ament_auto_add_executable(number_cruncher_benchmark
  src/ros2/number_cruncher_benchmark.cpp
)

# Add new executors to test here
ament_auto_add_executable(autoware_default_custom
  src/ros2/executor/autoware_default_klepsydrasinglethreaded.cpp
)

ament_target_dependencies(autoware_default_custom ${dependencies})

if(${BUILD_TESTING})
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # tracetools uses ROS_HOME to place tracing data
  if(NOT DEFINED $ENV{ROS_HOME})
    set(ROS_HOME $ENV{HOME}/.ros)
  else()
    set(ROS_HOME $ENV{ROS_HOME})
  endif()

  # std trace type uses LOG directory
  if(NOT DEFINED $ENV{ROS_LOG_DIR})
    set(ROS_LOG_DIR ${CMAKE_INSTALL_PREFIX}/../../log/latest_test/${PROJECT_NAME})
  else()
    set(ROS_LOG_DIR $ENV{ROS_LOG_DIR})
  endif()

  set(DEFAULT_TIMEOUT 300)  # seconds, 300 = roughly 5min

  if(${TEST_PLATFORM})
    # check current platform
    ament_add_pytest_test(${PROJECT_NAME}
      test/test_platform.py
      TIMEOUT 5
    )
  endif()

  if(${RUN_BENCHMARK})
    # Add executables here to test
    set(TEST_TARGETS
      autoware_default_singlethreaded
      autoware_default_staticsinglethreaded
      autoware_default_multithreaded
      autoware_default_custom
    )

    # Add more run times here (time to run traces for)
    set(RUN_TIMES
      5
      # 10
      # 30
      # 60
    )

    # Add more trace types here
    # had to seperate out traces due to dropped events
    # see https://github.com/ros-realtime/reference-system/pull/33#issuecomment-928264240
    set(TRACE_TYPES
      callback  # uses ros2_tracing, LTTng
      memory    # uses psrecord
      std       # parses the log files that include the prints from std::cout
      # cpu     # built into memory tests using psrecord
    )

    # remove ros2_tracing trace_types if SKIP_TRACING is TRUE
    if(${SKIP_TRACING})
      message(STATUS "SKIP_TRACING is TRUE")
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
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_traces.cmake)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_report.cmake)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_summary_report.cmake)

    # check each executable matches the system requirements
    foreach(exe ${TEST_TARGETS})
      test_requirements(${exe} 3)
    endforeach()

    # generate traces for each executable
    foreach(exe ${TEST_TARGETS})
      # loop over each rmw implmentation
      foreach(rmw_implementation ${rmws_available})
        find_package("${rmw_implementation}" REQUIRED)
        foreach(time ${RUN_TIMES})
          foreach(type ${TRACE_TYPES})
              generate_traces(${exe} ${type} ${time})
              generate_report(${exe} ${type} ${time})
          endforeach()
        endforeach()
      endforeach()
    endforeach()

    foreach(trace_type ${TRACE_TYPES})
      foreach(time ${RUN_TIMES})
        generate_summary_report(${trace_type} ${time})
      endforeach()
    endforeach()
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE test scripts
)
