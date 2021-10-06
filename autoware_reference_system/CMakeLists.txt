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



if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
#ament_auto_add_executable(autoware_default_custom
#  src/ros2/executor/autoware_default_custom.cpp
#)

if(${BUILD_TESTING})
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # tracetools uses ROS_HOME to place tracing data
  if(NOT DEFINED $ENV{ROS_HOME})
    set(ROS_HOME $ENV{HOME}/.ros)
  else()
    set(ROS_HOME $ENV{ROS_HOME})
  endif()

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
      #autoware_default_custom
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
      # cpu     # built into memory tests using psrecord
    )

    # remove ros2_tracing trace_types if SKIP_TRACING is TRUE
    if(${SKIP_TRACING})
      message(STATUS "SKIP_TRACING is TRUE")
      message(STATUS "Removing callback trace tests")
      list(REMOVE_ITEM TRACE_TYPES "callback")
    endif()

    find_package(ros_testing REQUIRED)

    # make sure executable matches system requirements
    function(test_requirements target)
      set(TEST_EXECUTABLE ${CMAKE_CURRENT_BINARY_DIR}/${target})
      set(TEST_EXECUTABLE_NAME test_requirements_${target})

      # replaces all @var@ and ${var} within input file
      configure_file(
        test/test_requirements.py
        test_requirements_${target}.py
        @ONLY
      )

      add_ros_test(
        ${CMAKE_CURRENT_BINARY_DIR}/test_requirements_${target}.py
        TIMEOUT 3 # seconds
      )

      if(TARGET ${target})
        ament_target_dependencies(${target}
          "rclcpp" "reference_interfaces" "reference_system")
      endif()
    endfunction()

    # run target for n seconds
    function(generate_traces target  trace_type runtime)
      set(TEST_EXECUTABLE ${target})
      set(TEST_EXECUTABLE_NAME ${target}_${rmw_implementation})
      set(RUNTIME ${runtime})
      set(RMW_IMPLEMENTATION ${rmw_implementation})

      # ensure timeout is longer than the test runtime
      math(EXPR TIMEOUT "${runtime} + 5")

      if(${trace_type} MATCHES "memory")
        set(MEMRECORD_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/scripts/benchmark_one.sh")
        set(MEMRECORD_LOG "${ROS_HOME}/${trace_type}")
        set(MEMRECORD_LOG_NAME "${target}_${rmw_implementation}_${runtime}s.txt")
        set(MEMRECORD_EXE "$(ros2 pkg prefix ${PROJECT_NAME})/lib/${PROJECT_NAME}/${target}")
        # run psrecord script instead of ros2_tracing for memory usage plots
        add_test(
          NAME generate_${trace_type}_trace_${target}_${rmw_implementation}_${runtime}s
          COMMAND
            bash -c
              "chmod u+x ${MEMRECORD_PATH};
              RMW_IMPLEMENTATION=${rmw_implementation} \
              ${MEMRECORD_PATH} \
              ${MEMRECORD_EXE} \
              ${runtime} \
              ${MEMRECORD_LOG} \
              ${MEMRECORD_LOG_NAME}"
        )
      else()
        # replaces all @var@ and ${var} within input file
        configure_file(
          test/generate_${trace_type}_traces.py
          generate_${trace_type}_traces_${target}_${rmw_implementation}_${runtime}s.py
          @ONLY
        )

        add_ros_test(
          ${CMAKE_CURRENT_BINARY_DIR}/generate_${trace_type}_traces_${target}_${rmw_implementation}_${runtime}s.py
          TIMEOUT ${TIMEOUT} # seconds
        )
      endif()

      if(TARGET ${target})
        ament_target_dependencies(${target}
          "rclcpp" "reference_interfaces" "reference_system")
      endif()
    endfunction()

    # generate report from traces
    function(generate_report target trace_type runtime)
      if(${trace_type} MATCHES "memory")
        set(TRACE_DIR "${ROS_HOME}/${trace_type}/${target}_${rmw_implementation}_${runtime}s.txt")
        add_test(
          NAME generate_${trace_type}_report_${target}_${rmw_implementation}_${runtime}s
          COMMAND bash -c "python3 ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/test/generate_reports.py ${TRACE_DIR}"
          COMMENT "Generate CPU and Memory Usage report from psrecord data"
        )
      else()
        set(TRACE_DIR "${ROS_HOME}/tracing/${trace_type}_${target}_${rmw_implementation}_${runtime}s")
        # future note: this will fail on windows
        add_test(
          NAME convert_${trace_type}_trace_${target}_${rmw_implementation}_${runtime}s
          COMMAND bash -c "ros2 run tracetools_analysis convert ${TRACE_DIR}"
        )

        add_test(
          NAME process_${trace_type}_trace_${target}_${rmw_implementation}_${runtime}s
          COMMAND bash -c "ros2 run tracetools_analysis process ${TRACE_DIR}"
        )

        add_test(
          NAME generate_${trace_type}_report_${target}_${rmw_implementation}_${runtime}s
          COMMAND bash -c "python3 ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/test/generate_reports.py ${TRACE_DIR}"
          COMMENT "Process converted traces to create data model"
        )
      endif()
    endfunction()

    # generate summary report from all trace data
    function(generate_summary_report trace_type run_time)
      set(TRACE_DIR "${ROS_HOME}/${trace_type}")
      add_test(
        NAME generate_memory_summary_report_${trace_type}_${run_time}s
        COMMAND bash -c "python3 ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/test/generate_summary_reports.py ${TRACE_DIR} ${run_time}"
      COMMENT "Generate CPU and Memory Usage Summary Report"
      )

    endfunction()

    # get available rmw implementations
    find_package(rmw_implementation_cmake REQUIRED)
    get_available_rmw_implementations(rmws_available)

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
