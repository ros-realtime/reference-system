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
  message(STATUS "Building full benchmark tests")
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

# Add new executors to test here
#ament_auto_add_executable(autoware_default_custom
#  src/ros2/executor/autoware_default_custom.cpp
#)

if(${BUILD_TESTING})
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

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

    find_package(ros_testing REQUIRED)

    # run target for n seconds
    function(test_requirements target timeout)
      set(TEST_EXECUTABLE ${CMAKE_CURRENT_BINARY_DIR}/${target})
      set(TEST_EXECUTABLE_NAME test_requirements_${target}_${timeout})
      set(TIMEOUT ${timeout})

      # replaces all @var@ and ${var} within input file
      configure_file(
        test/test_requirements.py
        test_requirements_${target}_${timeout}s.py
        @ONLY
      )

      add_ros_test(
        ${CMAKE_CURRENT_BINARY_DIR}/test_requirements_${target}_${timeout}s.py
        TIMEOUT ${timeout} # seconds
      )

      if(TARGET ${target})
        ament_target_dependencies(${target}
          "rclcpp" "reference_interfaces" "reference_system")
      endif()
    endfunction()

    # run target for n seconds
    function(generate_traces target timeout)
      set(TEST_EXECUTABLE ${CMAKE_CURRENT_BINARY_DIR}/${target})
      set(TEST_EXECUTABLE_NAME ${target}_${rmw_implementation})
      set(TIMEOUT ${timeout})
      set(RMW_IMPLEMENTATION ${rmw_implementation})

      # replaces all @var@ and ${var} within input file
      configure_file(
        test/generate_traces.py
        generate_traces_${target}_${rmw_implementation}_${timeout}.py
        @ONLY
      )

      add_ros_test(
        ${CMAKE_CURRENT_BINARY_DIR}/generate_traces_${target}_${rmw_implementation}_${timeout}.py
        TIMEOUT ${timeout} # seconds
      )

      if(TARGET ${target})
        ament_target_dependencies(${target}
          "rclcpp" "reference_interfaces" "reference_system")
      endif()
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
        message(STATUS "genereate_traces for ${exe}_${rmw_implementation}_5s")
        #generate_traces(${exe} 5)
      endforeach()
    endforeach()
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE test
)
