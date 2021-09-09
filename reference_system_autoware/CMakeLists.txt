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

# Static Single Threaded Executor
ament_auto_add_executable(autoware_default_staticsinglethreaded
  src/ros2/executor/autoware_default_staticsinglethreaded.cpp
)

# Multi Threaded Executor
ament_auto_add_executable(autoware_default_multithreaded
  src/ros2/executor/autoware_default_multithreaded.cpp
)

if(${BUILD_TESTING})
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # run target for 5 seconds
  function(test_reference_system_autoware_5s target)
    ament_add_gtest(test_${rmw_implementation}_${target}
      test/test_reference_system_autoware.cpp
      TIMEOUT 5
      APPEND_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}
      ENV
      RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
      RMW_IMPLEMENTATION=${rmw_implementation})
    if(TARGET ${target})
      ament_target_dependencies(${target}
        "rclcpp" "reference_interfaces")
    endif()
  endfunction()

  # get available rmw implementations
  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(rmws_available)

  # loop over each rmw implmentation
  foreach(rmw_implementation ${rmws_available})
    find_package("${rmw_implementation}" REQUIRED)

    # add each exe to test
    test_reference_system_autoware_5s(autoware_default_multithreaded)
  endforeach()
endif()

ament_auto_package(
  INSTALL_TO_SHARE test
)
