cmake_minimum_required(VERSION 3.5)
project(reference_system)

set(FRAMEWORK ros CACHE STRING "The framework to build for. Currently supported values are ROS 2 (\"ros\").")
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

# Add header-only library
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(${BUILD_TESTING})
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

  # unit tests
  ament_add_gtest(test_sample_management
      test/test_sample_management.cpp)
  target_link_libraries(test_sample_management ${PROJECT_NAME})
  ament_target_dependencies(test_sample_management reference_interfaces)

  ament_add_gtest(test_number_cruncher
      test/test_number_cruncher.cpp)
  target_link_libraries(test_number_cruncher ${PROJECT_NAME})

  # integration tests
  if(${FRAMEWORK} STREQUAL ros)
    ament_add_gtest(test_reference_system_rclcpp
        test/test_fixtures.hpp
        test/test_reference_system_rclcpp.cpp)
    target_link_libraries(test_reference_system_rclcpp ${PROJECT_NAME})
    ament_target_dependencies(test_reference_system_rclcpp reference_interfaces rclcpp)
  endif()
endif()

# Install
install(TARGETS ${PROJECT_NAME}
        EXPORT "export_${PROJECT_NAME}"
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION include
)

ament_python_install_package(reference_system_py)

ament_auto_package(
  INSTALL_TO_SHARE cfg
)
