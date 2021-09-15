cmake_minimum_required(VERSION 3.5)
project(reference_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# add additional messages here
set(msg_files
  "msg/TransmissionStats.idl"
  "msg/Message4kb.idl"
)

# add additional message dependencies here
#set(msg_dependencies
#  "std_msgs"
#)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    ${msg_dependencies}
  ADD_LINTER_TESTS
)

# fix rosidl_generator_py bug #143
# https://github.com/ros2/rosidl_python/issues/143
add_custom_command(
  TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND sed -i "s/all\\(val \\>\\= 0 and val\\) \\< 256/all\\(ord\\(val\\) \\>\\= 0 and ord\\(val\\) \\< 256/" ${CMAKE_BINARY_DIR}/rosidl_generator_py/reference_interfaces/msg/_transmission_stats.py
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Check generated message files for extra parenthesis..."
)

ament_auto_package()
