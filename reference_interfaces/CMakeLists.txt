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

ament_auto_package()

# remove the below lines if fix to #143 is backported to older ROS distros
# fix rosidl_generator_py bug #143
# https://github.com/ros2/rosidl_python/issues/143
set(GENERATED_FILE "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/msg/_transmission_stats.py")

set(DISTROS_WITH_FIX
  "humble")
list(FIND DISTROS_WITH_FIX $ENV{ROS_DISTRO} DISTRO_ALREADY_FIXED)

if (${DISTRO_ALREADY_FIXED} MATCHES -1)
  message(STATUS "checking generated file: ${GENERATED_FILE}")

  set(DISTROS_WITH_RENAMED_TARGET "rolling")
  list(FIND DISTROS_WITH_RENAMED_TARGET $ENV{ROS_DISTRO} TARGET_RENAMED)

  if (${TARGET_RENAMED} MATCHES -1)
    set(TARGET_NAME ${PROJECT_NAME}__python)
  else()
    set(TARGET_NAME ${PROJECT_NAME}__rosidl_generator_py)
  endif()

  add_custom_command(
    TARGET ${TARGET_NAME}
    POST_BUILD
    COMMAND sed -i "s/all(val >= 0 and val) < 256/all(ord(val) >= 0 and ord(val) < 256/" ${GENERATED_FILE}
    COMMENT "Check generated IDL files for extra parenthesis..."
    VERBATIM)
endif()
