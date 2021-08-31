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
  "msg/Message1kb.idl"
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
