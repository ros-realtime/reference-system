cmake_minimum_required(VERSION 3.5)
project(reference_system_autoware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Message.idl"
  DEPENDENCIES
    "std_msgs"
)


add_executable(reference_system src/reference_system.cpp)
target_include_directories(reference_system PRIVATE include)
ament_target_dependencies(reference_system rclcpp std_msgs)
rosidl_target_interfaces(reference_system ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  reference_system
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
    "std_msgs"
    "rosidl_default_runtime")
ament_package()
