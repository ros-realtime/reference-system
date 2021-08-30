cmake_minimum_required(VERSION 3.5)
project(reference-system-autoware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(reference-system src/reference_system.cpp)
target_include_directories(reference-system PRIVATE include)
ament_target_dependencies(reference-system rclcpp std_msgs)

install(TARGETS
  reference-system
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
