cmake_minimum_required(VERSION 3.16)
project(robotcore_control_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotcore_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(command_authority src/command_authority_node.cpp)
ament_target_dependencies(command_authority rclcpp robotcore_interfaces std_msgs)

install(TARGETS command_authority DESTINATION lib/${PROJECT_NAME})
ament_package()
