cmake_minimum_required(VERSION 3.16)
project(robotcore_hardware LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT ipo_supported)
    if(ipo_supported)
      set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
    endif()
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(robotcore_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(aboard_protocol src/aboard_protocol.cpp)
target_include_directories(aboard_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(aboard_bridge_node src/aboard_bridge_node.cpp)
target_include_directories(aboard_bridge_node PRIVATE include)
target_link_libraries(aboard_bridge_node aboard_protocol Boost::system)
ament_target_dependencies(aboard_bridge_node diagnostic_msgs diagnostic_updater robotcore_interfaces rclcpp sensor_msgs std_srvs)

install(TARGETS aboard_protocol aboard_bridge_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_aboard_protocol test/test_aboard_protocol.cpp)
  target_link_libraries(test_aboard_protocol aboard_protocol)
endif()

ament_export_include_directories(include)
ament_export_libraries(aboard_protocol)
ament_package()
