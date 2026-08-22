cmake_minimum_required(VERSION 3.16)
project(robotcore_sensors LANGUAGES CXX)

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
find_package(ament_cmake_gtest REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robotcore_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(zed_msgs REQUIRED)

add_library(robotcore_sensors_components SHARED
  src/apriltag_map_localizer_component.cpp
  src/vio_tag_fusion_component.cpp)
target_include_directories(robotcore_sensors_components PUBLIC include)
target_link_libraries(robotcore_sensors_components
  Eigen3::Eigen nlohmann_json::nlohmann_json ${OpenCV_LIBS}
  isaac_ros_apriltag_interfaces::isaac_ros_apriltag_interfaces__rosidl_typesupport_cpp)
ament_target_dependencies(robotcore_sensors_components
  diagnostic_msgs diagnostic_updater robotcore_interfaces geometry_msgs
  nav_msgs rclcpp rclcpp_components sensor_msgs
  std_srvs tf2 tf2_eigen tf2_geometry_msgs tf2_ros zed_msgs)

rclcpp_components_register_node(robotcore_sensors_components
  PLUGIN "robotcore_sensors::AprilTagMapLocalizerComponent" EXECUTABLE apriltag_localization_node)
rclcpp_components_register_node(robotcore_sensors_components
  PLUGIN "robotcore_sensors::VioTagFusionComponent" EXECUTABLE vio_tag_fusion_node)

install(TARGETS robotcore_sensors_components
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_apriltag_map test/test_apriltag_map.cpp)
  target_include_directories(test_apriltag_map PRIVATE include)
  target_link_libraries(test_apriltag_map Eigen3::Eigen nlohmann_json::nlohmann_json)
endif()

ament_export_include_directories(include)
ament_export_libraries(robotcore_sensors_components)
ament_package()
