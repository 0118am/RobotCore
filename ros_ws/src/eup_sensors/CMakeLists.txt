cmake_minimum_required(VERSION 3.16)
project(eup_sensors LANGUAGES CXX)

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
find_package(eup_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(eup_localization_core src/fixed_lag_eskf.cpp)
target_include_directories(eup_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(eup_localization_core Eigen3::Eigen)

add_library(eup_sensors_components SHARED
  src/imu_conditioner_component.cpp
  src/zed_odometry_adapter_component.cpp
  src/apriltag_map_localizer_component.cpp
  src/fixed_lag_eskf_component.cpp)
target_include_directories(eup_sensors_components PUBLIC include)
target_link_libraries(eup_sensors_components
  eup_localization_core Eigen3::Eigen nlohmann_json::nlohmann_json ${OpenCV_LIBS} yaml-cpp
  isaac_ros_apriltag_interfaces::isaac_ros_apriltag_interfaces__rosidl_typesupport_cpp)
ament_target_dependencies(eup_sensors_components
  diagnostic_msgs diagnostic_updater eup_interfaces geometry_msgs
  nav_msgs rclcpp rclcpp_components sensor_msgs
  std_msgs std_srvs tf2 tf2_eigen tf2_geometry_msgs tf2_ros)

rclcpp_components_register_node(eup_sensors_components
  PLUGIN "eup_sensors::ImuConditionerComponent" EXECUTABLE imu_conditioning_node)
rclcpp_components_register_node(eup_sensors_components
  PLUGIN "eup_sensors::ZedOdometryAdapterComponent" EXECUTABLE zed_odometry_adapter_node)
rclcpp_components_register_node(eup_sensors_components
  PLUGIN "eup_sensors::AprilTagMapLocalizerComponent" EXECUTABLE apriltag_localization_node)
rclcpp_components_register_node(eup_sensors_components
  PLUGIN "eup_sensors::FixedLagEskfComponent" EXECUTABLE fixed_lag_eskf_node)

install(TARGETS eup_localization_core eup_sensors_components
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_fixed_lag_eskf test/test_fixed_lag_eskf.cpp)
  target_link_libraries(test_fixed_lag_eskf eup_localization_core Eigen3::Eigen)
endif()

ament_export_include_directories(include)
ament_export_libraries(eup_localization_core eup_sensors_components)
ament_package()
