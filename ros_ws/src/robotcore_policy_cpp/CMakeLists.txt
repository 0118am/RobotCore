cmake_minimum_required(VERSION 3.16)
project(robotcore_policy_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotcore_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATHS /usr/include/aarch64-linux-gnu REQUIRED)
find_library(NVINFER_LIBRARY nvinfer REQUIRED)
find_library(NVONNXPARSER_LIBRARY nvonnxparser REQUIRED)

add_executable(t60_policy src/t60_policy_node.cpp)
target_include_directories(t60_policy PRIVATE ${TENSORRT_INCLUDE_DIR})
ament_target_dependencies(
  t60_policy rclcpp robotcore_interfaces sensor_msgs std_msgs)
target_link_libraries(
  t60_policy CUDA::cudart ${NVINFER_LIBRARY} ${NVONNXPARSER_LIBRARY})

install(TARGETS t60_policy DESTINATION lib/${PROJECT_NAME})
ament_package()
