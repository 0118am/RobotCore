cmake_minimum_required(VERSION 3.16)
project(eup_firmware C)

# This standalone CMake target lets packet/failsafe code be compiled on a host
# machine before it is bound to a specific STM32 HAL or Aboard SDK.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(eup_firmware_packet
  src/packet.c
)

target_include_directories(eup_firmware_packet
  PUBLIC include
)

add_executable(eup_firmware_skeleton
  src/main.c
)

target_link_libraries(eup_firmware_skeleton
  PRIVATE eup_firmware_packet
)
