cmake_minimum_required(VERSION 3.0.0)

project(rplidarsdk)

set(RPLIDARSDK_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Set this for parent cmake
set(RPLIDARSDK_INCLUDE_DIRS ${RPLIDARSDK_INCLUDE_DIRS} CACHE INTERNAL "")

# Set this to build this library
include_directories(${RPLIDARSDK_INCLUDE_DIRS})

# Add target as a static library
add_library(${PROJECT_NAME} STATIC
    src/sl_lidar_driver.cpp
    src/hal/thread.cpp
    src/sl_crc.cpp
    src/sl_serial_channel.cpp
    src/sl_lidarprotocol_codec.cpp
    src/sl_async_transceiver.cpp
    src/sl_tcp_channel.cpp
    src/sl_udp_channel.cpp
)
