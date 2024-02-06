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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sl_lidar_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hal/thread.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sl_crc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sl_serial_channel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sl_lidarprotocol_codec.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sl_async_transceiver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sl_tcp_channel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sl_udp_channel.cpp
)
