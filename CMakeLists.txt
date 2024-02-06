cmake_minimum_required(VERSION 3.0.0)

project(rplidar-simple-example)

add_subdirectory(rplidar_sdk)

include_directories(
    ${RPLIDARSDK_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    main.cpp
)

target_link_libraries(${PROJECT_NAME}
    rplidarsdk
)
