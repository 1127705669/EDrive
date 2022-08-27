cmake_minimum_required(VERSION 3.0.2)

PROJECT (EDrive)

include_directories(/opt/ros/melodic/include)
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(./EROS/src)

add_executable(EDrive ./app/EDrive.cc)

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)