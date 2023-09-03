cmake_minimum_required(VERSION 3.0.2)
project(vehicle_state)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/proto)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(VEHICLE_STATE_SRC 
  vehicle_state_provider.cc
)

add_library(vehicle_state_provider SHARED ${VEHICLE_STATE_SRC})

target_link_libraries(vehicle_state_provider 
  vehicle_state_proto
)