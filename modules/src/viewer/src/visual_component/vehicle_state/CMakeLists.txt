cmake_minimum_required(VERSION 3.0.2)
project(vehicle_state)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ../
)

set(VEHIVLE_STATE_SRC 
  vehicle_state_visualizer.cc
)

add_library(vehicle_state_visualizer SHARED ${VEHIVLE_STATE_SRC})