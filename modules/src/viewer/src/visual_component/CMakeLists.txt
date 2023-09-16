cmake_minimum_required(VERSION 3.0.2)
project(visual_component)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/environment)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/planning)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/vehicle_state)

###################################
#    visual_component             #
###################################

add_library(visual_component INTERFACE)

target_link_libraries(visual_component INTERFACE
  vehicle_state_visualizer
)