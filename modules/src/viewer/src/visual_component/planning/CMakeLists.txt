cmake_minimum_required(VERSION 3.0.2)
project(planning_handle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(PLANNING_HANDLE_SRC 
  planning_handle.cc
)

add_library(planning_handle SHARED ${PLANNING_HANDLE_SRC})
add_dependencies(planning_handle ${viewer_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_handle 

)