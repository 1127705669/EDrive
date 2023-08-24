cmake_minimum_required(VERSION 3.0.2)
project(adapters)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/proto)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  control
  planning
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(ADAPTER_SRC 
  adapter_manager.cc
)

add_library(adapters SHARED ${ADAPTER_SRC})

target_link_libraries(adapters adapter_proto)