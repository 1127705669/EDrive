cmake_minimum_required(VERSION 3.0.2)
project(environment)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(ENV_HANDLE_SRC 
  env_handle.cc
)

add_library(env_data SHARED ${ENV_HANDLE_SRC})

target_link_libraries(env_data
  ${catkin_LIBRARIES}
  viewer_common
)