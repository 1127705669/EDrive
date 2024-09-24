cmake_minimum_required(VERSION 3.0.2)
project(vehicle_config)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/proto)

find_package(gflags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(VEHICLE_CONFIG_HELPER_SRC 
  vehicle_config_helper.cc
)

add_library(vehicle_config_helper SHARED ${VEHICLE_CONFIG_HELPER_SRC})

target_link_libraries(vehicle_config_helper 
  vehicle_config_proto
  util
)

###################################
#          config_gflags          #
###################################
add_library(config_gflags SHARED 
  config_gflags.cc
)

target_include_directories(config_gflags PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(config_gflags 
  gflags
)