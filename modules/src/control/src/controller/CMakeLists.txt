cmake_minimum_required(VERSION 3.0.2)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

###################################
#    controller_agent             #
###################################

set(CONTROLLER_AGENT_SRC 
  controller_agent.cc
)

add_library(controller_agent SHARED 
  ${CONTROLLER_AGENT_SRC}
)

target_link_libraries(controller_agent 
  lon_controller
  lat_controller
)

###################################
#    lon_controller               #
###################################

set(LON_CONTROLLER_SRC 
  lon_controller.cc
)

add_library(lon_controller SHARED ${LON_CONTROLLER_SRC})

target_link_libraries(lon_controller 
  pid_controller
  trajectory_analyzer
  vehicle_state_provider
  vehicle_config_helper
)

###################################
#    lat_controller               #
###################################

set(LAT_CONTROLLER_SRC 
  lat_controller.cc
)

add_library(lat_controller SHARED ${LAT_CONTROLLER_SRC})

target_link_libraries(lat_controller 
  trajectory_analyzer
  vehicle_state_provider
  vehicle_config_helper
)

###################################
#    controller                   #
###################################

add_library(controller INTERFACE)

target_link_libraries(controller INTERFACE
  controller_agent
  lon_controller
  lat_controller
)