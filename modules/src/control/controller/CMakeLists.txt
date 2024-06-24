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
  mpc_controller
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
#         mpc_controller          #
###################################

set(MPC_CONTROLLER_SRC 
  mpc_controller.cc
)

add_library(mpc_controller SHARED ${MPC_CONTROLLER_SRC})

target_link_libraries(mpc_controller
  interpolation_1d
  interpolation_2d
  trajectory_analyzer
  vehicle_state_provider
  vehicle_config_helper
  digital_filter_coefficients
  digital_filter
  mean_filter
  geometry
)

###################################
#    controller                   #
###################################

add_library(controller INTERFACE)

target_link_libraries(controller INTERFACE
  controller_agent
  lon_controller
  lat_controller
  mpc_controller
)