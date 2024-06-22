cmake_minimum_required(VERSION 3.0.2)
project(common)

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
#    pid_controller               #
###################################

set(PID_CONTROLLER_SRC 
  pid_controller.cc
)

add_library(pid_controller SHARED 
  ${PID_CONTROLLER_SRC}
)

###################################
#    trajectory_analyzer          #
###################################

set(TARJECTORY_ANALYZER_SRC
  trajectory_analyzer.cc
)

add_library(trajectory_analyzer SHARED 
  ${TARJECTORY_ANALYZER_SRC}
)

add_dependencies(trajectory_analyzer ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(trajectory_analyzer
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  common_proto
)

###################################
#       interpolation_2d          #
###################################

set(INTERPOLATION_2D
  interpolation_2d.cc
)

add_library(interpolation_2d SHARED 
  ${INTERPOLATION_2D}
)

###################################
#         control_common          #
###################################

add_library(control_common INTERFACE)

target_link_libraries(control_common INTERFACE
  interpolation_2d
  pid_controller
  trajectory_analyzer
)