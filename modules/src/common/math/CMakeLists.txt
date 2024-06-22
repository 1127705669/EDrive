cmake_minimum_required(VERSION 3.0.2)
project(math)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/qp_solver)

###################################
#   cartesian_frenet_conversion   #
###################################

add_library(cartesian_frenet_conversion SHARED 
  cartesian_frenet_conversion.cc
)

###################################
#            geometry             #
###################################

add_library(geometry SHARED 
  math_utils.cc
  vec2d.cc
)

###################################
#               lqr               #
###################################

add_library(lqr SHARED 
  linear_quadratic_regulator.cc
)

###################################
#               mpc               #
###################################

add_library(mpc SHARED
  mpc_solver.cc
)

target_link_libraries(mpc
  qp_solver
  active_set_qp_solver
)

###################################
#           math                  #
###################################

add_library(common_math INTERFACE)

target_link_libraries(common_math INTERFACE
  cartesian_frenet_conversion
  geometry
  lqr
)