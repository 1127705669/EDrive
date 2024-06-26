cmake_minimum_required(VERSION 3.0.2)
project(math)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)

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

target_link_libraries(lqr
  Eigen3::Eigen
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
  Eigen3::Eigen
)

###################################
#            mpc_osqp             #
###################################
add_library(mpc_osqp SHARED
  mpc_osqp.cc
)

target_link_libraries(mpc_osqp
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  osqp::osqp
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