cmake_minimum_required(VERSION 3.0.2)
project(qp_solver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

###################################
#         qp_solver_gflags        #
###################################

add_library(qp_solver_gflags SHARED 
  qp_solver_gflags.cc
)

target_link_libraries(qp_solver_gflags
  gflags
)

###################################
#           qp_solver             #
###################################

add_library(qp_solver SHARED 
  qp_solver.cc
)

target_link_libraries(qp_solver
  Eigen3::Eigen
)

###################################
#      active_set_qp_solver       #
###################################

add_library(active_set_qp_solver SHARED 
  active_set_qp_solver.cc
)

target_link_libraries(active_set_qp_solver
  qp_solver
  qp_solver_gflags
  Eigen3::Eigen
)