cmake_minimum_required(VERSION 3.0.2)
project(math)

# Add osqp installation directory to CMAKE_PREFIX_PATH
set(CMAKE_PREFIX_PATH 
  ${CMAKE_PREFIX_PATH}
  ${THIRD_PARTY_DIR}/osqp/lib/cmake
  ${THIRD_PARTY_DIR}/osqp-eigen/lib/cmake
)

find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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

target_include_directories(lqr PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(lqr
  ${catkin_LIBRARIES}
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

target_include_directories(mpc_osqp PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${THIRD_PARTY_DIR}/osqp/include
  ${THIRD_PARTY_DIR}/osqp-eigen/include
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