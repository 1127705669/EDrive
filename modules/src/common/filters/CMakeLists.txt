cmake_minimum_required(VERSION 3.0.2)
project(filters)


find_package(PkgConfig)
pkg_check_modules(GLOG REQUIRED libglog)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GLOG_INCLUDE_DIRS}
)

###################################
#   digital_filter_coefficients   #
###################################
set(DIGITIAL_FILTER_COEFFICIENTS_SRC 
  digital_filter_coefficients.cc
)

add_library(digital_filter_coefficients SHARED ${DIGITIAL_FILTER_COEFFICIENTS_SRC})

target_link_libraries(digital_filter_coefficients 
  
)

###################################
#          digital_filter         #
###################################
set(DIGITIAL_FILTER_SRC 
  digital_filter.cc
)

add_library(digital_filter SHARED ${DIGITIAL_FILTER_SRC})

target_link_libraries(digital_filter 
  
)

###################################
#          mean_filter         #
###################################
set(MEAN_FILTER_SRC 
  mean_filter.cc
)

add_library(mean_filter SHARED ${MEAN_FILTER_SRC})

target_link_libraries(mean_filter 
  glog
)


###################################
#            filters              #
###################################

add_library(filters INTERFACE)

target_link_libraries(filters INTERFACE
  digital_filter_coefficients
  digital_filter
  mean_filter
)