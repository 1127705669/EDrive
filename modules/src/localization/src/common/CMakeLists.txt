cmake_minimum_required(VERSION 3.0.2)
project(localization)

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
#      localization_gflags        #
###################################
set(LOCALIZATION_GFLAGS_SRC 
  localization_gflags.cc
)

add_library(localization_gflags SHARED 
  ${LOCALIZATION_GFLAGS_SRC}
)

target_link_libraries(localization_gflags 
  gflags
)

###################################
#      localization_common        #
###################################
add_library(localization_common INTERFACE)

target_link_libraries(localization_common INTERFACE
  localization_gflags
)