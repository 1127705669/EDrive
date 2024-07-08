cmake_minimum_required(VERSION 3.0.2)
project(routing_common)

find_package(gflags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


###################################
#         routing_gflags          #
###################################
set(ROUTING_FLAGS_SRC 
  routing_gflags.cc
)

add_library(routing_gflags SHARED 
  ${ROUTING_FLAGS_SRC}
)

target_include_directories(routing_gflags PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(routing_gflags
  ${catkin_LIBRARIES}
  gflags
)