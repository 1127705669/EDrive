cmake_minimum_required(VERSION 3.0.2)
project(routing_strategy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
#            strategy             #
###################################
add_library(strategy INTERFACE)

target_link_libraries(controller INTERFACE
  routing_a_star_strategy
)

###################################
#     routing_a_star_strategy     #
###################################
add_library(routing_a_star_strategy SHARED 
  a_star_strategy.cc
)

target_include_directories(routing_a_star_strategy PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(routing_a_star_strategy 
  common_proto
  map_proto
  routing_gflags
  graph
  routing_proto
)
