cmake_minimum_required(VERSION 3.0.2)
project(routing_graph)

find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
#             graph               #
###################################
add_library(graph INTERFACE)

target_link_libraries(graph INTERFACE
  routing_node_with_range
  routing_sub_topo_graph
  routing_topo_graph
  routing_topo_node
  routing_topo_range_manager
)

###################################
#       routing_topo_range        #
###################################
add_library(routing_topo_range SHARED 
  topo_range.cc
)

target_include_directories(routing_topo_range PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(routing_topo_range 
  routing_gflags
)

###################################
#       routing_topo_node        #
###################################
add_library(routing_topo_node SHARED 
  topo_node.cc
)

target_include_directories(routing_topo_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(routing_topo_node PRIVATE
  ${Protobuf_LIBRARIES}
  common_proto
  map_proto
  routing_proto
)

###################################
#   routing_topo_range_manager    #
###################################
add_library(routing_topo_range_manager SHARED 
  topo_range_manager.cc
)

target_include_directories(routing_topo_range_manager PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(routing_topo_range_manager 
  routing_topo_node
  routing_topo_range
)

###################################
#    routing_node_with_range      #
###################################
add_library(routing_node_with_range SHARED 
  node_with_range.cc
)

target_include_directories(routing_node_with_range PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(routing_node_with_range 
  routing_topo_node
  routing_topo_range
)

###################################
#     routing_sub_topo_graph      #
###################################
add_library(routing_sub_topo_graph SHARED 
  sub_topo_graph.cc
)

target_include_directories(routing_sub_topo_graph PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(routing_sub_topo_graph 
  routing_node_with_range
  routing_topo_node
  common_proto
  map_proto
  routing_gflags
  routing_proto
  util
)

###################################
#       routing_topo_graph        #
###################################
add_library(routing_topo_graph SHARED 
  topo_graph.cc
)

target_include_directories(routing_topo_graph PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(routing_topo_graph 
  routing_topo_node
  common_proto
  map_proto
  routing_gflags
  routing_proto
  util
)