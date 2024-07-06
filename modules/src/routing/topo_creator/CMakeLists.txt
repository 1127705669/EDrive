cmake_minimum_required(VERSION 3.0.2)
project(topo_creator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
#         node_creator            #
###################################
set(NODE_CREATOR_SRC 
  node_creator.cc
)

add_library(node_creator SHARED 
  ${NODE_CREATOR_SRC}
)

target_include_directories(node_creator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

target_link_libraries(node_creator
  map_proto
  routing_gflags
  routing_proto
)

###################################
#         edge_creator            #
###################################
set(EDGE_CREATOR_SRC 
  edge_creator.cc
)

add_library(edge_creator SHARED 
  ${EDGE_CREATOR_SRC}
)

target_include_directories(edge_creator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

target_link_libraries(edge_creator
  map_proto
  routing_gflags
  routing_proto
)

###################################
#         graph_creator           #
###################################
# set(GRAPH_CREATOR_SRC 
#   graph_creator.cc
# )

# add_library(graph_creator SHARED 
#   ${GRAPH_CREATOR_SRC}
# )

# target_include_directories(graph_creator PUBLIC
#   ${CMAKE_CURRENT_SOURCE_DIR}
#   ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
# )

# target_link_libraries(graph_creator
#   edge_creator
#   node_creator
#   map_proto
#   routing_proto
# )