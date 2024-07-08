cmake_minimum_required(VERSION 3.0.2)
project(map_hdmap)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/adapter)

###################################
#           hdmap_util            #
###################################
add_library(hdmap_util SHARED 
  hdmap_util.cc
)

target_include_directories(hdmap_util PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  ${THIRD_PARTY_DIR}/glog/include
)

target_link_libraries(hdmap_util
  common_adapters
  config_gflags
  util
  string_util
)
