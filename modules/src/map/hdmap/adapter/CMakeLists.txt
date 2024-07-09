cmake_minimum_required(VERSION 3.0.2)
project(map_adapter)

set(CMAKE_PREFIX_PATH 
  ${CMAKE_PREFIX_PATH} 
  ${CMAKE_SOURCE_DIR}
  ${THIRD_PARTY_DIR}/tinyxml2
)

find_package(tinyxml2 REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_library(PROJ4_LIB NAMES proj PATHS /usr/local/lib /usr/lib /path/to/proj4)

if(NOT PROJ4_LIB)
    message(FATAL_ERROR "Cannot find libproj4 library")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

set(OPENDRIVER_ADAPTER_SRC
  coordinate_convert_tool.cc
  opendrive_adapter.cc
  proto_organizer.cc
  xml_parser/header_xml_parser.cc
  xml_parser/junctions_xml_parser.cc
  xml_parser/lanes_xml_parser.cc
  xml_parser/objects_xml_parser.cc
  xml_parser/roads_xml_parser.cc
  xml_parser/signals_xml_parser.cc
  xml_parser/util_xml_parser.cc
)

add_library(opendrive_adapter
  ${OPENDRIVER_ADAPTER_SRC}
)

target_include_directories(opendrive_adapter PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${THIRD_PARTY_DIR}/tinyxml2/include
  ${THIRD_PARTY_DIR}/glog/include
  ${THIRD_PARTY_DIR}/gflags/include
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

target_link_libraries(opendrive_adapter
  ${catkin_LIBRARIES}
  common_math
  util
  tinyxml2
  glog::glog
  gflags
  map_proto
  ${PROJ4_LIB}
)