cmake_minimum_required(VERSION 3.0.2)
project(common_util)

find_package(Protobuf REQUIRED)

set(Protobuf_INCLUDE_DIRS
  ${Protobuf_INCLUDE_DIRS}
  ${THIRD_PARTY_ROOT_DIR}/protobuf/src
  ${THIRD_PARTY_DIR}/protobuf/include
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
#              util               #
###################################
set(UTIL_SRC 
  file.cc
  util.cc
  string_tokenizer.cc
)

add_library(util SHARED 
  ${UTIL_SRC}
)

target_include_directories(util PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(util PRIVATE 
  ${PROTOBUF_LIBRARIES}
  string_util
  common_math
  common_proto
  perception_proto
)


###################################
#           string_util           #
###################################
set(STRING_UTIL_SRC 
  string_util.cc
)

add_library(string_util SHARED 
  ${STRING_UTIL_SRC}
)

target_include_directories(string_util PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(string_util PRIVATE 
  ${PROTOBUF_LIBRARIES}
)
