cmake_minimum_required(VERSION 3.0.2)
project(viewer_proto)

## Generate Protocol Buffer message
## set protocol buffer output address
SET(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE proto_files ${CMAKE_CURRENT_SOURCE_DIR}/*.proto)

set(MESSAGE_SRC "")
set(MESSAGE_HDRS "")

foreach(proto_file ${proto_files})
  get_filename_component(FIL_WE ${proto_file} NAME_WE)
  list(APPEND MESSAGE_SRC "${PROTO_DIR}/${FIL_WE}.pb.cc")
  list(APPEND MESSAGE_HDRS "${PROTO_DIR}/${FIL_WE}.pb.h")
  add_custom_command(
    OUTPUT "${PROTO_DIR}/${FIL_WE}.pb.cc"
           "${PROTO_DIR}/${FIL_WE}.pb.h"
    COMMAND  ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROTO_DIR}
      -I ${PROTO_DIR}
      ${proto_file}
    DEPENDS ${proto_file}
    COMMENT "Running C++ protocol buffer compiler on ${proto_file}"
    VERBATIM
)
endforeach()

set_source_files_properties(${MESSAGE_SRC} ${MESSAGE_HDRS} PROPERTIES GENERATED TRUE)

add_custom_target(generate_viewer_proto_message ALL
  DEPENDS ${MESSAGE_SRC} ${MESSAGE_HDRS}
  COMMENT "generate Protocol Buffer target"
  VERBATIM
)

set(VIEWER_PROTO_SRC 
  viewer_conf.pb.cc
)

add_library(viewer_proto SHARED ${VIEWER_PROTO_SRC})

target_include_directories(viewer_proto PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${GENERATED_PROTO_DIR}
  ${Protobuf_INCLUDE_DIRS}
  ${PROTOBUF_SRC_DIR}
)

target_link_libraries(viewer_proto PRIVATE 
  ${Protobuf_LIBRARIES}
)