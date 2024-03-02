cmake_minimum_required(VERSION 3.0.2)
project(control_proto)

find_package(Protobuf REQUIRED)

## Generate Protocol Buffer message
## set protocol buffer output address
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PROTOBUF_IMPORT_DIRS "${CMAKE_SOURCE_DIR}")

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
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROTO_DIR}
      -I ${PROTO_DIR}
      -I ${PROTOBUF_IMPORT_DIRS} 
      ${proto_file}
    DEPENDS ${proto_file}
    COMMENT "Running C++ protocol buffer compiler on ${proto_file}"
    VERBATIM
)
endforeach()

set_source_files_properties(${MESSAGE_SRC} ${MESSAGE_HDRS} PROPERTIES GENERATED TRUE)

add_custom_target(generate_control_message ALL
  DEPENDS ${MESSAGE_SRC} ${MESSAGE_HDRS}
  COMMENT "generate Protocol Buffer target"
  VERBATIM
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
)

set(ADAPTER_PROTO_SRC 
  control_cmd.pb.cc
  control_conf.pb.cc
  lat_controller_conf.pb.cc
  lon_controller_conf.pb.cc
  pid_conf.pb.cc
)

add_library(control_proto SHARED ${ADAPTER_PROTO_SRC})