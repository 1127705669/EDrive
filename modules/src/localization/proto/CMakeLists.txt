cmake_minimum_required(VERSION 3.0.2)
project(localization_proto)

find_package(Protobuf REQUIRED)

## Generate Protocol Buffer message
## set protocol buffer output address
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PROTOBUF_IMPORT_DIRS ${CMAKE_SOURCE_DIR})
set(GENERATED_PROTO_DIR ${CATKIN_DEVEL_PREFIX}/include)

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
    ARGS --cpp_out  ${GENERATED_PROTO_DIR}
      -I ${PROTOBUF_IMPORT_DIRS}
      -I ${PROTO_DIR}
      ${proto_file}
    DEPENDS ${proto_file}
    COMMENT "Running C++ protocol buffer compiler on ${proto_file}"
    VERBATIM
)
endforeach()

set_source_files_properties(${MESSAGE_SRC} ${MESSAGE_HDRS} PROPERTIES GENERATED TRUE)

add_custom_target(generate_localization_message ALL
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
  ${GENERATED_PROTO_DIR}/localization/proto/localization_conf.pb.cc
)

add_library(localization_proto SHARED ${ADAPTER_PROTO_SRC})

add_dependencies(localization_proto generate_localization_message)

target_link_libraries(localization_proto
  control_proto
  planning_proto
)