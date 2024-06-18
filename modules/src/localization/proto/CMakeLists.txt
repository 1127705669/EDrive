cmake_minimum_required(VERSION 3.0.2)
project(localization_proto)

find_package(Protobuf REQUIRED)

## Set protocol buffer output address
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PROTOBUF_IMPORT_DIRS ${CMAKE_SOURCE_DIR})
set(GENERATED_PROTO_DIR ${CATKIN_DEVEL_PREFIX}/include)

# Ensure the output directory exists
file(MAKE_DIRECTORY ${GENERATED_PROTO_DIR}/localization/proto)

file(GLOB_RECURSE proto_files "${PROTO_DIR}/*.proto")

set(MESSAGE_SRC "")
set(MESSAGE_HDRS "")

foreach(proto_file ${proto_files})
  get_filename_component(FIL_WE ${proto_file} NAME_WE)
  set(GEN_SRC "${GENERATED_PROTO_DIR}/localization/proto/${FIL_WE}.pb.cc")
  set(GEN_HDR "${GENERATED_PROTO_DIR}/localization/proto/${FIL_WE}.pb.h")
  list(APPEND MESSAGE_SRC ${GEN_SRC})
  list(APPEND MESSAGE_HDRS ${GEN_HDR})
  add_custom_command(
    OUTPUT ${GEN_SRC} ${GEN_HDR}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out=${GENERATED_PROTO_DIR} -I${PROTOBUF_IMPORT_DIRS} -I${PROTO_DIR} ${proto_file}
    DEPENDS ${proto_file}
    COMMENT "Running C++ protocol buffer compiler on ${proto_file}"
    VERBATIM
  )
endforeach()

add_custom_target(generate_localization_message ALL
  DEPENDS ${MESSAGE_SRC} ${MESSAGE_HDRS}
  COMMENT "Generating Protocol Buffer messages"
)

# Create an empty library target
add_library(localization_proto SHARED)
add_dependencies(localization_proto generate_localization_message)

# Append source files to the target after they are generated
target_sources(localization_proto PRIVATE ${MESSAGE_SRC} ${MESSAGE_HDRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${GENERATED_PROTO_DIR}
)

# Link with other libraries
target_link_libraries(localization_proto

)