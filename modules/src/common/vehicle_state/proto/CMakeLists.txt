cmake_minimum_required(VERSION 3.0.2)
project(vehicle_state_proto)

set(PROTO_DIR_NAME "vehicle_state")

## Set protocol buffer output address
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PROTOBUF_IMPORT_DIRS ${CMAKE_SOURCE_DIR})
set(GENERATED_PROTO_DIR ${CATKIN_DEVEL_PREFIX}/include)

# Ensure the output directory exists
file(MAKE_DIRECTORY ${GENERATED_PROTO_DIR}/common/${PROTO_DIR_NAME}/proto)

file(GLOB_RECURSE proto_files ${PROTO_DIR}/*.proto)

set(MESSAGE_SRC "")
set(MESSAGE_HDRS "")

foreach(proto_file ${proto_files})
  get_filename_component(FIL_WE ${proto_file} NAME_WE)
  set(GEN_SRC ${GENERATED_PROTO_DIR}/common/${PROTO_DIR_NAME}/proto/${FIL_WE}.pb.cc)
  set(GEN_HDR ${GENERATED_PROTO_DIR}/common/${PROTO_DIR_NAME}/proto/${FIL_WE}.pb.h)
  list(APPEND MESSAGE_SRC ${GEN_SRC})
  list(APPEND MESSAGE_HDRS ${GEN_HDR})
  add_custom_command(
    OUTPUT ${GEN_SRC} ${GEN_HDR}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --cpp_out=${GENERATED_PROTO_DIR} -I${PROTOBUF_IMPORT_DIRS} ${proto_file}
    DEPENDS ${proto_file}
    COMMENT "Running C++ protocol buffer compiler on ${proto_file}"
    VERBATIM
  )
endforeach()

add_custom_target(generate_${PROTO_DIR_NAME}_message ALL
  DEPENDS ${MESSAGE_SRC} ${MESSAGE_HDRS}
  COMMENT "Generating map ${PROTO_DIR_NAME} Protocol Buffer messages"
)

# Create an empty library target
add_library(${PROTO_DIR_NAME}_proto SHARED)
add_dependencies(${PROTO_DIR_NAME}_proto generate_common_proto_message)

# Append source files to the target after they are generated
target_sources(${PROTO_DIR_NAME}_proto PRIVATE ${MESSAGE_SRC} ${MESSAGE_HDRS})

target_include_directories(${PROTO_DIR_NAME}_proto PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${GENERATED_PROTO_DIR}
  ${Protobuf_INCLUDE_DIRS}
  ${PROTOBUF_SRC_DIR}
)

# Link with other libraries
target_link_libraries(${PROTO_DIR_NAME}_proto
  ${Protobuf_LIBRARIES}
  localization_proto
)