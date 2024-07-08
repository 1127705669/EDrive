cmake_minimum_required(VERSION 3.0.2)
project(adapters)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/proto)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

set(ADAPTER_SRC 
  adapter_manager.cc
)

add_library(common_adapters SHARED ${ADAPTER_SRC})

add_dependencies(common_adapters 
  planning_generate_messages_cpp
  control_generate_messages_cpp
  perception_generate_messages_cpp
  localization_generate_messages_cpp
)

target_include_directories(common_adapters PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

target_link_libraries(common_adapters 
  adapter_proto
)