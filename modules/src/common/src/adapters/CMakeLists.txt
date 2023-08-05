cmake_minimum_required(VERSION 3.0.2)
project(adapters)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/proto)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_library(ADAPTERS_DEPEND_LIBS adapter_proto PATH ${CMAKE_SOURCE_DIR}/common/lib)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
)

set(ADAPTER_SRC 
  adapter_manager.cc
)

add_library(adapters SHARED ${ADAPTER_SRC})

set_target_properties(adapters PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/common/lib
)

# target_link_libraries(adapters PRIVATE ${MY_SHARED_LIBRARY})