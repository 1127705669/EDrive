cmake_minimum_required(VERSION 3.0.2)
project(control)

# Set the third party directory
get_filename_component(PARENT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
get_filename_component(GRANDPARENT_DIR ${PARENT_DIR} DIRECTORY)
set(THIRD_PARTY_DIR ${GRANDPARENT_DIR}/third_party/build)

# Add osqp installation directory to CMAKE_PREFIX_PATH
set(CMAKE_PREFIX_PATH 
  ${CMAKE_PREFIX_PATH}
  ${THIRD_PARTY_DIR}/gflags/lib/cmake
  ${THIRD_PARTY_DIR}/glog/lib/cmake
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${THIRD_PARTY_DIR}/glog/include
  ${THIRD_PARTY_DIR}/gflags/include
)

# Link directories
link_directories(
  ${THIRD_PARTY_DIR}/glog/lib
  ${THIRD_PARTY_DIR}/gflags/lib
)

find_package(glog REQUIRED)
find_package(gflags REQUIRED)

# Find packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# file(GLOB MSG_FOLDERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/*)

# foreach(MSG_FOLDER ${MSG_FOLDERS})
#   message("Found msg folder: ${MSG_FOLDER}")
#   file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg/${MSG_FOLDER} ${CMAKE_CURRENT_SOURCE_DIR}/msg/${MSG_FOLDER}/*.msg)

#   foreach(MSG_FILE ${MSG_FILES})
#     message(" Found msg file: ${MSG_FILE}")
#   endforeach()
#   add_message_files(
#     DIRECTORY msg/${MSG_FOLDER}
#     FILES
#     ${MSG_FILES}
#   )

# endforeach()

add_message_files(
  DIRECTORY msg/control_cmd
  FILES
  TurnSignal.msg
  LatencyStats.msg
  ControlCommand.msg
  SimpleMPCDebug.msg
)

add_message_files(
  DIRECTORY msg/pid_conf
  FILES
  PidConf.msg
)

add_message_files(
  DIRECTORY msg/lon_controller_conf
  FILES
  FilterConf.msg
  LonControllerConf.msg
)

add_message_files(
  DIRECTORY msg/simulator_carla
  FILES
  CarlaEgoVehicleControl.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/proto)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/common)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/controller)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES control_proto
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

## Declare a C++ library
add_library(control_lib
  src/control.cc
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(control 
  src/main.cc
)

add_dependencies(control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(control
  ${catkin_LIBRARIES}
  EDrive_app
  control_lib
)

target_link_libraries(control_lib
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  controller
  control_proto
  EDrive_app
  common_adapters
  localization_proto
)