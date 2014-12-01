cmake_minimum_required(VERSION 2.8.3)
project(vigir_humanoid_control_msgs)

#set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs actionlib_msgs actionlib)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
  FILES
  ControlModeStatus.msg
)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
  action
  FILES
  ChangeControlMode.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy message_runtime std_msgs actionlib_msgs actionlib
  #DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
    include/${PROJECT_NAME}/vigir_humanoid_control_msgs.h
)

set(SOURCES
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Declare a cpp library
# add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Declare a cpp executable
# add_executable(vigir_footstep_planning_msgs_node src/vigir_footstep_planning_msgs_node.cpp)

## Add dependencies to the executable
# add_dependencies(vigir_footstep_planning_msgs_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
# target_link_libraries(vigir_footstep_planning_msgs_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")
  
install(DIRECTORY msg
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  	FILES_MATCHING PATTERN "*.msg"
)

install(DIRECTORY action
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  	FILES_MATCHING PATTERN "*.action"
)

## Mark executables and/or libraries for installation
# install(TARGETS vigir_footstep_planning_msgs vigir_footstep_planning_msgs_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vigir_footstep_planning_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
