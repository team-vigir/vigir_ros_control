cmake_minimum_required(VERSION 2.8.3)
project(vigir_ros_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joint_trajectory_controller
  vigir_joint_interfaces
  controller_interface
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS joint_trajectory_controller vigir_joint_interfaces controller_interface
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(HEADERS
include/${PROJECT_NAME}/vigir_pelvis_trajectory_controller.h
include/${PROJECT_NAME}/vigir_pelvis_trajectory_controller_impl.h
)

SET(SOURCE
src/vigir_joint_trajectory_controller.cpp
src/vigir_pelvis_trajectory_controller.cpp
)

add_library(${PROJECT_NAME} ${SOURCE} ${HEADERS} )
target_link_libraries(vigir_ros_controllers ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
install( TARGETS ${PROJECT_NAME} 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
	vigir_ros_controllers_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


