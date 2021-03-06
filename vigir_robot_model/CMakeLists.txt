cmake_minimum_required(VERSION 2.8.3)
project(vigir_robot_model)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
set(ROS_BUILD_TYPE RelWithDebInfo)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Eigen REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs vigir_utilities urdf eigen_stl_containers eigen_conversions pcl_ros rbdl)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${LIBRARY_OUTPUT_PATH})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msgflor
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

SET(HEADERS
include/${PROJECT_NAME}/VigirRobotDataTypes.h
include/${PROJECT_NAME}/VigirRobotState.h
include/${PROJECT_NAME}/VigirRobotModel.h
include/${PROJECT_NAME}/VigirRobotRBDLModel.h
include/${PROJECT_NAME}/VigirRobotFilterBase.h
include/${PROJECT_NAME}/VigirRobotSimpleFilter.h
include/${PROJECT_NAME}/VigirRobotBasic2StateKF.h
include/${PROJECT_NAME}/VigirRobotPoseFilterBase.h
include/${PROJECT_NAME}/VigirRobotPoseFilter.h
include/${PROJECT_NAME}/VigirRobotCalibrationBase.h
include/${PROJECT_NAME}/VigirRobotStability.h
include/${PROJECT_NAME}/VigirTimeOffset.h
)

SET(SOURCE
src/VigirRobotState.cpp
src/VigirRobotModel.cpp
src/VigirRobotRBDLModel.cpp
src/VigirRobotBasic2StateKF.cpp
src/VigirRobotPoseFilter.cpp
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependeflornt projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vigir_robot_model
  CATKIN_DEPENDS eigen_stl_containers eigen_conversions

#  DEPENDS system_lib
)

###########flor_dynamics
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCE} ${HEADERS} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Declare cpp executables for testing 
add_executable(test_${PROJECT_NAME} src/test_vigir_robot_model.cpp ${HEADERS})
target_link_libraries(test_${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME} )
add_dependencies(test_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#
#add_executable(test_vigir_robot_basic_kf src/test_vigir_robot_basic_kf.cpp ${HEADERS})
#target_link_libraries(test_vigir_robot_basic_kf ${catkin_LIBRARIES} ${PROJECT_NAME} )
#add_dependencies(test_vigir_robot_basic_kf ${catkin_EXPORTED_TARGETS})
#
#add_executable(test_vigir_robot_pose_filter src/test_vigir_robot_pose_filter.cpp ${HEADERS})
#target_link_libraries(test_vigir_robot_pose_filter ${catkin_LIBRARIES} ${PROJECT_NAME} )
#add_dependencies(test_vigir_robot_pose_filter ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############
install( TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Only used for testing
#install(DIRECTORY launch
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  	FILES_MATCHING PATTERN "*.launch"
#)


