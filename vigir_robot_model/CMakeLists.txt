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
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs flor_utilities urdf eigen_stl_containers eigen_conversions pcl_ros rbdl )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


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
include/${PROJECT_NAME}/VigirRobotFilter.h
include/${PROJECT_NAME}/VigirRobotKalmanFilterBase.h
include/${PROJECT_NAME}/VigirRobotBasicKF.h
)

SET(SOURCE
src/VigirRobotState.cpp
src/VigirRobotModel.cpp
src/VigirRobotRBDLModel.cpp
src/VigirRobotKalmanFilterBase.cpp
src/VigirRobotBasicKF.cpp
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
  LIBRARIES Vigir_robot_model
#  CATKIN_DEPENDS other_catkin_pkg
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

## Declare a cpp executable
add_executable(test_${PROJECT_NAME} src/test_vigir_robot_model.cpp ${HEADERS})
target_link_libraries(test_${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME} )

add_dependencies(test_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(test_vigir_robot_basic_kf src/test_vigir_robot_basic_kf.cpp ${HEADERS})
target_link_libraries(test_vigir_robot_basic_kf ${catkin_LIBRARIES} ${PROJECT_NAME} )

add_dependencies(test_vigir_robot_basic_kf ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS flor_dynamics flor_dynamics_test
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
# catkin_add_gtest(${PROJECT_NAME}-test test/flor_dynamics_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)