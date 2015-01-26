cmake_minimum_required(VERSION 2.8.3)
project(vigir_humanoid_controller)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
set(ROS_BUILD_TYPE RelWithDebInfo)

# Using std::atomic
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Eigen REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs eigen_stl_containers eigen_conversions hardware_interface controller_manager flor_utilities vigir_robot_model vigir_humanoid_interfaces)

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
include/${PROJECT_NAME}/VigirRealTimeBuffer.h
include/${PROJECT_NAME}/VigirRealTimePublisher.h
include/${PROJECT_NAME}/VigirHumanoidStatusCodes.h
include/${PROJECT_NAME}/VigirHumanoidInterface.h
include/${PROJECT_NAME}/VigirHumanoidHWInterface.h
include/${PROJECT_NAME}/VigirHumanoidController.h
)

SET(SOURCE
src/VigirHumanoidHWInterface.cpp
src/VigirHumanoidController.cpp
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
  LIBRARIES vigir_humanoid_controller
  CATKIN_DEPENDS vigir_robot_model vigir_humanoid_interfaces flor_utilities roscpp hardware_interface controller_manager
  DEPENDS eigen
)

###########
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
add_executable(test_${PROJECT_NAME} src/test_vigir_humanoid_controller.cpp ${HEADERS})
target_link_libraries(test_${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME} )
add_dependencies(test_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


install( TARGETS ${PROJECT_NAME} 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(DIRECTORY launch
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  	FILES_MATCHING PATTERN "*.*"
#)


