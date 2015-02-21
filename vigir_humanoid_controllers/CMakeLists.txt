cmake_minimum_required(VERSION 2.8.3)
project(vigir_humanoid_controllers)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Eigen REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  joint_trajectory_controller
  vigir_humanoid_interfaces
  controller_interface
  eigen_stl_containers
  eigen_conversions
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
  CATKIN_DEPENDS roscpp joint_trajectory_controller vigir_humanoid_interfaces controller_interface
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
include/${PROJECT_NAME}/vigir_controller_controller_base.h
include/${PROJECT_NAME}/vigir_gravity_compensation_controller.h
include/${PROJECT_NAME}/vigir_inverse_dynamics_controller.h
include/${PROJECT_NAME}/vigir_friction_compensation_controller.h
)

SET(SOURCE
src/vigir_joint_trajectory_controller.cpp
src/vigir_pelvis_trajectory_controller.cpp
src/vigir_gravity_compensation_controller.cpp
src/vigir_inverse_dynamics_controller.cpp
src/vigir_friction_compensation_controller.cpp
)

add_library(${PROJECT_NAME} ${SOURCE} ${HEADERS} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
install( TARGETS ${PROJECT_NAME} 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
	vigir_humanoid_controllers_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

