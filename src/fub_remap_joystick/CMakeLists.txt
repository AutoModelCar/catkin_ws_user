cmake_minimum_required(VERSION 2.8.3)
project(fub_remap_joystick)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################
## include common CMake setup ##
##############################
find_package(cmake_modules REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})
###################################
## catkin specific configuration ##
###################################

catkin_package(
#	INCLUDE_DIRS include
#	LIBRARIES ...
    CATKIN_DEPENDS 
    			roscpp
                nodelet
                sensor_msgs
    			std_msgs
)


############################
## Build node(let) FlexibleUnitAstar ##
############################
add_library(${PROJECT_NAME}_nodelet
        src/remap_joystick.cpp
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}
        src/main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES} ${PROJECT_NAME}_nodelet
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_nodelet
        ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(
        TARGETS
                ${PROJECT_NAME}
                ${PROJECT_NAME}_nodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#	FILES_MATCHING PATTERN "*.h"
#	PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
        DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
        FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

