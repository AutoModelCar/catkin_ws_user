cmake_minimum_required(VERSION 2.8.3)
project(fub_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        ecl_geometry
        fub_trajectory_msgs
        geometry_msgs
        nav_msgs
        nodelet
        roscpp
        std_msgs
        tf
)

# include common CMake setup
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
        # Fallback to cmake_modules for Ubuntu Trusty  / ROS Indigo
        find_package(cmake_modules REQUIRED)
        find_package(Eigen REQUIRED)
        set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

generate_dynamic_reconfigure_options(
        cfg/ControllerMig.cfg
        cfg/ControllerMigPath.cfg
        cfg/ControllerMigSteer.cfg
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#        INCLUDE_DIRS include
#        LIBRARIES ...
        CATKIN_DEPENDS
                dynamic_reconfigure
                fub_trajectory_msgs
                geometry_msgs
                nav_msgs
                std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
        "src/"
)
include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)


#########################
## Build ControllerMig ##
#########################

## Declare a cpp executable
add_executable(${PROJECT_NAME}_mig
        src/controller_mig/main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_mig
        ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_mig_nodelet
        src/controller_mig/ControllerMig.cpp
        src/controller_mig/PathFollower.cpp
        src/controller_mig/SteerController.cpp
        src/controller_mig/VehicleState.cpp
        src/util/Math.cpp
        src/util/Spline.cpp
)

# make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME}_mig
        ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_mig_nodelet
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
            ${PROJECT_NAME}_mig
            ${PROJECT_NAME}_mig_nodelet
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

#############
## Testing ##
#############


## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
	catkin_add_gtest(${PROJECT_NAME}-test
		tests/TestSteeringAndWheelRatio.cpp
		tests/testMath.cpp
		src/util/Math.cpp
		src/controller_mig/VehicleState.cpp
	)

	target_link_libraries(${PROJECT_NAME}-test
		pthread
		${GTEST_BOTH_LIBRARIES}
		${catkin_LIBRARIES}
	)
endif()
