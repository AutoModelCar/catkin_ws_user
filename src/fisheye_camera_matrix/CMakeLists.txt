cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3 ${CMAKE_CXX_FLAGS}" )
project( fisheye_camera_matrix )

find_package( catkin REQUIRED COMPONENTS
  roscpp cv_bridge image_transport geometry_msgs message_generation fisheye_camera_matrix_msgs
)

find_package( OpenCV REQUIRED )

add_service_files( FILES
  perspective_projection.srv
)

generate_messages( DEPENDENCIES std_msgs geometry_msgs )

include_directories(
  include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS cv_bridge image_transport geometry_msgs message_runtime fisheye_camera_matrix_msgs
)

add_executable( offline_calibration_manual src/offline_calibration_manual.cpp src/camera_matrix.cpp )
target_link_libraries( offline_calibration_manual ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( camera_matrix_publisher src/camera_matrix_publisher.cpp src/camera_matrix.cpp )
target_link_libraries( camera_matrix_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( undistorted_image_publisher src/undistorted_image_publisher.cpp src/camera_matrix.cpp )
target_link_libraries( undistorted_image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_library( fisheye_camera_matrix src/camera_matrix.cpp )
target_link_libraries( fisheye_camera_matrix ${catkin_LIBRARIES} ${OpenCV_LIBS} )

install(
  TARGETS ${PROJECT_NAME} offline_calibration_manual camera_matrix_publisher undistorted_image_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install( DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)