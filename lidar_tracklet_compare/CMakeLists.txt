cmake_minimum_required(VERSION 2.8.12)
project(lidar_tracklet_compare)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  eigen_conversions
  image_geometry
  image_transport
  jsk_recognition_utils
  jsk_topic_tools
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else ()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(PkgConfig REQUIRED)

catkin_package()

#fusion Library
add_library(lidar_tracklet_compare_lib SHARED
  include/lidar_tracklet_compare/lidar_tracklet_compare.h
  src/lidar_tracklet_compare.cpp
)

target_include_directories(lidar_tracklet_compare_lib PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(lidar_tracklet_compare_lib
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(lidar_tracklet_compare_lib
  ${catkin_EXPORTED_TARGETS}
)

#Fusion node
add_executable(lidar_tracklet_compare
  src/lidar_tracklet_compare_node.cpp
)
target_include_directories(lidar_tracklet_compare PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(lidar_tracklet_compare
  lidar_tracklet_compare_lib
)

install(
  TARGETS lidar_tracklet_compare lidar_tracklet_compare_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
