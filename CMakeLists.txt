cmake_minimum_required(VERSION 2.8.3)
project(pcl_multitask)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  roslaunch
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_multitask
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp roslaunch rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/detect_rain.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(detect_rain_node src/detect_rain_node.cc)
add_executable(calib_lidar2car_byground_node src/calib_lidar2car_byground.cc)
add_executable(bag2pcd_node src/bag2pcd.cc)
add_executable(bag2pcd_leishen_node src/bag2pcd_leishen.cc)
add_executable(buildMapWithGPS_node src/buildMapWithGPS.cc)
add_executable(genBagImagePointcloud_node src/genBagImagePointcloud.cc)
add_executable(genNavMap_node src/genNavMap.cc)
add_executable(localize_indoor_node src/localize_indoor.cc)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(detect_rain_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(calib_lidar2car_byground_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(bag2pcd_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)

target_link_libraries(bag2pcd_leishen_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(buildMapWithGPS_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(genBagImagePointcloud_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(genNavMap_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(localize_indoor_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)