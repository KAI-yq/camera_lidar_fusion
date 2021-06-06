cmake_minimum_required(VERSION 2.8.3)
project(camera_lidar_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  image_transport
  pcl_ros
  cv_bridge
  message_filters
  sensor_msgs
)

find_package(PCL REQUIRED)
find_package(OpenCV)
find_package(Threads)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(fusion src/fusion.cpp src/common.h)
target_link_libraries(fusion ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})


