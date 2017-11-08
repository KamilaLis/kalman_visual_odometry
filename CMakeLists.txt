project(monocamera_visual_odometry_kalman_filter)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

##############
## Packages ##
##############

## Catkin ##

find_package(catkin REQUIRED COMPONENTS
  roscpp
  viso2_ros
  libviso2
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES visual_odom
  CATKIN_DEPENDS roscpp viso2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Locations ##

include_directories(
  ${libviso2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include/
)

## Libraries ##

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/component_filter.cpp
)

#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

## Executables ##

add_executable(monocamera_visual_odometry_kalman_filter_node
  src/monocamera_visual_odometry_kalman_filter_node.cpp
)

target_link_libraries(monocamera_visual_odometry_kalman_filter_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

