cmake_minimum_required(VERSION 2.8.3)
project(crab_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  hebiros
  crab_description
  drivebot_description
  hebihexapod_description
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crab_gazebo
#  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp rospy
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
