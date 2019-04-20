cmake_minimum_required(VERSION 2.8.3)
project(duckiepond)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Heading.msg
  MotorCmd.msg
  Motor4Cmd.msg
  VelocityVector.msg
  Box.msg
  Boxlist.msg
  BoolStamped.msg
  UsvDrive.msg
)

add_service_files(
  FILES
  SetValue.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES duckiepond
#  CATKIN_DEPENDS geometry_msgs message_generation roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

