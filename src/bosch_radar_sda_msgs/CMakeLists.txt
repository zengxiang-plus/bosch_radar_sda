cmake_minimum_required(VERSION 3.5.1)

project(bosch_radar_sda_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CR5TPRadarParsed.msg
  CR5TPRadarParsedArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS std_msgs message_runtime sensor_msgs
  INCLUDE_DIRS
  LIBRARIES
)
