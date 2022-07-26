cmake_minimum_required(VERSION 2.8.3)
project(talker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services

## Generate added messages and services

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    std_msgs

)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
