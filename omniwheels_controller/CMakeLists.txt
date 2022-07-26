cmake_minimum_required(VERSION 2.8.3)
project(omniwheels_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs message_generation)
add_message_files(
 FILES
 motorCommand.msg
#   Message2.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/omni_wheel_controller_node.py scripts/crazyflie_logger_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



