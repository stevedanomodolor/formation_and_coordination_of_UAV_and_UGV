cmake_minimum_required(VERSION 3.0.2)
project(coordination_formation_control_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  crazyswarm
  geometry_msgs
  roscpp
  dynamic_reconfigure
  message_generation
  rosbag
)
find_package(catkin)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)
find_package (Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
  cfg/missionController.cfg
#   cfg/DynReconf2.cfg
 )

add_message_files(
  FILES
  coordination.msg
  waypoint.msg
  logData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coordination_formation_control_pkg
 CATKIN_DEPENDS crazyswarm geometry_msgs roscpp dynamic_reconfigure roscpp crazyswarm message_runtime rosbag
#  DEPENDS system_lib
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(mission_controller src/mission_controller.cpp src/robot_state.cpp)
add_dependencies(mission_controller ${PROJECT_NAME}_gencfg)
target_link_libraries(mission_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES}   ${OpenCV_LIBS})

add_executable(drone_node src/drone_node.cpp src/robot_state.cpp src/swamControllerAlg.cpp)
target_link_libraries(drone_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES}   ${OpenCV_LIBS})

add_executable(omniwheel_node src/omniwheel_node.cpp src/robot_state.cpp src/swamControllerAlg.cpp)
target_link_libraries(omniwheel_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES}   ${OpenCV_LIBS})

add_executable(testbag src/testbag.cpp )
target_link_libraries(testbag ${catkin_LIBRARIES} ${Eigen_LIBRARIES}   ${OpenCV_LIBS})


add_executable(test_controller src/test_controller.cpp src/robot_state.cpp src/swamControllerAlg.cpp)
target_link_libraries(test_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES}   ${OpenCV_LIBS})

install(DIRECTORY
launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/model_ugv_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
