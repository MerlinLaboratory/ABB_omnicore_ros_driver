cmake_minimum_required(VERSION 3.0.2)
project(irim_ss_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  #moveit-ros-control-interface
  #joint-trajectory-controller
  #gripper-action-controller
  #effort-controllers
  rospy
  moveit_commander
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES irim_ss_pkg
#  CATKIN_DEPENDS cv_bridge moveit-commander rospy
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/move_group_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/vision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/dice.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/gripper_simulation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

