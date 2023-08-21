cmake_minimum_required(VERSION 3.0.2)
project(ni_wrapper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# add_message_files(DIRECTORY msg FILES Floats.msg HeaderString.msg)
# add_service_files(DIRECTORY srv FILES AddTwoInts.srv BadTwoInts.srv)
# generate_messages(DEPENDENCIES std_msgs)
# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ni_wrapper
 CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# foreach(T
#     001_talker_listener)
#   install(DIRECTORY ${T}
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#     USE_SOURCE_PERMISSIONS)
# endforeach()