cmake_minimum_required(VERSION 3.0.2)
project(omni3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_ros
  controller_manager
  robot_state_publisher
  teleop_twist_keyboard
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs gazebo_ros controller_manager robot_state_publisher teleop_twist_keyboard
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/keyboard_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
