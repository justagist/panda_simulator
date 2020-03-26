cmake_minimum_required(VERSION 2.8.3)
project(panda_sim_moveit)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  panda_moveit_config
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  panda_moveit_config
)

