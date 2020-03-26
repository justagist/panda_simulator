cmake_minimum_required(VERSION 2.8.3)
project(panda_simulator_examples)

find_package(catkin REQUIRED COMPONENTS
    rospy
    franka_core_msgs
    gazebo_msgs
    franka_moveit
    panda_sim_moveit
    panda_gazebo
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    franka_core_msgs
    gazebo_msgs
    franka_moveit
    panda_sim_moveit
    panda_gazebo
)

install(PROGRAMS
   scripts/move_robot.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

