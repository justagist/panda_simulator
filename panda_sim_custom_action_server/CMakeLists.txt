cmake_minimum_required(VERSION 2.8.3)
project(panda_sim_custom_action_server)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  franka_core_msgs
  franka_interface
  panda_moveit_config
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/PandaPositionFFJointTrajectoryActionServer.cfg
  )

add_dependencies(${PROJECT_NAME}_gencfg franka_core_msgs_generate_messages_py)

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  franka_core_msgs
  franka_interface
  panda_moveit_config
)

catkin_install_python(PROGRAMS scripts/start_gripper_action_server.py
scripts/start_joint_trajectory_server.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
