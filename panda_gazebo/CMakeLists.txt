cmake_minimum_required(VERSION 3.0.2)
project(panda_gazebo)

find_package(catkin
  REQUIRED COMPONENTS
    panda_hardware_interface
    cv_bridge
    gazebo_ros
    gazebo_ros_control
    image_transport
    franka_core_msgs
    realtime_tools
    roscpp
    roslint
    std_msgs
    kdl_parser
    tf_conversions
    orocos_kdl
    panda_sim_controllers
    panda_sim_custom_action_server
)

# Depend on system install of Gazebo
find_package(GAZEBO REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
find_package(Eigen3 REQUIRED)

catkin_package(
 CATKIN_DEPENDS
  panda_hardware_interface
  gazebo_ros_control
  image_transport
  franka_core_msgs
  realtime_tools
  roscpp
  kdl_parser
  tf_conversions
  panda_sim_controllers
  panda_sim_custom_action_server
 INCLUDE_DIRS include
 LIBRARIES panda_robot_hw_sim
)

roslint_cpp()
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(panda_gazebo_ros_control
  src/panda_gazebo_ros_control_plugin.cpp
  src/arm_controller_interface.cpp
  src/arm_kinematics_interface.cpp
  src/kdl_methods.cpp
)
set_property(TARGET panda_gazebo_ros_control PROPERTY CXX_STANDARD 14)

target_link_libraries(panda_gazebo_ros_control
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_library(panda_robot_hw_sim src/panda_robot_hw_sim.cpp)
target_link_libraries(panda_robot_hw_sim
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(
  panda_gazebo_ros_control
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS panda_gazebo_ros_control panda_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES panda_robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY config launch worlds share
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

catkin_python_setup()
catkin_install_python(PROGRAMS startup_scripts/force_neutral_pose.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
