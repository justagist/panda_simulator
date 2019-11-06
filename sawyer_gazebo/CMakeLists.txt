cmake_minimum_required(VERSION 3.0.2)
project(sawyer_gazebo)

find_package(catkin
  REQUIRED COMPONENTS
    sawyer_hardware_interface
    cv_bridge
    gazebo_ros
    gazebo_ros_control
    image_transport
    intera_core_msgs
    realtime_tools
    roscpp
    roslint
    std_msgs
    kdl_parser
    tf_conversions
    sns_ik_lib
)

# Depend on system install of Gazebo
find_package(GAZEBO REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
 CATKIN_DEPENDS
  sawyer_hardware_interface
  gazebo_ros_control
  image_transport
  intera_core_msgs
  realtime_tools
  roscpp
  kdl_parser
  tf_conversions
  sns_ik_lib
 INCLUDE_DIRS include
 LIBRARIES    sawyer_robot_hw_sim
)

roslint_cpp()
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(sawyer_gazebo_ros_control
  src/sawyer_gazebo_ros_control_plugin.cpp
  # src/assembly_interface.cpp
  src/arm_controller_interface.cpp
  src/arm_kinematics_interface.cpp
  # src/head_interface.cpp
)
set_property(TARGET sawyer_gazebo_ros_control PROPERTY CXX_STANDARD 14)

target_link_libraries(sawyer_gazebo_ros_control
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_library(sawyer_robot_hw_sim src/sawyer_robot_hw_sim.cpp)
target_link_libraries(sawyer_robot_hw_sim
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(
  sawyer_gazebo_ros_control
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS sawyer_gazebo_ros_control sawyer_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES sawyer_robot_hw_sim_plugins.xml
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
