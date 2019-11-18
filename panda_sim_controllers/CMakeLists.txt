cmake_minimum_required(VERSION 2.8.3)
project(panda_sim_controllers)

## Add support for C++14, supported in ROS Kinetic and newer
add_definitions(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  effort_controllers
  roscpp
  roslint
  franka_core_msgs
  franka_gripper
  actionlib
  realtime_tools
  panda_hardware_interface
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES panda_sim_controllers
  CATKIN_DEPENDS panda_hardware_interface roscpp actionlib actionlib_msgs
  # DEPENDS Franka
)
roslint_cpp()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/panda_position_controller.cpp
  src/panda_velocity_controller.cpp
  src/panda_effort_controller.cpp
  src/panda_gravity_controller.cpp
  src/panda_joint_position_controller.cpp
  src/panda_joint_velocity_controller.cpp
  src/panda_joint_effort_controller.cpp
  src/panda_gripper_controller.cpp
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY config launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
