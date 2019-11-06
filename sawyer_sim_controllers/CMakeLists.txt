cmake_minimum_required(VERSION 2.8.3)
project(sawyer_sim_controllers)

## Add support for C++14, supported in ROS Kinetic and newer
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  effort_controllers
  roscpp
  roslint
  intera_core_msgs
  realtime_tools
  sawyer_hardware_interface
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sawyer_sim_controllers
  CATKIN_DEPENDS sawyer_hardware_interface roscpp
)
roslint_cpp()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/sawyer_position_controller.cpp
  src/sawyer_velocity_controller.cpp
  src/sawyer_effort_controller.cpp
  src/sawyer_gravity_controller.cpp
  # src/electric_gripper_controller.cpp
  ## Copies of upstream ros_controllers/effort_controllers:
  #  (remove once inheritable change is upstreamed)
  src/sawyer_joint_position_controller.cpp
  src/sawyer_joint_velocity_controller.cpp
  src/sawyer_joint_effort_controller.cpp
  # src/sawyer_head_controller.cpp
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
