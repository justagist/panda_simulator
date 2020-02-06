# Panda Simulator [![Build Status](https://travis-ci.org/justagist/panda_simulator.svg?branch=master)](https://travis-ci.org/justagist/panda_simulator)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

A **Gazebo simulator** for the Franka Emika Panda robot with ROS interface, providing exposed **controllers** and real-time **robot state feedback** similar to the real robot when using the [*franka-ros*][franka-ros] package.
## Features
  - Low-level *controllers* (joint position, velocity, torque) available that can be controlled through ROS topics (including position control for gripper).
  - Real-time *robot state* (end-effector state, joint state, controller state, etc.) available through ROS topics.
  - The [*franka_ros_interface*][fri-repo] package (which is a ROS interface for controlling the real Panda robot) can also be used with the panda_simulator, providing direct *sim-to-real* code transfer.
  
  ### Continuous Integration Builds
  
  ROS Kinetic (master branch) | ROS Melodic (melodic-devel branch)
----------- | ----------- 
[![Build Status](https://travis-ci.org/justagist/panda_simulator.svg?branch=master)](https://travis-ci.org/justagist/panda_simulator) | [![Build Status](https://travis-ci.org/justagist/panda_simulator.svg?branch=melodic-devel)](https://travis-ci.org/justagist/panda_simulator)
  
  ![](_extra/panda_simulator.gif)
 Watch video [here](https://www.youtube.com/watch?v=NdSbXC0r7tU).
 
 
  ### Dependencies

 - *libfranka* (`sudo apt install ros-<version>-libfranka` or [install from source][libfranka-doc])
 - *franka-ros* (`sudo apt install ros-<version>-franka-ros` or [install from source][libfranka-doc])
 - [*franka_ros_interface*][fri-repo] (at least the *franka_core_msgs* package from *franka_ros_interface* should be installed)
 - [*franka_panda_description*][fpd-repo] (urdf and model files from *panda_description* package modified to work in Gazebo, and with the custom controllers)
 - [*sns_ik*](https://github.com/RethinkRobotics-opensource/sns_ik)
 
### Installation
Once the above dependencies are installed, the package can be installed using catkin_make:

    $ cd <catkin_ws>
    $ git clone https://github.com/justagist/panda_simulator src/panda_simulator
    $ catkin_make
    $ source devel/setup.bash
 
### Usage

The simulator can be started by running:
    
    $ roslaunch panda_gazebo panda_world.launch
    
This exposes a variety of ROS topics and services for communicating with and controlling the robot in simulation.

A demo node 'move_robot.py' is provided demonstrating (i) controlling the robot (ii) retrieving state information of the robot. 

#### Some useful ROS topics

##### Published Topics:
| ROS Topic | Data |
| ------ | ------ |
| */panda_simulator/custom_franka_state_controller/robot_state* | gravity, coriolis, jacobian, cartesian velocity, etc. |
| */panda_simulator/custom_franka_state_controller/tip_state* | end-effector pose, wrench, etc. |
| */panda_simulator/joint_states* | joint positions, velocities, efforts |

##### Subscribed Topics:
| ROS Topic | Data |
| ------ | ------ |
| */panda_simulator/motion_controller/arm/joint_commands* | command the robot using the currently active controller |
| */panda_simulator/franka_gripper/move* | (action msg) command the joints of the gripper |

Other topics for changing the controller gains (also dynamically configurable), command timeout, etc. are also available.

#### ROS Services:
Controller manager service can be used to switch between all available controllers (joint position, velocity, effort). Gripper joints can be controlled using the ROS ActionClient (via the same topics as the real robot and [*franka_ros*][franka-ros]).

## Related Packages

- [*franka_ros_interface*][fri-repo] : A ROS API for controlling and managing the Franka Emika Panda robot (real and simulated). Contains controllers for the robot (joint position, velocity, torque), interfaces for the gripper, controller manager, coordinate frames interface, etc.. Provides almost complete sim-to-real transfer of code.
- [*panda_robot*](https://github.com/justagist/panda_robot) : Python interface providing higher-level control of the robot integrated with its gripper, controller manager, coordinate frames manager, etc. It also provides access to the kinematics and dynamics of the robot using the [KDL library](http://wiki.ros.org/kdl).

The [*franka_ros_interface*][fri-repo] package provides Python API and interface tools to control and communicate with the robot using the ROS topics and services exposed by the simulator. Since the simulator exposes similar information and controllers as the *robot_state_controller_node* of the [*franka_ros_interface*][fri-repo], the API can be used to control both the real robot, and the simulated robot in this package, with minimum change in code.

#### License
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)


   [fri-repo]: <https://github.com/justagist/franka_ros_interface>
   [fpd-repo]: <https://github.com/justagist/franka_panda_description>
   [libfranka-doc]: <https://frankaemika.github.io/docs/installation_linux.html#building-from-source>
   [franka-ros]: <https://frankaemika.github.io/docs/franka_ros.html>
   
   
   
