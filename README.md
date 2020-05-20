# Panda Simulator [![Build Status](https://travis-ci.org/justagist/panda_simulator.svg?branch=master)](https://travis-ci.org/justagist/panda_simulator) [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3747459.svg)](https://doi.org/10.5281/zenodo.3747459)

Latest version: [![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/justagist/panda_simulator?include_prereleases&style=flat)](https://github.com/justagist/panda_simulator/tags)

Latest Stable Release: [![GitHub release (latest by date)](https://img.shields.io/github/v/release/justagist/panda_simulator?style=flat)](https://github.com/justagist/panda_simulator/tags)

*(version 1.0.0)* - **Now Supports [MoveIt!](https://moveit.ros.org/)** (See [version log](https://github.com/justagist/panda_simulator/blob/melodic-devel/versionLog.md) for details)

A **Gazebo simulator** for the Franka Emika Panda robot with ROS interface, providing exposed **controllers** and real-time **robot state feedback** similar to the real robot when using the [*franka-ros*][franka-ros] package.

## Features

- Low-level *controllers* (joint position, velocity, torque) available that can be controlled through ROS topics (including position control for gripper).
- Real-time *robot state* (end-effector state, joint state, controller state, etc.) available through ROS topics.
- The [*franka_ros_interface*][fri-repo] package (which is a ROS interface for controlling the real Panda robot) can also be used with the panda_simulator, providing kinematics and dynamics computation for the robot, and direct *sim-to-real* code transfer.
- Supports MoveIt planning and control for Franka Panda Emika robot and arm and Franka Gripper.
  
### Continuous Integration Builds

ROS Melodic (melodic-devel branch): [![Build Status](https://travis-ci.org/justagist/panda_simulator.svg?branch=melodic-devel)](https://travis-ci.org/justagist/panda_simulator)

ROS Kinetic (kinetic-devel branch): [![Build Status](https://travis-ci.org/justagist/panda_simulator.svg?branch=kinetic-devel)](https://travis-ci.org/justagist/panda_simulator)
  
  ![vid](_extra/panda_simulator.gif)
 Watch video [here](https://www.youtube.com/watch?v=NdSbXC0r7tU).

### Dependencies

- *libfranka* (`apt install ros-${ROS_DISTRO}-libfranka` or [install from source][libfranka-doc])
- *franka-ros* (`apt install ros-${ROS_DISTRO}-franka-ros` or [install from source][libfranka-doc])
- `pip install -r requirements.txt`

For other possible missing dependencies, check the `apt` packages specified in Dockerfile.

The following dependencies can be installed using the `.rosinstall` file (instructions in next section)

- [*franka_ros_interface*][fri-repo]
- [*franka_panda_description*][fpd-repo] (urdf and model files from *panda_description* package modified to work in Gazebo, and with the custom controllers)
- [*orocos-kinematics-dynamics*](https://github.com/orocos/orocos_kinematics_dynamics)

### Installation

1.Clone the repo:

```bash
    cd <catkin_ws>/src
    git clone https://github.com/justagist/panda_simulator
```

Steps 2 and 3 can be automated by running `./build_ws.sh` from `<catkin_ws>/src/panda_simulator`.

2.Update dependency packages:

```bash
    wstool init
    wstool merge panda_simulator/dependencies.rosinstall
    wstool up

    # use old ros-compatible version of kdl
    cd orocos_kinematics_dynamics && git checkout b35c424e77ebc5b7e6f1c5e5c34f8a4666fbf5bc
    cd ../.. && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

3.Once the dependencies are met, the package can be installed using catkin_make:

```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    catkin build # if catkin not found, install catkin tools (apt install python-catkin-tools)
    source devel/setup.bash
```

### Docker Build

**Requires [nvidia-docker](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0))**

To build the docker image of the package, run `docker build docker/ -t ps_melodic:v1.0.0`, or pull built image from github (`docker pull docker.pkg.github.com/justagist/panda_simulator/ps_melodic:v1.0.0`).

To run the built image interactively, use the script `run_docker.sh`. The container starts in a catkin workspace (directory location in host machine: `$HOME/.panda_sim_melodic_ws`). When running for the first time, the catkin workspace has to be built (`cd src/panda_simulator && ./build_ws.sh`). Any edits made to this directory in the host machine will be reflected in the docker container (and vice-versa). If everything was successfully built in the previous step, you should be able to run the simulator (see [Usage](#usage) section below). This also allows for running ROS nodes and scripts without having any ROS installation on the host machine.

### Usage

The simulator can be started by running:

```bash
    roslaunch panda_gazebo panda_world.launch
```

This exposes a variety of ROS topics and services for communicating with and controlling the robot in simulation.

A demo node 'move_robot.py' is provided demonstrating (i) controlling the robot (ii) retrieving state information of the robot.

#### Some useful ROS topics

##### Published Topics

| ROS Topic | Data |
| ------ | ------ |
| */panda_simulator/custom_franka_state_controller/robot_state* | gravity, coriolis, jacobian, cartesian velocity, etc. |
| */panda_simulator/custom_franka_state_controller/tip_state* | end-effector pose, wrench, etc. |
| */panda_simulator/joint_states* | joint positions, velocities, efforts |

##### Subscribed Topics

| ROS Topic | Data |
| ------ | ------ |
| */panda_simulator/motion_controller/arm/joint_commands* | command the robot using the currently active controller |
| */panda_simulator/franka_gripper/move* | (action msg) command the joints of the gripper |

Other topics for changing the controller gains (also dynamically configurable), command timeout, etc. are also available.

#### ROS Services

Controller manager service can be used to switch between all available controllers (joint position, velocity, effort). Gripper joints can be controlled using the ROS ActionClient (via the same topics as the real robot and [*franka_ros*][franka-ros]).

## Related Packages

- [*franka_ros_interface*][fri-repo] : A ROS API for controlling and managing the Franka Emika Panda robot (real and simulated). Contains controllers for the robot (joint position, velocity, torque), interfaces for the gripper, controller manager, coordinate frames interface, etc.. Provides almost complete sim-to-real transfer of code.
- [*panda_robot*](https://github.com/justagist/panda_robot) : Python interface providing higher-level control of the robot integrated with its gripper, controller manager, coordinate frames manager, etc. It also provides access to the kinematics and dynamics of the robot using the [KDL library](http://wiki.ros.org/kdl).

The [*franka_ros_interface*][fri-repo] package provides Python API and interface tools to control and communicate with the robot using the ROS topics and services exposed by the simulator. Since the simulator exposes similar information and controllers as the *robot_state_controller_node* of the [*franka_ros_interface*][fri-repo], the API can be used to control both the real robot, and the simulated robot in this package, with minimum change in code.

### Version Update

Check [versionLog.md](https://github.com/justagist/panda_simulator/blob/melodic-devel/versionLog.md).

### License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Copyright (c) 2019-2020, Saif Sidhik

If you use this software, please cite it using [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3747459.svg)](https://doi.org/10.5281/zenodo.3747459).

   [fri-repo]: <https://github.com/justagist/franka_ros_interface>
   [fpd-repo]: <https://github.com/justagist/franka_panda_description>
   [libfranka-doc]: <https://frankaemika.github.io/docs/installation_linux.html#building-from-source>
   [franka-ros]: <https://frankaemika.github.io/docs/franka_ros.html>
